/**
 * @file iap_flashif.c
 *
 * @ingroup iap
 *
 * @brief External flash writing interface and Intel HEX parsing
 *
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * Author: Sudarshan Rajagopalan
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */


/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <stdbool.h>
#include <hal-utils.h>
#include <bspopts.h>
#include <stm32f-processor-specific.h>
#include <rtems/irq-extension.h>

#include <hal-qspi.h>
#include "iap_flashif.h"
#include "iap_tftpserver.h"

#include stm_processor_header( TARGET_STM_PROCESSOR_PREFIX )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, flash )

#define QSPI_BASE_ADDRESS       0x90000000

#define QSPI_FLASH      true
#define IHEX_PARSING    true

extern tftp_firmware_image_info firmware_info;
extern uint32_t rom_buf_start;

volatile uint32_t timestamp=0;
volatile uint32_t time_start=0;
volatile uint32_t time_end=0;

volatile  uint32_t numbytes_till_page_end;
volatile  uint32_t numbytes_to_write;

uint32_t base_address=0;
uint32_t address_to_flash=0;
char record_buffer[100];
uint16_t record_buffer_index = 0;

bool incomplete_record = FALSE;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

#if IHEX_PARSING
void stm32_ihex_process_record (void)
{

  uint8_t num_bytes_written = 0;
  uint8_t sum_data = 0;
  char two_byte_data[2 + 3] = {0};
  char four_byte_addr[4 + 3] = {0};
  bool record_error = false;

  stm32_qspi_command command;

  int i=0;

  uint8_t data[256] = {0};

  uint8_t data_byte_count = 0;

  uint8_t byte_count = 0;
  uint8_t record_type = 0;
  uint8_t checksum = 0;

  uint16_t address = 0;

  /* Assert if not a valid record */
  _Assert(
           (record_buffer[0] == ':')
           && (record_buffer[record_buffer_index-1] == '\n')
           && (record_buffer[record_buffer_index-2] == '\r')
           && (record_buffer[record_buffer_index-3] == '\0')
           && (record_buffer[record_buffer_index-4] == '\r')
          );

  two_byte_data[0] = '0';
  two_byte_data[1] = 'X';
  two_byte_data[4] = ' ';

  four_byte_addr[0] = '0';
  four_byte_addr[1] = 'X';
  four_byte_addr[6] = ' ';

  /* Tow byte byte count */
  two_byte_data[2] = record_buffer[1];
  two_byte_data[3] = record_buffer[2];
  byte_count = strtoul(two_byte_data, NULL, 16);

  /* four byte address */
  four_byte_addr[2] = record_buffer[3];
  four_byte_addr[3] = record_buffer[4];
  four_byte_addr[4] = record_buffer[5];
  four_byte_addr[5] = record_buffer[6];
  address = strtoul(four_byte_addr, NULL, 16);

  /* two byte record type */
  two_byte_data[2] = record_buffer[7];
  two_byte_data[3] = record_buffer[8];
  record_type = strtoul(two_byte_data, NULL, 16);

  /* Extract data in byte format */
  for(data_byte_count = 0; data_byte_count < byte_count; data_byte_count++)
    {
      two_byte_data[2] = record_buffer[9 + data_byte_count + i];
      two_byte_data[3] = record_buffer[10 + data_byte_count + i];
      i++;

      data[data_byte_count] = strtoul(two_byte_data, NULL, 16);

      /* Calculate sum of data bytes. To be used in checksum error verification*/
      sum_data = sum_data + data[data_byte_count];
    }

  /* two byte checksum */
  two_byte_data[2] = record_buffer[record_buffer_index-6];
  two_byte_data[3] = record_buffer[record_buffer_index-5];
  checksum = strtoul(two_byte_data, NULL, 16);

  /* Verify record for errors using checksum.
   *   Start code + Byte count + Address + Record type + Data  =  two's complement of Checksum
   */
  record_error = ( (uint8_t)(byte_count + (uint8_t)(address%0x100) + (uint8_t)(address/0x100) + record_type + sum_data) == (uint8_t)( (checksum^0xff) + 1 ) );

  if (record_error == false)
    {
      firmware_info.firmware_corrupted = true;
      firmware_info.firmware_flashed = false;
      stm32_ethernet_iap_update_firmware_info();
      __asm__ volatile ("BKPT #01"); /* Break debugger here */
    }

  /* If record type is Extended Linear Address (0x4), then update the base address */
  if( record_type == 0x04)      // Extended Address record type
    {
      four_byte_addr[2] = record_buffer[9];
      four_byte_addr[3] = record_buffer[10];
      four_byte_addr[4] = record_buffer[11];
      four_byte_addr[5] = record_buffer[12];

      base_address = strtoul(four_byte_addr, NULL, 16);
      base_address = base_address * 0x10000;

    }

  /* If record type is Data, the flash the data to the appropriate address */
  else if (record_type == 0)    //Data record type
    {
      address_to_flash = base_address + address;

#if QSPI_FLASH
      /* Get the offset address from QSPI base. QSPI controller expects
       * the offset address from the base address region to flash
       */
      address_to_flash = address_to_flash - QSPI_BASE_ADDRESS;
#endif

      numbytes_till_page_end = QSPI_PAGE_SIZE - ( address_to_flash % QSPI_PAGE_SIZE );

      /* See if this fits entirely in one page */
      numbytes_to_write = (data_byte_count < numbytes_till_page_end ? data_byte_count : numbytes_till_page_end);

      num_bytes_written = 0;

      do
        {
          command.instruction = QUAD_IN_FAST_PROG_CMD;
          command.addr = address_to_flash;
          command.num_bytes = numbytes_to_write;
          command.pBuf = (uint8_t*)(&(data[num_bytes_written]));

          stm32_qspi_write(command);

          num_bytes_written = num_bytes_written + command.num_bytes;

          address_to_flash = address_to_flash + command.num_bytes;

          data_byte_count = data_byte_count - command.num_bytes;

          numbytes_to_write = (data_byte_count < QSPI_PAGE_SIZE ? data_byte_count : QSPI_PAGE_SIZE);

        } while( data_byte_count != 0 );

    }

  /* Reset the buffer index for use for next record */
  record_buffer_index = 0;
}
#endif /* IHEX_PARSING */

/**
  * @brief  Unlocks Flash for write access
  * @param  None
  * @retval None
  */
void FLASH_If_Init(void)
{ 

  /* Initialize QSPI */
  stm32_qspi_init();
}

/**
  * @brief  This function does an erase of all user flash area
  * @param  StartSector: start of user flash area
  * @retval 0: user flash area successfully erased
  *         1: error occurred
  */
int8_t FLASH_If_Erase(uint32_t StartSector)
{

  stm32_qspi_command command;

  command.instruction   = BULK_ERASE_CMD; //TODO:: Use BULK_ERASE command
  command.addr          = 0;

  stm32_qspi_erase(command);

  return (0);
}

uint32_t FLASH_If_Write( uint32_t* FlashAddress, uint32_t* Data ,uint16_t DataLength)
{

#if IHEX_PARSING

  /* Define boundaries for the transferred block */
  uint32_t* block_start_addr = Data;
  uint32_t* block_end_addr = block_start_addr + (DataLength);

  char *temp_ptr = block_start_addr;

  if(incomplete_record)
    {

      while(*temp_ptr != ':')
        {
          record_buffer[record_buffer_index] = *temp_ptr;
          record_buffer_index++;
          temp_ptr++;
        }

      /* Process the record and flash data */
      stm32_ihex_process_record();

      incomplete_record = FALSE;
    }

  /* Assert if temp_ptr is not pointing to ':' and we missed some data */
  if(*temp_ptr != ':')
    {
      firmware_info.firmware_corrupted = true;
      firmware_info.firmware_flashed = false;
      stm32_ethernet_iap_update_firmware_info();
      __asm__ volatile ("BKPT #01"); /* Break debugger here */
    }

  do
    {
      /* Store ':' into start of buffer */
      record_buffer[record_buffer_index] = *temp_ptr;
      record_buffer_index++;
      temp_ptr++;

      while(*temp_ptr != ':')
        {
          record_buffer[record_buffer_index] = *temp_ptr;

          if( temp_ptr+1 >  block_end_addr)
            {
              incomplete_record = TRUE;
              goto block_exit_label;
            }
          else
            {
              record_buffer_index++;
              temp_ptr++;
            }
        }

      /* Process the record and flash data */
      stm32_ihex_process_record();

    } while(temp_ptr < (char *)block_end_addr);


  block_exit_label:

  time_end = rtems_clock_get_ticks_since_boot();
  timestamp = time_end - time_start;

#else // Else do Binary Parsing

  uint32_t ptr = 0;
  uint32_t numbytes = DataLength * 4;

  stm32_qspi_command command;

  numbytes_till_page_end = QSPI_PAGE_SIZE - ( *FlashAddress % QSPI_PAGE_SIZE );

  /* See if this fits entirely in one page */
  numbytes_to_write = (numbytes < numbytes_till_page_end ? numbytes : numbytes_till_page_end);


  /* First write till page end */
  do
    {
      command.instruction = QUAD_IN_FAST_PROG_CMD;
      command.addr = *FlashAddress;
      command.num_bytes = numbytes_to_write;
      command.pBuf = (uint32_t*)(Data+ptr);

      stm32_qspi_write(command);

      /* Increment FLASH destination address */
      *FlashAddress = (*FlashAddress) + numbytes_to_write;

      ptr = ptr + (numbytes_to_write / 4);

      numbytes = numbytes - numbytes_to_write;

      numbytes_to_write = (numbytes < QSPI_PAGE_SIZE ? numbytes : QSPI_PAGE_SIZE);

    } while( numbytes != 0 );

#endif

  return 0;
}


void stm32_ethernet_iap_update_firmware_info(void)
{
  stm32_flash_if_init();
  stm32_flash_if_erase((uint32_t) &rom_buf_start);
  stm32_flash_if_write((uint8_t*)&rom_buf_start, (uint8_t*)&firmware_info, sizeof(firmware_info));
  stm32_flash_if_lock();

}
