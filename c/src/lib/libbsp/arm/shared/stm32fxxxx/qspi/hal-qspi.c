/**
 * @file hal-qspi.c
 *
 * @ingroup qspi
 *
 *
 * @brief QSPI driver for the STM32xxxx series processors. Provides a provides
 * high level routines to manage external Flash programming (erase and write).
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

#include <rtems.h>
#include <stdlib.h>
#include <stdbool.h>
#include <hal-utils.h>
#include <stm32f-processor-specific.h>
#include <bspopts.h>
#include <errno.h>

#include stm_processor_header( TARGET_STM_PROCESSOR_PREFIX )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, dma )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, gpio )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, rcc )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, qspi )

#include <hal-qspi.h>
#include <rtems/irq-extension.h>

QSPI_HandleTypeDef qspi_handle;
QSPI_CommandTypeDef      qspi_command;
QSPI_MemoryMappedTypeDef qspi_mem_mapped_cfg;


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Definition for QSPI clock resources */
#define QSPI_CLK_ENABLE()          __HAL_RCC_QSPI_CLK_ENABLE()
#define QSPI_CLK_DISABLE()         __HAL_RCC_QSPI_CLK_DISABLE()
#define QSPI_CS_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()
#define QSPI_CLK_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define QSPI_D0_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOD_CLK_ENABLE()
#define QSPI_D1_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOD_CLK_ENABLE()
#define QSPI_D2_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOE_CLK_ENABLE()
#define QSPI_D3_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOD_CLK_ENABLE()
#define QSPI_DMA_CLK_ENABLE()      __HAL_RCC_DMA2_CLK_ENABLE()

#define QSPI_FORCE_RESET()         __HAL_RCC_QSPI_FORCE_RESET()
#define QSPI_RELEASE_RESET()       __HAL_RCC_QSPI_RELEASE_RESET()

/* Definition for QSPI Pins */
#define QSPI_CS_PIN                GPIO_PIN_6
#define QSPI_CS_GPIO_PORT          GPIOB
#define GPIO_AF_CS                 GPIO_AF10_QUADSPI

#define QSPI_CLK_PIN               GPIO_PIN_2
#define QSPI_CLK_GPIO_PORT         GPIOB
#define GPIO_AF_CLK                GPIO_AF9_QUADSPI

#define QSPI_D0_PIN                GPIO_PIN_11
#define QSPI_D0_GPIO_PORT          GPIOD
#define GPIO_AF_D0                 GPIO_AF9_QUADSPI

#define QSPI_D1_PIN                GPIO_PIN_12
#define QSPI_D1_GPIO_PORT          GPIOD
#define GPIO_AF_D1                 GPIO_AF9_QUADSPI

#define QSPI_D2_PIN                GPIO_PIN_2
#define QSPI_D2_GPIO_PORT          GPIOE
#define GPIO_AF_D2                 GPIO_AF9_QUADSPI

#define QSPI_D3_PIN                GPIO_PIN_13
#define QSPI_D3_GPIO_PORT          GPIOD
#define GPIO_AF_D3                 GPIO_AF9_QUADSPI

/* Definition for QSPI DMA */
#define QSPI_DMA_INSTANCE          DMA2_Stream7
#define QSPI_DMA_CHANNEL           DMA_CHANNEL_3
#define QSPI_DMA_IRQ               DMA2_Stream7_IRQn
#define QSPI_DMA_IRQ_HANDLER       DMA2_Stream7_IRQHandler

/* N25Q512A Micron memory */
/* Size of the flash */
#define QSPI_FLASH_SIZE                      23
#define QSPI_PAGE_SIZE                       256

/* Reset Operations */
#define RESET_ENABLE_CMD                     0x66
#define RESET_MEMORY_CMD                     0x99

/* Identification Operations */
#define READ_ID_CMD                          0x9E
#define READ_ID_CMD2                         0x9F
#define MULTIPLE_IO_READ_ID_CMD              0xAF
#define READ_SERIAL_FLASH_DISCO_PARAM_CMD    0x5A

/* Read Operations */
#define READ_CMD                             0x03
#define READ_4_BYTE_ADDR_CMD                 0x13

#define FAST_READ_CMD                        0x0B
#define FAST_READ_DTR_CMD                    0x0D
#define FAST_READ_4_BYTE_ADDR_CMD            0x0C

#define DUAL_OUT_FAST_READ_CMD               0x3B
#define DUAL_OUT_FAST_READ_DTR_CMD           0x3D
#define DUAL_OUT_FAST_READ_4_BYTE_ADDR_CMD   0x3C

#define DUAL_INOUT_FAST_READ_CMD             0xBB
#define DUAL_INOUT_FAST_READ_DTR_CMD         0xBD
#define DUAL_INOUT_FAST_READ_4_BYTE_ADDR_CMD 0xBC

#define QUAD_OUT_FAST_READ_CMD               0x6B
#define QUAD_OUT_FAST_READ_DTR_CMD           0x6D
#define QUAD_OUT_FAST_READ_4_BYTE_ADDR_CMD   0x6C

#define QUAD_INOUT_FAST_READ_CMD             0xEB
#define QUAD_INOUT_FAST_READ_DTR_CMD         0xED
#define QUAD_INOUT_FAST_READ_4_BYTE_ADDR_CMD 0xEC

/* Write Operations */
#define WRITE_ENABLE_CMD                     0x06
#define WRITE_DISABLE_CMD                    0x04

/* Register Operations */
#define READ_STATUS_REG_CMD                  0x05
#define WRITE_STATUS_REG_CMD                 0x01

#define READ_LOCK_REG_CMD                    0xE8
#define WRITE_LOCK_REG_CMD                   0xE5

#define READ_FLAG_STATUS_REG_CMD             0x70
#define CLEAR_FLAG_STATUS_REG_CMD            0x50

#define READ_NONVOL_CFG_REG_CMD              0xB5
#define WRITE_NONVOL_CFG_REG_CMD             0xB1

#define READ_VOL_CFG_REG_CMD                 0x85
#define WRITE_VOL_CFG_REG_CMD                0x81

#define READ_ENHANCED_VOL_CFG_REG_CMD        0x65
#define WRITE_ENHANCED_VOL_CFG_REG_CMD       0x61

#define READ_EXT_ADDR_REG_CMD                0xC8
#define WRITE_EXT_ADDR_REG_CMD               0xC5

/* Program Operations */
#define PAGE_PROG_CMD                        0x02
#define PAGE_PROG_4_BYTE_ADDR_CMD            0x12

#define DUAL_IN_FAST_PROG_CMD                0xA2
#define EXT_DUAL_IN_FAST_PROG_CMD            0xD2

#define QUAD_IN_FAST_PROG_CMD                0x32
#define EXT_QUAD_IN_FAST_PROG_CMD            0x12 /*0x38*/
#define QUAD_IN_FAST_PROG_4_BYTE_ADDR_CMD    0x34

/* Erase Operations */
#define SUBSECTOR_ERASE_CMD                  0x20
#define SUBSECTOR_ERASE_4_BYTE_ADDR_CMD      0x21

#define SECTOR_ERASE_CMD                     0xD8
#define SECTOR_ERASE_4_BYTE_ADDR_CMD         0xDC

#define BULK_ERASE_CMD                       0xC7

#define PROG_ERASE_RESUME_CMD                0x7A
#define PROG_ERASE_SUSPEND_CMD               0x75

/* One-Time Programmable Operations */
#define READ_OTP_ARRAY_CMD                   0x4B
#define PROG_OTP_ARRAY_CMD                   0x42

/* 4-byte Address Mode Operations */
#define ENTER_4_BYTE_ADDR_MODE_CMD           0xB7
#define EXIT_4_BYTE_ADDR_MODE_CMD            0xE9

/* Quad Operations */
#define ENTER_QUAD_CMD                       0x35
#define EXIT_QUAD_CMD                        0xF5

/* Default dummy clocks cycles */
#define DUMMY_CLOCK_CYCLES_READ              8
#define DUMMY_CLOCK_CYCLES_READ_QUAD         10

#define DUMMY_CLOCK_CYCLES_READ_DTR          6
#define DUMMY_CLOCK_CYCLES_READ_QUAD_DTR     8

/* End address of the QSPI memory */
#define QSPI_END_ADDR              (1 << QSPI_FLASH_SIZE)


static bool qspi_initialized = false;


/**
  * @brief  This function handles QUADSPI interrupt request.
  * @param  None
  * @retval None
  */
void QUADSPI_IRQHandler(void)
{
  HAL_QSPI_IRQHandler(&qspi_handle);
}

void DMA2_Stream7_IRQHandler (void)
{

}

void HAL_QSPI_MspInit(QSPI_HandleTypeDef *hqspi)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  static DMA_HandleTypeDef hdma;
  rtems_status_code sc;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable the QuadSPI memory interface clock */
  QSPI_CLK_ENABLE();
  /* Reset the QuadSPI memory interface */
  QSPI_FORCE_RESET();
  QSPI_RELEASE_RESET();
  /* Enable GPIO clocks */
  QSPI_CS_GPIO_CLK_ENABLE();
  QSPI_CLK_GPIO_CLK_ENABLE();
  QSPI_D0_GPIO_CLK_ENABLE();
  QSPI_D1_GPIO_CLK_ENABLE();
  QSPI_D2_GPIO_CLK_ENABLE();
  QSPI_D3_GPIO_CLK_ENABLE();
  /* Enable DMA clock */
  QSPI_DMA_CLK_ENABLE();

  /*##-2- Configure peripheral GPIO ##########################################*/
  /* QSPI CS GPIO pin configuration  */
  GPIO_InitStruct.Pin       = QSPI_CS_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF_CS;
  HAL_GPIO_Init(QSPI_CS_GPIO_PORT, &GPIO_InitStruct);

  /* QSPI CLK GPIO pin configuration  */
  GPIO_InitStruct.Pin       = QSPI_CLK_PIN;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Alternate = GPIO_AF_CLK;
  HAL_GPIO_Init(QSPI_CLK_GPIO_PORT, &GPIO_InitStruct);

  /* QSPI D0 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = QSPI_D0_PIN;
  GPIO_InitStruct.Alternate = GPIO_AF_D0;
  HAL_GPIO_Init(QSPI_D0_GPIO_PORT, &GPIO_InitStruct);

  /* QSPI D1 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = QSPI_D1_PIN;
  GPIO_InitStruct.Alternate = GPIO_AF_D1;
  HAL_GPIO_Init(QSPI_D1_GPIO_PORT, &GPIO_InitStruct);

  /* QSPI D2 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = QSPI_D2_PIN;
  GPIO_InitStruct.Alternate = GPIO_AF_D2;
  HAL_GPIO_Init(QSPI_D2_GPIO_PORT, &GPIO_InitStruct);

  /* QSPI D3 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = QSPI_D3_PIN;
  GPIO_InitStruct.Alternate = GPIO_AF_D3;
  HAL_GPIO_Init(QSPI_D3_GPIO_PORT, &GPIO_InitStruct);

  /*##-3- Configure the NVIC for QSPI #########################################*/
  /* Install SPI vectors */
  sc = rtems_interrupt_handler_install(
      QUADSPI_IRQn,
    "QSPI Event Interrupt",
    RTEMS_INTERRUPT_UNIQUE,
    QUADSPI_IRQHandler,
    NULL
       );

  /*##-4- Configure the DMA channel ###########################################*/
  /* QSPI DMA channel configuration */
  hdma.Init.Channel             = QSPI_DMA_CHANNEL;
  hdma.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma.Init.MemInc              = DMA_MINC_ENABLE;
  hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma.Init.Mode                = DMA_NORMAL;
  hdma.Init.Priority            = DMA_PRIORITY_LOW;
  hdma.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;        /* FIFO mode disabled     */
  hdma.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  hdma.Init.MemBurst            = DMA_MBURST_SINGLE;           /* Memory burst           */
  hdma.Init.PeriphBurst         = DMA_PBURST_SINGLE;           /* Peripheral burst       */
  hdma.Instance                 = QSPI_DMA_INSTANCE;

  __HAL_LINKDMA(hqspi, hdma, hdma);
  HAL_DMA_Init(&hdma);

  /* NVIC configuration for DMA interrupt */
  /* NVIC configuration for DMA interrupt */
  sc = rtems_interrupt_handler_install(
      DMA2_Stream7_IRQn,
    "DMA2 Channel 7 Event Interrupt",
    RTEMS_INTERRUPT_UNIQUE,
    DMA2_Stream7_IRQHandler,
    NULL
       );
}

/**
 * @brief  Initializes the QSPI instance.
 * @param  None
 * @retval  RTEMS Status Code
 * @Note  De-initializes any previously initialized qspi instance
 */
rtems_status_code stm32_qspi_init(void)
{
  rtems_status_code sc;

  qspi_handle.Instance = QUADSPI;

  /* De-initialize any previous instance */
  HAL_QSPI_DeInit(&qspi_handle);

  /* Set the QSPI clock and other parameters */
  qspi_handle.Init.ClockPrescaler     = 2;
  qspi_handle.Init.FifoThreshold      = 4;
  qspi_handle.Init.SampleShifting     = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  qspi_handle.Init.FlashSize          = POSITION_VAL(0x1000000) - 1;
  qspi_handle.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_2_CYCLE;
  qspi_handle.Init.ClockMode          = QSPI_CLOCK_MODE_0;
  qspi_handle.Init.FlashID            = QSPI_FLASH_ID_1;
  qspi_handle.Init.DualFlash          = QSPI_DUALFLASH_DISABLE;

  /* Populate the QSPI command structure */
  qspi_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  qspi_command.AddressSize       = QSPI_ADDRESS_24_BITS;
  qspi_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  qspi_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  qspi_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  qspi_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Initialize the QSPI Instance */
  if (HAL_QSPI_Init(&qspi_handle) != HAL_OK)
    {
      qspi_initialized = false;
      ////rtems_set_errno_and_return_minus_one( EIO );
    }

  qspi_initialized = true;

  return 0;
}

/**
  * @brief  This function sends a Write Enable and waits until it is effective.
  * @param  hqspi: QSPI handle
  * @retval None
  */
static void stm32_qspi_write_enable(void)
{
  QSPI_CommandTypeDef     sCommand;
  QSPI_AutoPollingTypeDef sConfig;

  /* Assert if QSPI is not initialized */
  _Assert(qspi_initialized == true);

  /* Enable write operations ------------------------------------------ */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = WRITE_ENABLE_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_NONE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(&qspi_handle, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
      //rtems_set_errno_and_return_minus_one( EIO );
  }

  /* Configure automatic polling mode to wait for write enabling ---- */
  sConfig.Match           = 0x02;
  sConfig.Mask            = 0x02;
  sConfig.MatchMode       = QSPI_MATCH_MODE_AND;
  sConfig.StatusBytesSize = 1;
  sConfig.Interval        = 0x10;
  sConfig.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  sCommand.Instruction    = READ_STATUS_REG_CMD;
  sCommand.DataMode       = QSPI_DATA_1_LINE;

  if (HAL_QSPI_AutoPolling(&qspi_handle, &sCommand, &sConfig, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
      //rtems_set_errno_and_return_minus_one( EIO );
  }
}

/**
  * @brief  This function reads the SR of the memory and awaits the EOP.
  * @param  hqspi: QSPI handle
  * @retval None
  */
static void stm32_qspi_auto_polling_mode(void)
{
  QSPI_CommandTypeDef     sCommand;
  QSPI_AutoPollingTypeDef sConfig;

  /* Assert if QSPI is not initialized */
  _Assert(qspi_initialized == true);

  /* Configure automatic polling mode to wait for memory ready ------ */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = READ_STATUS_REG_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_1_LINE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  sConfig.Match           = 0x00;
  sConfig.Mask            = 0x01;
  sConfig.MatchMode       = QSPI_MATCH_MODE_AND;
  sConfig.StatusBytesSize = 1;
  sConfig.Interval        = 0x10;
  sConfig.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  if (HAL_QSPI_AutoPolling_IT(&qspi_handle, &sCommand, &sConfig) != HAL_OK)
  {
      //rtems_set_errno_and_return_minus_one( EIO );
  }
}

/**
  * @brief  This function configures the dummy cycles on memory side.
  * @param  hqspi: QSPI handle
  * @retval None
  */
static void stm32_qspi_config_dummy_cycles(void)
{
  QSPI_CommandTypeDef sCommand;
  uint8_t reg;

  /* Assert if QSPI is not initialized */
  _Assert(qspi_initialized == true);

  /* Read Volatile Configuration register --------------------------- */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = READ_VOL_CFG_REG_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_1_LINE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode         = QSPI_SIOO_INST_EVERY_CMD;
  sCommand.NbData            = 1;

  if (HAL_QSPI_Command(&qspi_handle, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
      //rtems_set_errno_and_return_minus_one( EIO );
  }

  if (HAL_QSPI_Receive(&qspi_handle, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
      //rtems_set_errno_and_return_minus_one( EIO );
  }

  /* Enable write operations ---------------------------------------- */
  stm32_qspi_write_enable();

  /* Write Volatile Configuration register (with new dummy cycles) -- */
  sCommand.Instruction = WRITE_VOL_CFG_REG_CMD;
  MODIFY_REG(reg, 0xF0, (DUMMY_CLOCK_CYCLES_READ_QUAD << POSITION_VAL(0xF0)));

  if (HAL_QSPI_Command(&qspi_handle, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
      //rtems_set_errno_and_return_minus_one( EIO );
  }

  if (HAL_QSPI_Transmit(&qspi_handle, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
      //rtems_set_errno_and_return_minus_one( EIO );
  }
}

/**
 * @brief  Erases the QSPI Flash area defined by address.
 * @param  QSPI Command Struct - Command and Address
 * @retval  RTEMS Status Code
 * @note addr can be a NULL pointer.
 */
rtems_status_code stm32_qspi_erase(stm32_qspi_command command)
{
  /* Assert if QSPI is not initialized */
  _Assert(qspi_initialized == true);

  /* Enable Write Operation */
  stm32_qspi_write_enable();

  /* Erasing Sequence -------------------------------------------------- */
  qspi_command.Instruction = command.instruction;
  qspi_command.Address     = command.addr;

  qspi_command.AddressMode = QSPI_ADDRESS_1_LINE;
  qspi_command.DataMode    = QSPI_DATA_NONE;
  qspi_command.DummyCycles = 0;

  if (HAL_QSPI_Command(&qspi_handle, &qspi_command, 100) != HAL_OK)
    {
      //rtems_set_errno_and_return_minus_one( EIO );
    }

  /* Configure automatic polling mode to wait for end of erase */
  stm32_qspi_auto_polling_mode();

  return 0;
}

/**
 * @brief  Write the data pointer by pBuf to the QSPI address.
 * @param  QSPI Command Struct - Command, Address, num of bytes and pointer to
 *                               data buffer.
 * @retval  Number of bytes written.
 * @note addr can be a NULL pointer.
 */
uint32_t stm32_qspi_write(stm32_qspi_command command)
{
  /* Assert if QSPI is not initialized */
  _Assert(qspi_initialized == true);

  /* Enable Write Operation */
  stm32_qspi_write_enable();

  /* Writing Sequence  */
  qspi_command.Instruction = command.instruction;
  qspi_command.Address     = command.addr;
  qspi_command.NbData      = command.num_bytes;

  qspi_command.AddressMode = QSPI_ADDRESS_1_LINE;
  qspi_command.DataMode    = QSPI_DATA_4_LINES;

  if (HAL_QSPI_Command(&qspi_handle, &qspi_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      //rtems_set_errno_and_return_minus_one( EIO );
    }

  if (HAL_QSPI_Transmit(&qspi_handle, (uint8_t*)command.pBuf, 100) != HAL_OK)
    {
      //rtems_set_errno_and_return_minus_one( EIO );
    }

  /* Wait till end of flashing is complete */
  stm32_qspi_auto_polling_mode();

  return qspi_command.NbData;
}


/**
 * @brief  Read from QSPI Address into the data buffer pointer by pBuf.
 * @param  QSPI Command Struct - Command, Address, num of bytes and pointer to
 *                               data buffer.
 * @retval  Number of bytes read.
 * @note addr can be a NULL pointer.
 */
uint32_t stm32_qspi_read(stm32_qspi_command command)
{
  /* Assert if QSPI is not initialized */
  _Assert(qspi_initialized == true);

  /* Configure Volatile Configuration register (with new dummy cycles) */
  stm32_qspi_config_dummy_cycles();

  /* Reading Sequence */
  qspi_command.Instruction = command.instruction;
  qspi_command.Address     = command.addr;
  qspi_command.NbData      = command.num_bytes;

  qspi_command.AddressMode = QSPI_ADDRESS_1_LINE;
  qspi_command.DataMode    = QSPI_DATA_4_LINES;
  qspi_command.DummyCycles = DUMMY_CLOCK_CYCLES_READ_QUAD;

  if (HAL_QSPI_Command(&qspi_handle, &qspi_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      //rtems_set_errno_and_return_minus_one( EIO );
    }

  if (HAL_QSPI_Receive(&qspi_handle, (uint8_t*)command.pBuf, 100) != HAL_OK)
    {
      //rtems_set_errno_and_return_minus_one( EIO );
    }

  /* Wait till end of flashing is complete */
  stm32_qspi_auto_polling_mode();

  return qspi_command.NbData;
}

void stm32_qspi_memory_mapped (void)
{
  /* Assert if QSPI is not initialized */
  _Assert(qspi_initialized == true);

  /* Enable Write Operation */
  stm32_qspi_write_enable();

  qspi_command.Instruction = ENTER_QUAD_CMD;
  qspi_command.AddressMode = QSPI_ADDRESS_1_LINE;
  qspi_command.DataMode    = QSPI_DATA_4_LINES;

  if (HAL_QSPI_Command(&qspi_handle, &qspi_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      //rtems_set_errno_and_return_minus_one( EIO );
    }

  /* Wait till end of flashing is complete */
  stm32_qspi_auto_polling_mode();

  /* Configure Volatile Configuration register (with new dummy cycles)*/
  stm32_qspi_config_dummy_cycles();

  /* Reading Sequence */
  qspi_command.Instruction = QUAD_OUT_FAST_READ_CMD;
  qspi_command.DummyCycles = DUMMY_CLOCK_CYCLES_READ_QUAD;

  qspi_mem_mapped_cfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;

  if (HAL_QSPI_MemoryMapped(&qspi_handle, &qspi_command, &qspi_mem_mapped_cfg) != HAL_OK)
    {
      //rtems_set_errno_and_return_minus_one( EIO );
    }
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
