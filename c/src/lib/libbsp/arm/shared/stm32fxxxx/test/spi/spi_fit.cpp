/**
 * NOTICE: This software  source code and any of  its derivatives are the
 * confidential  and  proprietary   information  of  Vecna  Technologies,
 * Inc. (such source  and its derivatives are hereinafter  referred to as
 * "Confidential Information"). The  Confidential Information is intended
 * to be  used exclusively by  individuals or entities that  have entered
 * into either  a non-disclosure agreement or license  agreement (or both
 * of  these agreements,  if  applicable) with  Vecna Technologies,  Inc.
 * ("Vecna")   regarding  the  use   of  the   Confidential  Information.
 * Furthermore,  the  Confidential  Information  shall be  used  only  in
 * accordance  with   the  terms   of  such  license   or  non-disclosure
 * agreements.   All  parties using  the  Confidential Information  shall
 * verify that their  intended use of the Confidential  Information is in
 * compliance  with and  not in  violation of  any applicable  license or
 * non-disclosure  agreements.  Unless expressly  authorized by  Vecna in
 * writing, the Confidential Information  shall not be printed, retained,
 * copied, or  otherwise disseminated,  in part or  whole.  Additionally,
 * any party using the Confidential  Information shall be held liable for
 * any and  all damages incurred  by Vecna due  to any disclosure  of the
 * Confidential  Information (including  accidental disclosure).   In the
 * event that  the applicable  non-disclosure or license  agreements with
 * Vecna  have  expired, or  if  none  currently  exists, all  copies  of
 * Confidential Information in your  possession, whether in electronic or
 * printed  form, shall be  destroyed or  returned to  Vecna immediately.
 * Vecna  makes no  representations  or warranties  hereby regarding  the
 * suitability  of  the   Confidential  Information,  either  express  or
 * implied,  including  but not  limited  to  the  implied warranties  of
 * merchantability,    fitness    for    a   particular    purpose,    or
 * non-infringement. Vecna  shall not be liable for  any damages suffered
 * by  licensee as  a result  of  using, modifying  or distributing  this
 * Confidential Information.  Please email [info@vecnatech.com]  with any
 * questions regarding the use of the Confidential Information.
 */

/*
 * spi_unit.cpp
 *
 *  @author sudarshan.rajagopalan
 *
 *  Change Log:
 *
 */

#include <CppUTest/TestHarness.h>
#include <stdio.h>


extern "C" {
#include <stm32f7xx_hal_dma.h>
#include <stm32f7xx_hal_spi.h>
#include <stm32f7xx_hal_gpio.h>
#include <hal-utils.h>
#include <bspopts.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>
#include <hal-spi.h>
}

/* ############################ Global Variables ############################ */
/* Buffer used for transmission */
uint8_t aTxBuffer;

/* Buffer used for reception */
uint8_t aRxBuffer;

/* Slave Select GPIO Port*/
gpio acc_ss =
    {
        .port = PORTE,
        .pin = GPIO_PIN_3,
    };

uint8_t colors[3 * 16];
uint8_t zeros[3 * 16];

/* SPI Config struct */
struct spi_config {
    char name[16];
    int handle;
    SPI_Instance instance;
};

/* SPI Message */
spi_msg spy =
  {
  .pRxBuf = (uint8_t *)&aRxBuffer,
  .pTxBuf = (uint8_t *)&colors,
  .flags = SPI_M_WR,
  .len = (3 * 16),
  .slave_select = acc_ss
  };

static spi_config spi_buses[MAX_SPI_INSTANCES];
static int num_spi_buses = 0;

/* ######################## Internal Functions ###$$$######################## */

/**
 * @brief Returns the registered SPI device driver number by name
 */
int find_spi_bus_by_name(const char* spi_bus_name) {

  uint32_t i;

  for(i = 0; i < MAX_SPI_INSTANCES; i++){
    if(strcmp(spi_buses[i].name, spi_bus_name) == 0){
      return spi_buses[i].handle;
    }
  }

  return 0;
}

/**
 *  @brief Checks all IOCTL commands
 */
void spi_fit_ioctl(spi_config* pBus)
{

  int ret_val;

  // Check that handle is non-null
  CHECK_TEXT(pBus->handle != 0, "Invalid spi bus handle");

  /* Change to 2-Line half duplex */
  ret_val = ioctl(pBus->handle, SPI_IOCTL_DIR_2LINE, NULL);
  CHECK_TEXT(ret_val == 0, "Invalid return value ioctl command for SPI_IOCTL_DIR_2LINE");

  /* Change to Slave Mode */
  ret_val = ioctl(pBus->handle, SPI_IOCTL_MODE_SLAVE, NULL);
  CHECK_TEXT(ret_val == 0, "Invalid return value ioctl command for SPI_IOCTL_MODE_SLAVE");

  /* Change to Master Mode */
  ret_val = ioctl(pBus->handle, SPI_IOCTL_MODE_MASTER, NULL);
  CHECK_TEXT(ret_val == 0, "Invalid return value ioctl command for SPI_IOCTL_MODE_MASTER");

  /* Change to 1-line half duplex */
  ret_val = ioctl(pBus->handle, SPI_IOCTL_DIR_1LINE, NULL);
  CHECK_TEXT(ret_val == 0, "Invalid return value ioctl command for SPI_IOCTL_DIR_1LINE");

  /* Change to 2-Line half duplex */
  ret_val = ioctl(pBus->handle, SPI_IOCTL_ENABLE_CRC, NULL);
  CHECK_TEXT(ret_val == 0, "Invalid return value ioctl command for SPI_IOCTL_ENABLE_CRC");

  /* Change to 1-line half duplex */
  ret_val = ioctl(pBus->handle, SPI_IOCTL_DISABLE_CRC, NULL);
  CHECK_TEXT(ret_val == 0, "Invalid return value ioctl command for SPI_IOCTL_DISABLE_CRC");

  /* Check for invalid ioctl command*/
  ret_val = ioctl(pBus->handle, 0xBADA55, NULL);
  CHECK_TEXT(ret_val == -1, "Error - IOCTL for an invalid command passed!!");

}

/* ########################### Test Harnesses ############################### */

/**
 * @brief Test harness group
 */
TEST_GROUP(hal_spi_fit)
{
  void setup() {

    uint32_t i;

    for(i = 0; i < COUNTOF(spi_buses); i++){
      snprintf((char*)spi_buses[num_spi_buses].name, sizeof(spi_buses[num_spi_buses].name), "/dev/spi%lu", i+1);

      if( access( spi_buses[num_spi_buses].name, F_OK ) != -1 ) {

        spi_buses[num_spi_buses].handle = open((char*)spi_buses[num_spi_buses].name, O_RDWR | O_APPEND);

        // If everything worked correctly then increment the number of buses
        if(spi_buses[num_spi_buses].handle > 0) {
          num_spi_buses++;
          spi_buses[num_spi_buses].instance = (SPI_Instance ) i;
        }
      }
    }
  }

  void teardown() {

    int i;

    // close all spi buses
    for(i  = 0; i < num_spi_buses; i++) {
      close(spi_buses[i].handle);
      spi_buses[i].handle = 0;
      spi_buses[i].name[0] = '\0';
    }
    num_spi_buses = 0;
  }
};


/**
 *  @brief Check all the IOCTL commands
 */
TEST(hal_spi_fit, spi_ioctl) {
  int count = 0;

  for( count=0; count<num_spi_buses; count++)
    {
      spi_fit_ioctl((spi_config*) &(spi_buses[count]));
    }
}

/**
 *  @brief Check for a single write in SPI Bus
 */
TEST(hal_spi_fit, check_spi_write) {

  /* Check if number of buses is not zero */
  CHECK_TEXT(num_spi_buses != 0, "Number of buses is zero");

 int count = 0;
 int ret_val = 0;

 for( count = 0; count < num_spi_buses; count++ )
   {
     CHECK_TEXT(spi_buses[count].handle != 0, "Invalid can bus handle");

     ret_val = write(spi_buses[count].handle, &spy, spy.len);

     // Make sure that the number of bytes written == the size of the message
     CHECK_TEXT(ret_val == spy.len, "Invalid return value write");
   }
}


/**
 * @brief Validates that spi1 is visible in file system
 *    if it has been configured in the configure.ac file
 */
#if STM32_ENABLE_SPI1
TEST(hal_spi_fit, check_spi1) {

  CHECK_TEXT(find_spi_bus_by_name("/dev/spi1") > 0, "Failed to open /dev/spi1");

}
#endif

/**
 * @brief Validates that spi2 is visible in file system
 *    if it has been configured in the configure.ac file
 */
#if STM32_ENABLE_SPI2
TEST(hal_spi_fit, check_spi2) {

  CHECK_TEXT(find_spi_bus_by_name("/dev/spi2") > 0, "Failed to open /dev/spi2");

}
#endif

/**
 * @brief Validates that spi3 is visible in file system
 *    if it has been configured in the configure.ac file
 */
#if STM32_ENABLE_SPI3
TEST(hal_spi_fit, check_spi3) {

  CHECK_TEXT(find_spi_bus_by_name("/dev/spi3") > 0, "Failed to open /dev/spi3");
}
#endif


/**
 * @brief Validates that spi4 is visible in file system
 *    if it has been configured in the configure.ac file
 */
#if STM32_ENABLE_SPI4
TEST(hal_spi_fit, check_spi4) {

  CHECK_TEXT(find_spi_bus_by_name("/dev/spi4") > 0, "Failed to open /dev/spi4");
}
#endif


/**
 * @brief Validates that spi5 is visible in file system
 *    if it has been configured in the configure.ac file
 */
#if STM32_ENABLE_SPI5
TEST(hal_spi_fit, check_spi5) {

  CHECK_TEXT(find_spi_bus_by_name("/dev/spi5") > 0, "Failed to open /dev/spi5");

}
#endif


/**
 *  @brief Try to close the driver again and see if it returns error code
 */
TEST(hal_spi_fit, spi_close_again) {

  int count = 0;
  int ret_val = 0;

  for( count = 0; count < num_spi_buses; count++)
    {
      ret_val = close(spi_buses[count].handle);
      CHECK_TEXT(ret_val != 0, "Error - Able to close() again. This should return error");
    }
}

/**
 * @brief Try to open the driver again and see if it returns error code
 */
TEST(hal_spi_fit, spi_open_again) {

  int count = 0;
  int handle = 0;

  for( count = 0; count < num_spi_buses; count++)
    {
      handle = open((char*)spi_buses[num_spi_buses].name, O_RDWR | O_APPEND);
      CHECK_TEXT(handle != 0, "Error - Able to open() again. This return error");
    }
}






