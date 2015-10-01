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
 * i2c_unit.cpp
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
#include <stm32f7xx_hal_i2c.h>
#include <stm32f7xx_hal_gpio.h>
#include <hal-utils.h>
#include <bspopts.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>
#include <dev/i2c/i2c.h>
#include <linux/i2c.h>
#include <hal-i2c.h>
}

#define I2C_ADDRESS 0xE6

/* ############################ Global Variables ############################ */

/* I2C Data Buffer */
uint8_t I2CMSgBuffer[2] = {0,0};

/* I2C Config struct */
struct i2c_config {
    char name[16];
    int handle;
    I2C_Instance instance;
};

/* I2C Message */
i2c_msg i2c_msgg;

static i2c_config i2c_buses[MAX_I2C_INSTANCES];
static int num_i2c_buses = 0;

/* ######################## Internal Functions ###$$$######################## */

/**
 * @brief Returns the registered I2C device driver number by name
 */
int find_i2c_bus_by_name(const char* i2c_bus_name) {

  uint32_t i;

  for(i = 0; i < MAX_I2C_INSTANCES; i++){
    if(strcmp(i2c_buses[i].name, i2c_bus_name) == 0){
      return i2c_buses[i].handle;
    }
  }

  return 0;
}

/* ########################### Test Harnesses ############################### */

/**
 * @brief Test harness group
 */
TEST_GROUP(hal_i2c_fit)
{
  void setup() {

    uint32_t i;

    for(i = 0; i < COUNTOF(i2c_buses); i++){
      snprintf((char*)i2c_buses[num_i2c_buses].name, sizeof(i2c_buses[num_i2c_buses].name), "/dev/i2c%lu", i+1);

      if( access( i2c_buses[num_i2c_buses].name, F_OK ) != -1 ) {

        i2c_buses[num_i2c_buses].handle = open((char*)i2c_buses[num_i2c_buses].name, O_RDWR | O_APPEND);

        // If everything worked correctly then increment the number of buses
        if(i2c_buses[num_i2c_buses].handle > 0) {
          num_i2c_buses++;
          i2c_buses[num_i2c_buses].instance = (I2C_Instance ) i;
        }
      }
    }
  }

  void teardown() {

    int i;

    // close all i2c buses
    for(i  = 0; i < num_i2c_buses; i++) {
      close(i2c_buses[i].handle);
      i2c_buses[i].handle = 0;
      i2c_buses[i].name[0] = '\0';
    }
    num_i2c_buses = 0;
  }
};

/**
 *  @brief Check for a single write in I2C Bus
 */
TEST(hal_i2c_fit, check_i2c_write) {

  /* Check if number of buses is not zero */
  CHECK_TEXT(num_i2c_buses != 0, "Number of buses is zero");

  i2c_msgg.addr = I2C_ADDRESS;
  i2c_msgg.len = 2;
  i2c_msgg.buf = &I2CMSgBuffer[0];

 int count = 0;
 int ret_val = 0;

 for( count = 0; count < num_i2c_buses; count++ )
   {
     CHECK_TEXT(i2c_buses[count].handle != 0, "Invalid can bus handle");

     ret_val = write(i2c_buses[count].handle, &i2c_msgg, i2c_msgg.len);

     // Make sure that the number of bytes written == the size of the message
     CHECK_TEXT(ret_val == i2c_msgg.len, "Invalid return value write");
   }
}

/**
 * @brief Validates that i2c1 is visible in file system
 *    if it has been configured in the configure.ac file
 */
#if STM32_ENABLE_I2C1
TEST(hal_i2c_fit, check_i2c1) {

  CHECK_TEXT(find_i2c_bus_by_name("/dev/i2c1") > 0, "Failed to open /dev/i2c1");

}
#endif

/**
 * @brief Validates that i2c2 is visible in file system
 *    if it has been configured in the configure.ac file
 */
#if STM32_ENABLE_I2C2
TEST(hal_i2c_fit, check_i2c2) {

  CHECK_TEXT(find_i2c_bus_by_name("/dev/i2c2") > 0, "Failed to open /dev/i2c2");

}
#endif

/**
 * @brief Validates that i2c3 is visible in file system
 *    if it has been configured in the configure.ac file
 */
#if STM32_ENABLE_I2C3
TEST(hal_i2c_fit, check_i2c3) {

  CHECK_TEXT(find_i2c_bus_by_name("/dev/i2c3") > 0, "Failed to open /dev/i2c3");
}
#endif


/**
 * @brief Validates that i2c4 is visible in file system
 *    if it has been configured in the configure.ac file
 */
#if STM32_ENABLE_I2C4
TEST(hal_i2c_fit, check_i2c4) {

  CHECK_TEXT(find_i2c_bus_by_name("/dev/i2c4") > 0, "Failed to open /dev/i2c4");
}
#endif

/**
 *  @brief Try to close the driver again and see if it returns error code
 */
TEST(hal_i2c_fit, i2c_close_again) {

  int count = 0;
  int ret_val = 0;

  for( count = 0; count < num_i2c_buses; count++)
    {
      ret_val = close(i2c_buses[count].handle);
      CHECK_TEXT(ret_val != 0, "Error - Able to close() again. This should return error");
    }
}

/**
 * @brief Try to open the driver again and see if it returns error code
 */
TEST(hal_i2c_fit, i2c_open_again) {

  int count = 0;
  int handle = 0;

  for( count = 0; count < num_i2c_buses; count++)
    {
      handle = open((char*)i2c_buses[num_i2c_buses].name, O_RDWR | O_APPEND);
      CHECK_TEXT(handle != 0, "Error - Able to open() again. This should return error");
    }
}
