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
 * can_unit.cpp
 *
 *  @author jay.doyle
 *
 *  Change Log:
 *    Created Apr 30, 2015 - jay.doyle
 */

#include <CppUTest/TestHarness.h>
#include <stdio.h>
#include <hal-utils.h>
#include <hal-can.h>
#include <bspopts.h>
#include <dev/can/can.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>
#include <hal-can.h>

#define MAX_NUMBER_OF_CAN_BUSES 3
#define NUM_TEST_TX_MESSAGES 10

struct can_config {
    char name[16];
    int handle;
    CAN_Instance instance;
};

static can_config can_buses[NUM_CAN_INSTANCES] = {0};
static int num_can_buses = 0;

TEST_GROUP(hal_can_fit)
{
  void setup() {

    uint32_t i;

    for(i = 0; i < COUNTOF(can_buses); i++){
      snprintf((char*)can_buses[num_can_buses].name, sizeof(can_buses[num_can_buses].name), "/dev/can%lu", i+1);

      if( access( can_buses[num_can_buses].name, F_OK ) != -1 ) {

        can_buses[num_can_buses].handle = open((char*)can_buses[num_can_buses].name, O_RDWR | O_APPEND);

        // If everything worked correctly then increment the number of buses
        if(can_buses[num_can_buses].handle > 0) {
          num_can_buses++;
          can_buses[num_can_buses].instance = (CAN_Instance ) i;
        }
      }
    }
  }

  void teardown() {

    int i;

    // close all can buses
    for(i  = 0; i < num_can_buses; i++) {
      close(can_buses[num_can_buses].handle);
      can_buses[num_can_buses].handle = 0;
      can_buses[num_can_buses].name[0] = '\0';
      num_can_buses--;
    }
  }
};

int find_can_bus_by_name(const char* can_bus_name) {

  uint32_t i;

  for(i = 0; i < COUNTOF(can_buses); i++){
    if(strcmp(can_buses[i].name, can_bus_name) == 0){
      return can_buses[i].handle;
    }
  }

  return 0;
}


/**
 * @brief Validates that can1 is visible in file system
 *    if it has been configured in the configure.ac file
 */
TEST(hal_can_fit, check_can1) {

#if STM32_ENABLE_CAN1
  CHECK_TEXT(find_can_bus_by_name("/dev/can1") > 0, "Failed to open /dev/can1");
#endif

}

/**
 * @brief Validates that can2 is visible in file system
 *    if it has been configured in the configure.ac file
 */
TEST(hal_can_fit, check_can2) {

#if STM32_ENABLE_CAN2
  CHECK_TEXT(find_can_bus_by_name("/dev/can2") > 0, "Failed to open /dev/can2");
#endif

}


#if 0
void can_fit_loopback_test(can_config* pBus) {

  uint32_t loopback_mode = CAN_FLAG_LOOPBACK_MODE;
  int ret_val;

  can_msg tx_msg = {
    .id = 0xDEADBEEFUL,
    .len = 8,
    .data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}
  };
  can_msg rx_msg = {0};

  // Check that handle is non-null
  CHECK_TEXT(handle != 0, "Invalid can bus handle");

  // Enable loop back mode
  ret_val = ioctl(pBus->handle, CAN_SET_FLAGS, &loopback_mode);
  CHECK_TEXT(ret_val == 0, "Invalid return value ioctl command for CAN loopback mode");

  ret_val = write(pBus->handle, &tx_msg, sizeof(tx_msg));
  CHECK_TEXT(ret_val == 0, "Invalid return value write");

  ret_val = read(pBus->handle, &rx_msg, sizeof(rx_msg));
  CHECK_TEXT(ret_val == 0, "Invalid return value read");
  CHECK_TEXT((memcmp((void*)&tx_msg, (void*)&rx_msg, sizeof(tx_msg)) == 0), "Did not receive test CAN message");

  // Disable loop back mode
  loopback_mode = 0;
  ret_val = ioctl(pBus->handle, CAN_SET_FLAGS, &loopback_mode);
  CHECK_TEXT(ret_val == 0, "Invalid return value ioctl command for disabling CAN loopback mode");
}


TEST(hal_can_fit, can_fit_loopback){
  int i;

  for(i  = 0; i < num_can_buses; i++) {
    can_fit_loopback_test((can_config*) &(can_buses[i]));
  }
}

#endif

void can_fit_can_blaster(can_config* pBus, uint32_t baudrate) {

  int ret_val;
  uint64_t tx_count;

  can_msg tx_msg = {
    .id = 0xDEADBEEFUL,
    .len = 8,
    .data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}
  };

  // Check that handle is non-null
  CHECK_TEXT(pBus->handle != 0, "Invalid can bus handle");

  //TODO: Set baud rate
  ret_val = ioctl(pBus->handle, CAN_SET_BAUDRATE, &baudrate);
  //CHECK_TEXT(ret_val == 0, "Invalid return value ioctl command for CAN_SET_BAUDRATE");

  tx_count = stm32_can_get_tx_count(pBus->instance);

  ret_val = write(pBus->handle, &tx_msg, sizeof(tx_msg));
  CHECK_TEXT(ret_val == sizeof(tx_msg), "Invalid return value write");
  CHECK_TEXT(stm32_can_get_tx_count(pBus->instance) == (tx_count + 1), "TX interrupt not received as expected");

}

// This  test requires and externally connected CAN bus to
TEST(hal_can_fit, can_fit_blaster){
  int i;

  for(i  = 0; i < num_can_buses; i++) {
    can_fit_can_blaster((can_config*) &(can_buses[i]), 2500000);
  }
}


void can_fit_ioctl(can_config* pBus) {

  int ret_val;
  uint32_t baudrate;
  uint32_t loopback_mode;
  uint32_t num_filter;

  can_filter  test_filter;

  // Check that handle is non-null
  CHECK_TEXT(pBus->handle != 0, "Invalid can bus handle");

  ret_val = ioctl(pBus->handle, CAN_SET_BAUDRATE, &baudrate);
  CHECK_TEXT(ret_val == 0, "Invalid return value ioctl command for CAN_SET_BAUDRATE");

  loopback_mode = CAN_FLAG_LOOPBACK_MODE;
  ret_val = ioctl(pBus->handle, CAN_SET_FLAGS, &loopback_mode);
  CHECK_TEXT(ret_val == 0, "Invalid return value ioctl command for CAN_SET_FLAGS");

  loopback_mode = 0;
  ret_val = ioctl(pBus->handle, CAN_SET_FLAGS, &loopback_mode);
  CHECK_TEXT(ret_val == 0, "Invalid return value ioctl command for CAN_SET_FLAGS");

  num_filter = ioctl(pBus->handle, CAN_GET_NUM_FILTERS, NULL);
  CHECK_TEXT(num_filter == stm32_can_get_num_filters(NULL), "Invalid return value ioctl command for CAN_GET_NUM_FILTERS");

  ret_val = ioctl(pBus->handle, CAN_SET_FILTER, &test_filter);
  CHECK_TEXT(ret_val == 0, "Invalid return value ioctl command for CAN_SET_FILTER");

  // test that bogus IOW will return error
  ret_val = ioctl(pBus->handle, 0xFFFFUL, NULL);
  CHECK_TEXT(ret_val != 0, "Invalid return value ioctl invocation with invalid value");
}

TEST(hal_can_fit, can_fit_ioctl){
  int i;

  for(i  = 0; i < num_can_buses; i++) {
    can_fit_ioctl((can_config*) &(can_buses[i]));
  }
}



