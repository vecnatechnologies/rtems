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

extern "C" {
#include <hal-utils.h>
#include <hal-can.h>
#include <bspopts.h>
#include <dev/can/can.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>
#include <hal-can.h>
}

#define MAX_NUMBER_OF_CAN_BUSES 3
#define NUM_TEST_TX_MESSAGES 10
#define MIN_BAUD_RATE       10000   // 10  kb/s @ 1 km range
#define TYPICAL_BAUD_RATE  500000   // 500 kb/s
#define MAX_BAUD_RATE     1000000   // 1   Mb/s @ 40 m range

struct can_config {
    char name[16];
    int handle;
    CAN_Instance instance;
};

static can_msg tx_msg = {
  .id = 0xDEADBEEFUL,
  .len = 8,
  .data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}
};

static can_msg rx_msg = {0};

static can_config can_buses[NUM_CAN_INSTANCES] = {0};
static int num_can_buses = 0;

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
 * This function requires that the can bus be connected to
 * another CAN bus device capable of acknowledging the transmission.
 * Both can buses must be configured for the same baud rate.
 */
void can_fit_write_one_message(can_config* pBus,
                              int32_t baudrate,
                              can_msg* pMsg) {

  int ret_val;
  uint64_t start_tx_count;
  uint64_t post_tx_count;

  // Check that handle is non-null
  CHECK_TEXT(pBus->handle != 0, "Invalid can bus handle");

  // Configure baudrate
  ret_val = ioctl(pBus->handle, CAN_SET_BAUDRATE, baudrate);
  CHECK_TEXT(ret_val == 0, "Invalid return value ioctl command for CAN_SET_BAUDRATE");

  // Keep track of how many messages have been sent so far
  start_tx_count = stm32_can_get_tx_count(pBus->instance);
  ret_val = write(pBus->handle, pMsg, sizeof(*pMsg));
  post_tx_count = stm32_can_get_tx_count(pBus->instance);

  // Make sure that the number of bytes written == the size of the message
  CHECK_TEXT(ret_val == sizeof(*pMsg), "Invalid return value write");

  // Make sure that the packet was actually sent by the hardware.  In order to
  // have been sent the hardware must sent the data, another device must acknowledge
  // the transmit, and the interrupt handler must fire and eventually call the
  // call back routine to indicated that the message has been completely sent.
  CHECK_TEXT(post_tx_count == (start_tx_count + 1), "TX interrupt not received as expected");
}

void can_fit_loopback_test(can_config* pBus,
                           int32_t baudrate) {

  uint32_t loopback_mode = CAN_FLAG_LOOPBACK_MODE;
  int ret_val;

  loopback_mode = CAN_FLAG_LOOPBACK_MODE;
  ret_val = ioctl(pBus->handle, CAN_SET_FLAGS, loopback_mode);
  CHECK_TEXT(ret_val == 0, "Invalid return value ioctl command for CAN_SET_FLAGS");

  // zero out rx message
  memset((void*)&rx_msg, 0, sizeof(rx_msg));

  // write test TX message
  can_fit_write_one_message(pBus, baudrate, &tx_msg);

  ret_val = read(pBus->handle, &rx_msg, sizeof(rx_msg));
  CHECK_TEXT(ret_val == 0, "Invalid return value read");
  CHECK_TEXT((memcmp((void*)&tx_msg, (void*)&rx_msg, sizeof(tx_msg)) == 0), "Did not receive test CAN message");

  // Disable loop back mode
  loopback_mode = 0;
  ret_val = ioctl(pBus->handle, CAN_SET_FLAGS, loopback_mode);
  CHECK_TEXT(ret_val == 0, "Invalid return value ioctl command for CAN_SET_FLAGS");
}


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


TEST(hal_can_fit, can_fit_loopback){
  int i;

  for(i  = 0; i < num_can_buses; i++) {
    can_fit_loopback_test((can_config*) &(can_buses[i]), MAX_BAUD_RATE);
  }
}

// This  test requires and externally connected CAN bus to
TEST(hal_can_fit, can_fit_write){
  int i;

  for(i  = 0; i < num_can_buses; i++) {
    can_fit_write_one_message((can_config*) &(can_buses[i]), MAX_BAUD_RATE, &tx_msg);
  }
}


void can_fit_ioctl(can_config* pBus) {

  int ret_val;
  int32_t  baudrate;
  uint32_t loopback_mode;
  int      num_filter;

  can_filter  test_filter;

  // Check that handle is non-null
  CHECK_TEXT(pBus->handle != 0, "Invalid can bus handle");

  //------------------------------------------------------
  // Baud rates can never be set outside the range (0, 1000000]
  // and the clock configuration reduces the set of possible
  // range even more.
  //------------------------------------------------------

  // Check that error code is received if baudrate is 0
  baudrate = 0;
  ret_val = ioctl(pBus->handle, CAN_SET_BAUDRATE, baudrate);
  CHECK_TEXT(ret_val == -1, "Invalid return value ioctl command for CAN_SET_BAUDRATE (baudrate 0)");

  baudrate = MIN_BAUD_RATE;
  ret_val = ioctl(pBus->handle, CAN_SET_BAUDRATE, baudrate);
  CHECK_TEXT(ret_val == 0, "Invalid return value ioctl command for CAN_SET_BAUDRATE (baudrate MIN_BAUD_RATE)");

  // Check normal value
  baudrate = TYPICAL_BAUD_RATE;
  ret_val = ioctl(pBus->handle, CAN_SET_BAUDRATE, baudrate);
  CHECK_TEXT(ret_val == 0, "Invalid return value ioctl command for CAN_SET_BAUDRATE (baudrate TYPICAL_BAUD_RATE)");

  // Check maximum value
  baudrate = MAX_BAUD_RATE;
  ret_val = ioctl(pBus->handle, CAN_SET_BAUDRATE, baudrate);
  CHECK_TEXT(ret_val == 0, "Invalid return value ioctl command for CAN_SET_BAUDRATE (baudrate MAX_BAUD_RATE)");

  // Check for first value above maximum value
  baudrate = MAX_BAUD_RATE+1;
  ret_val = ioctl(pBus->handle, CAN_SET_BAUDRATE, baudrate);
  CHECK_TEXT(ret_val == -1, "Invalid return value ioctl command for CAN_SET_BAUDRATE (baudrate MAX_BAUD_RATE+1)");

  //----------------------------------------------------------------
  // CAN_SET_FLAGS supports:
  //   -- CAN_FLAG_LOOPBACK_MODE (internal loopback mode)
  //----------------------------------------------------------------

  loopback_mode = CAN_FLAG_LOOPBACK_MODE;
  ret_val = ioctl(pBus->handle, CAN_SET_FLAGS, loopback_mode);
  CHECK_TEXT(ret_val == 0, "Invalid return value ioctl command for CAN_SET_FLAGS");

  loopback_mode = 0;
  ret_val = ioctl(pBus->handle, CAN_SET_FLAGS, loopback_mode);
  CHECK_TEXT(ret_val == 0, "Invalid return value ioctl command for CAN_SET_FLAGS");

  num_filter = ioctl(pBus->handle, CAN_GET_NUM_FILTERS, NULL);
  CHECK_TEXT(num_filter == stm32_can_get_num_filters(NULL), "Invalid return value ioctl command for CAN_GET_NUM_FILTERS");

  //----------------------------------------------------------------
  // CAN_SET_FILTER
  //----------------------------------------------------------------
  test_filter.number = 0;
  test_filter.mask   = 0xFFFF;
  test_filter.filter = 0x0003;

  ret_val = ioctl(pBus->handle, CAN_SET_FILTER, &test_filter);
  CHECK_TEXT(ret_val == 0, "Invalid return value ioctl command for CAN_SET_FILTER");

  // Test error condition by trying to use a filter number that is not supported
  test_filter.number = num_filter;
  ret_val = ioctl(pBus->handle, CAN_SET_FILTER, &test_filter);
  CHECK_TEXT(ret_val != 0, "Invalid return value ioctl command for CAN_SET_FILTER for invalid filter number");

  //----------------------------------------------------------------
  // Test for bogus IOW value
  //----------------------------------------------------------------

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



