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
 *    Created Oct. 9, 2015 - jay.doyle
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

#define MAX_PERCENT_BAUDRATE_ERROR 1.0F // 1% error

static const uint32_t test_can_baud_rates[] = {125000, 250000, 500000, 1000000};


TEST_GROUP( hal_can_unit )
{
  void setup()
  {
  }

  void teardown()
  {

  }
};


TEST( hal_can_unit, rtems_can_get_timing_values )
{
  CAN_Timing_Values result;
  char errorText[64];
  uint32_t i;

  for(i = 0; i < COUNTOF(test_can_baud_rates); i++) {
    result = rtems_can_get_timing_values ( test_can_baud_rates[i]);
    snprintf(errorText, sizeof(errorText), "Error producing BS1_MAX timing values for %lu baudrate", test_can_baud_rates[i]);
    CHECK_TEXT(result.s1 <= BS1_MAX, errorText);
    CHECK_TEXT(result.s1 >= 1, errorText);
    snprintf(errorText, sizeof(errorText), "Error producing BS2_MAX timing values for %lu baudrate", test_can_baud_rates[i]);
    CHECK_TEXT(result.s2 <= BS2_MAX, errorText);
    CHECK_TEXT(result.s2 >= 1, errorText);
    snprintf(errorText, sizeof(errorText), "Error producing timing values accurate enough for %lu baudrate", test_can_baud_rates[i]);
    CHECK_TEXT(((result.error / (float) test_can_baud_rates[i]) * 100.0) < MAX_PERCENT_BAUDRATE_ERROR, errorText);

  }
}


