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
 * Test_SCB.cpp
 *
 *  @author jay.doyle
 *
 *  Change Log:
 *    Created Apr 30, 2015 - jay.doyle
 */

#include <CppUTest/TestHarness.h>
#include <stdio.h>
#include <hal-utils.h>
#include <hal-sdram-interface.h>
#include <bspopts.h>

#define TEST_PATTERN 0xA5A5A5A5UL

TEST_GROUP(hal_sdram_fit) {

  void setup() {

  }

  void teardown() {

  }
};


TEST(hal_sdram_fit, stm32f_sdram_operation) {

  // Use the last uint32_t location SDRAM as a test location
  //volatile uint32_t* test_location = (uint32_t*) (STM32F7_START_SDRAM + STM32F7_SDRAM_SIZE - sizeof(uint32_t));
  volatile uint32_t* test_location = (uint32_t*) (STM32_START_SDRAM + STM32_SDRAM_SIZE - sizeof(uint32_t));

  uint32_t initial_value = *test_location;
  uint32_t readback_value;

  CHECK_TEXT(test_location != NULL, "Target SDRAM address is NULL");

  *test_location = 0UL;
  readback_value = *test_location;

  CHECK_TEXT(readback_value == 0, "stm32f_sdram_operational: Failed to initialize test location");

  *test_location = TEST_PATTERN;
  readback_value = *test_location;

  CHECK_TEXT(readback_value == TEST_PATTERN, "stm32f_sdram_operational: Failed to initialize test location with test value");

  *test_location = ~(TEST_PATTERN);
  readback_value = *test_location;

  CHECK_TEXT(readback_value == ~(TEST_PATTERN), "stm32f_sdram_operational: Failed to initialize test location with test value");

  *test_location = initial_value;
}




