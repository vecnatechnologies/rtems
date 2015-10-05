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
 * sdram_unit.cpp
 *
 *  @author jay.doyle
 *
 *  Change Log:
 *    Created Apr 30, 2015 - jay.doyle
 */

#include <CppUTest/TestHarness.h>

extern "C" {
#include <stdio.h>
#include <hal-utils.h>
#include <hal-sdram-interface.h>
#include <bspopts.h>
}

#include stm_processor_header( TARGET_STM_PROCESSOR_PREFIX )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, cortex )

TEST_GROUP(hal_sdram_unit) {

  void setup() {

  }

  void teardown() {

  }
};


TEST(hal_sdram_unit, MPU_Get_Region_Size) {

  // Check for error value if input value is greater than maximum value
  CHECK_TEXT(MPU_Get_Region_Size(0x200000000) == 0, "Invalid return value for 4GB");

  // Check maximum value of MPU region
  CHECK_TEXT(MPU_Get_Region_Size(0x100000000) == MPU_REGION_SIZE_4GB, "Invalid return value for 4GB");

  // Check region size in the middle of the acceptable range
  CHECK_TEXT(MPU_Get_Region_Size(0x100000) == MPU_REGION_SIZE_1MB, "Invalid return value for 1MB");

  // Check to see if the smallest possible size is handled correctly
  CHECK_TEXT(MPU_Get_Region_Size(32) == MPU_REGION_SIZE_32B, "Invalid return value for 32B");

  // Check to see if the function returns 0 for sizes that are too small
  CHECK_TEXT(MPU_Get_Region_Size(16) == 0, "Memory region of 16 bytes should return 0 (invalid value)");

  // Check to see if the function return 0 for non power of two size
  CHECK_TEXT(MPU_Get_Region_Size(1237) == 0, "Function should return 0 for non-power of 2 region sizes");
}





