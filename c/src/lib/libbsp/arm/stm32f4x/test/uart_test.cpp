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

#include <CppUTest/CommandLineTestRunner.h>
#include <CppUTest/TestHarness.h>

#define MAX_STR 255
#define COUNTOF(x) (sizeof(x)/sizeof(x[0]))
wchar_t strVecnaManufacturer[MAX_STR] = L"Vecna Technologies Inc.";
wchar_t strSCProduct[MAX_STR] = L"SC";

#define MAX_TRANSITION_TO_ACTIVE    1000
#define MAX_TRANSITION_TO_NO_POWER  20
#define MAX_TRANSITION_TO_DISABLED  20
#define POLL_DEVICE_STATE_PERIOD_ms 5
#define TARGET_MIN_TRANS_VELOCITY   0.5
#define MAX_WAIT_FOR_ACTIVE         100

static bool m_Verbose = false;


TEST_GROUP(Uart_Base) {

  void setup() {

    //printf("Stop Codes = %.016llX\n", DeviceState.u64_StopCodes);
    //LONGS_EQUAL(DRIVE_STATE_ACTIVE, DeviceState.enm_DriveState);
  }

  void teardown() {

  }
};



TEST(Uart_Base, dummy_test) {

    //printf("Stop Codes = %.016llX\n", DeviceState.u64_StopCodes);
    //LONGS_EQUAL(DRIVE_STATE_ACTIVE, DeviceState.enm_DriveState);
}



int main(int argc, char * argv[]) {
  return CommandLineTestRunner::RunAllTests(argc, argv);
}


