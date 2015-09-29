/**
 * @file cmsis_unit.c
 * @author Jay M. Doyle
 *
 * @ingroup test
 *
 * @brief CppUnit test code for testing the functions implemented in
 *   cmsis-os.c
 *
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */

#include <CppUTest/TestHarness.h>
#include <stdio.h>

extern "C"
{
#include <cmsis_os.h>
}

TEST_GROUP(cmsis_os_unit) {

  void setup() {

  }

  void teardown() {

  }
};

TEST(cmsis_os_unit, osSemaphoreRelease) {

  osSemaphoreId s_xSemaphore = NULL;
  int ret;

  osSemaphoreDef(SEM);
  s_xSemaphore = osSemaphoreCreate(osSemaphore(SEM) , 0 );
  CHECK_TEXT(s_xSemaphore != 0, "osSemaphore returned an invalid value (0)");

  // This wait should timeout since semaphore was created with an initial value of 0
  ret = osSemaphoreWait(s_xSemaphore, 100);
  CHECK_TEXT(ret != osOK, "osSemaphoreWait did not wait as expected");

  // Release semaphore
  ret = osSemaphoreRelease(s_xSemaphore);
  CHECK_TEXT(ret == osOK, "osSemaphoreRelease did not return osOK as expected");

  // Now check that semaphore is avaiable as expected
  ret = osSemaphoreWait(s_xSemaphore, 100);
  CHECK_TEXT(ret == osOK, "osSemaphoreWait did return immediately as expected");

}

TEST(cmsis_os_unit, osSemaphoreWait) {

  osSemaphoreId s_xSemaphore = NULL;

  osSemaphoreDef(SEM);
  s_xSemaphore = osSemaphoreCreate(osSemaphore(SEM) , 0 );
  CHECK_TEXT(s_xSemaphore != 0, "osSemaphore returned an invalid value (0)");

  // This wait should timeout since semaphore was created with an initial value of 0
  int ret = osSemaphoreWait(s_xSemaphore, 100);
  CHECK_TEXT(ret != osOK, "osSemaphoreWait did not wait as expected");
}

TEST(cmsis_os_unit, osSemaphoreCreate) {

  osSemaphoreId s_xSemaphore = NULL;

  osSemaphoreDef(SEM);
  s_xSemaphore = osSemaphoreCreate(osSemaphore(SEM) , 0 );
  CHECK_TEXT(s_xSemaphore != 0, "osSemaphore returned an invalid value (0)");
}

TEST(cmsis_os_unit, sanity_test) {

  rtems_id test_semaphore;
  rtems_status_code ret;

  ret = rtems_semaphore_create(
    rtems_build_name('C', 'S', 'M', '1'),
    1,
    RTEMS_SIMPLE_BINARY_SEMAPHORE,
    RTEMS_NO_PRIORITY,
    &test_semaphore
  );

  CHECK_TEXT(ret == RTEMS_SUCCESSFUL, "rtems_semaphore_create did not return RTEMS_SUCCESS");
  CHECK_TEXT(test_semaphore != 0, "rtems_semaphore_create did not return valid value");

  ret = rtems_semaphore_obtain(test_semaphore, RTEMS_WAIT, 1000);
  CHECK_TEXT(ret == RTEMS_SUCCESSFUL, "rtems_semaphore_obtain did not work as expected");

  ret = rtems_semaphore_obtain(test_semaphore, RTEMS_WAIT, 100);
  CHECK_TEXT(ret != RTEMS_SUCCESSFUL, "rtems_semaphore_obtain did not work as expected");

}


