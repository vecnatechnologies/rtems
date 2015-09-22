/**
 * @file rtc-config.c
 * @author Jay M. Doyle
 *
 * @ingroup rtc
 *
 * @brief A RTC driver implementation for all STM32F Cortex-M microcontrollers
 *   based upon ST's hardware abstraction layer API.
 *
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */

#include <rtems.h>
#include <rtems/system.h>
#include <bsp.h>
#include <libchip/rtc.h>
#include <hal-utils.h>
#include stm_processor_header( TARGET_STM_PROCESSOR_PREFIX )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, rtc )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, rcc )

#define STM32FXXXX_RTC_NUMBER 1
#define BASE_RTC_YEAR 2000

size_t                    RTC_Count = STM32FXXXX_RTC_NUMBER;
rtems_device_minor_number RTC_Minor = 0;

// Use Asynch prescaler value of 128, and synch division factor or 256
// to convert an internal (LSI) or external (LSE) 32.768 kHz into a
// 1 Hz RTC clock source.
static RTC_HandleTypeDef stm32fxxxx_rtc = {
  .Instance = RTC,
  .Init = {
    .HourFormat = RTC_HOURFORMAT_24,
    .AsynchPrediv = 128,
    .SynchPrediv = 256
  },
  .Lock = HAL_UNLOCKED,
  .State = HAL_RTC_STATE_RESET
};

static bool stm32fxxxx_rtc_device_probe( int minor )
{
  (void) minor;

  // All the processors in this family have internal RTCs
  return true;
}

static void stm32fxxxx_rtc_initialize( int minor )
{
  HAL_StatusTypeDef ret;
  RTC_TimeTypeDef   HALTime;
  RTC_DateTypeDef   HALDate;

  // To avoid compiler warnings about unused parameters
  (void) minor;

  HAL_RTC_Init( &stm32fxxxx_rtc );
}

static int stm32fxxxx_rtc_get_time(
  int                minor,
  rtems_time_of_day *time
)
{
  HAL_StatusTypeDef ret;
  RTC_TimeTypeDef   HALTime;
  RTC_DateTypeDef   HALDate;

  ret = HAL_RTC_GetTime( &stm32fxxxx_rtc, &HALTime, RTC_FORMAT_BIN );

  if ( ret == HAL_OK ) {
    time->ticks = 0;
    time->second = HALTime.Seconds;
    time->minute = HALTime.Minutes;
    time->hour = HALTime.Hours;
  } else {
    return -1;
  }

  ret = HAL_RTC_GetDate( &stm32fxxxx_rtc, &HALDate, RTC_FORMAT_BIN );

  if ( ret == HAL_OK ) {
    time->day = HALDate.Date;
    time->month = HALDate.Month;
    time->year = HALDate.Year + BASE_RTC_YEAR;
  } else {
    return -1;
  }

  return 0;
}

static int stm32fxxxx_rtc_set_time(
  int                      minor,
  const rtems_time_of_day *time
)
{
  HAL_StatusTypeDef ret;
  RTC_TimeTypeDef   HALTime;
  RTC_DateTypeDef   HALDate;

  HALTime.Seconds = time->second;
  HALTime.Minutes = time->minute;
  HALTime.Hours = time->hour;
  HALTime.SubSeconds = 0;
  HALTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  HALTime.StoreOperation = RTC_STOREOPERATION_SET;

  ret = HAL_RTC_SetTime( &stm32fxxxx_rtc, &HALTime, RTC_FORMAT_BIN );

  if ( ret != HAL_OK ) {
    return -1;
  }

  HALDate.Date = time->day;
  HALDate.Month = time->month;
  HALDate.Year = time->year - BASE_RTC_YEAR;

  ret = HAL_RTC_SetDate( &stm32fxxxx_rtc, &HALDate, RTC_FORMAT_BIN );

  if ( ret != HAL_OK ) {
    return -1;
  } else {
    return 0;
  }
}

/**
 * @brief  Initializes the RTC MSP.
 * @param  hrtc: pointer to a RTC_HandleTypeDef structure that contains
 *                the configuration information for RTC.
 * @retval None
 */
void HAL_RTC_MspInit( RTC_HandleTypeDef *hrtc )
{
  RCC_OscInitTypeDef       RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  //##-1- Configure LSI (low speed internal) as RTC clock source ############################
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI |
                                     RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;

  if ( HAL_RCC_OscConfig( &RCC_OscInitStruct ) != HAL_OK ) {
    stm32f_error_handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;

  if ( HAL_RCCEx_PeriphCLKConfig( &PeriphClkInitStruct ) != HAL_OK ) {
    stm32f_error_handler();
  }

  //##-2- Enable RTC peripheral Clocks #######################################
  // Enable RTC Clock
  __HAL_RCC_RTC_ENABLE();
}

/**
 * @brief  DeInitializes the RTC MSP.
 * @param  hrtc: pointer to a RTC_HandleTypeDef structure that contains
 *                the configuration information for RTC.
 * @retval None
 */
void HAL_RTC_MspDeInit( RTC_HandleTypeDef *hrtc )
{
  //##-1- Reset peripherals ##################################################
  __HAL_RCC_RTC_DISABLE();
}

static const rtc_fns stm32fxxxx_rtc_fns = {
  .deviceInitialize = stm32fxxxx_rtc_initialize,
  .deviceGetTime = stm32fxxxx_rtc_get_time,
  .deviceSetTime = stm32fxxxx_rtc_set_time
};

rtc_tbl RTC_Table[ STM32FXXXX_RTC_NUMBER ] = {
  {
    .sDeviceName = "/dev/rtc0",
    .deviceType = RTC_CUSTOM,
    .pDeviceFns = &stm32fxxxx_rtc_fns,
    .deviceProbe = stm32fxxxx_rtc_device_probe,
    .pDeviceParams = NULL,
    .ulCtrlPort1 = NULL,
    .ulDataPort = NULL,
    .getRegister = NULL,
    .setRegister = NULL
  }
};
