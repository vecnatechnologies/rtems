/**
 * @file hal-bsp-starthook.c
 * @author Jay M. Doyle
 *
 * @ingroup startup
 *
 * @brief A set of utility functions called during BSP startup.
 *
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */

#include <bsp.h>
#include <bsp/start.h>
#include <rtems.h>
#include <bspopts.h>
#include <hal-utils.h>
#include <stm32f-processor-specific.h>
#include <hal-startup-interface.h>

#include stm_processor_header( TARGET_STM_PROCESSOR_PREFIX )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, rcc )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, conf )

#define HZ_TO_MHZ( x ) ( x / 1000000 )
#define USB_OTG_CLK 48000000

// The STM32F HAL code requires this global variable
uint32_t SystemCoreClock = HSI_FREQUENCY;

/**
 * @brief  CPU L1-Cache enable.
 * @param  None
 * @retval None
 */
static void CPU_CACHE_Enable( void )
{
#if defined( ENABLE_PROCESSOR_CACHES )
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
#endif
}

/**
 * @brief Sets up clocks configuration.
 *
 * Set up clocks configuration to achieve desired system clock
 * as close as possible with simple math.
 *
 * Limitations:
 * It is assumed that 1MHz resolution is enough.
 * Best fits for the clocks are achieved with multiplies of 42MHz.
 * Even though APB1, APB2 and AHB are calculated user is still required
 * to provide correct values for the bsp configuration for the:
 * STM32F4_PCLK1
 * STM32F4_PCLK2
 * STM32F4_HCLK
 * as those are used for the peripheral clocking calculations.
 *
 * @param sys_clk Desired system clock in MHz.
 * @param hse_clk External clock speed in MHz.
 * @param hse_flag Flag determining which clock source to use, 1 for HSE,
 *                 0 for HSI.
 *
 * @retval RTEMS_SUCCESSFUL Configuration has been successfully applied for the
 *                          requested clock speed.
 * @retval RTEMS_TIMEOUT HSE clock didn't start or PLL didn't lock.
 * @retval RTEMS_INVALID_NUMBER Requested clock speed is out of range.
 */
static rtems_status_code set_system_clk(
  uint32_t sys_clk,
  uint32_t hse_clk,
  uint32_t hse_flag
)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef  ret = HAL_OK;

  int src_clk = 0;

  uint32_t pll_m = 0;
  uint32_t pll_n = 0;
  uint32_t pll_p = 0;
  uint32_t pll_q = 0;

  uint32_t ahbpre = 0;
  uint32_t apbpre1 = 0;
  uint32_t apbpre2 = 0;

  /*
   * Lets use 1MHz input for PLL so we get higher VCO output
   * this way we get better value for the PLL_Q divider for the USB
   *
   * Though you might want to use 2MHz as per CPU specification:
   *
   * Caution:The software has to set these bits correctly to ensure
   * that the VCO input frequency ranges from 1 to 2 MHz.
   * It is recommended to select a frequency of 2 MHz to limit PLL jitter.
   */

  if ( sys_clk > HZ_TO_MHZ( MAX_SYSCLK ) ) {
    return RTEMS_INVALID_NUMBER;
  } else if ( sys_clk >= 96 ) {
    pll_n = sys_clk << 1;
    pll_p = RCC_PLLP_DIV2;
  } else if ( sys_clk >= 48 ) {
    pll_n = sys_clk << 2;
    pll_p = RCC_PLLP_DIV4;
  } else if ( sys_clk >= 24 ) {
    pll_n = sys_clk << 3;
    pll_p = RCC_PLLP_DIV8;
  } else {
    return RTEMS_INVALID_NUMBER;
  }

  if ( hse_clk == 0 || hse_flag == 0 ) {
    src_clk = HZ_TO_MHZ( HSI_FREQUENCY );
    hse_flag = 0;
  } else {
    src_clk = hse_clk;
  }

  pll_m = src_clk; // divide by the oscillator speed in MHz

  /* pll_q is a prescaler from VCO for the USB OTG FS, SDIO and RNG,
   * best if results in the 48MHz for the USB
   */
  pll_q = ( (long) ( src_clk * pll_n ) ) / pll_m / HZ_TO_MHZ( USB_OTG_CLK );

  if ( pll_q < 2 ) {
    pll_q = 2;
  }

  // Apply clock configuration
  RCC_OscInitStruct.OscillatorType = (
    ( hse_flag != 0 ) ? RCC_OSCILLATORTYPE_HSE : RCC_OSCILLATORTYPE_HSI );
  RCC_OscInitStruct.HSEState =
    ( ( hse_flag != 0 ) ? RCC_HSE_ON : RCC_HSE_OFF );
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = (
    ( hse_flag != 0 ) ? RCC_PLLSOURCE_HSE : RCC_PLLSOURCE_HSI );
  RCC_OscInitStruct.PLL.PLLM = pll_m;
  RCC_OscInitStruct.PLL.PLLN = pll_n;
  RCC_OscInitStruct.PLL.PLLP = pll_p;
  RCC_OscInitStruct.PLL.PLLQ = pll_q;

  ret = HAL_RCC_OscConfig( &RCC_OscInitStruct );

  if ( ret != HAL_OK ) {
      //FIXME:: Handle this error
  }

  SystemCoreClock = src_clk;

  // Activate the OverDrive in case it is necessary to achieve desired frequency
#if defined( ENABLE_PROCESSOR_OVERDRIVE )
  ret = HAL_PWREx_EnableOverDrive();

  if ( ret != HAL_OK ) {
      //FIXME:: Handle this error
  }

#endif

  // APB1 prescaler, APB1 clock must be < 45MHz
  apbpre1 = ( sys_clk * 100 ) / HZ_TO_MHZ( APB1_CLK );

  if ( apbpre1 <= 100 ) {
    apbpre1 = RCC_HCLK_DIV1;
  } else if ( apbpre1 <= 200 ) {
    apbpre1 = RCC_HCLK_DIV2;
  } else if ( apbpre1 <= 400 ) {
    apbpre1 = RCC_HCLK_DIV4;
  } else if ( apbpre1 <= 800 ) {
    apbpre1 = RCC_HCLK_DIV8;
  } else if ( apbpre1 ) {
    apbpre1 = RCC_HCLK_DIV16;
  }

  // APB2 prescaler, APB2 clock must be < 90MHz
  apbpre2 = ( sys_clk * 100 ) / HZ_TO_MHZ( APB2_CLK );

  if ( apbpre2 <= 100 ) {
    apbpre2 = RCC_HCLK_DIV1;
  } else if ( apbpre2 <= 200 ) {
    apbpre2 = RCC_HCLK_DIV2;
  } else if ( apbpre2 <= 400 ) {
    apbpre2 = RCC_HCLK_DIV4;
  } else if ( apbpre2 <= 800 ) {
    apbpre2 = RCC_HCLK_DIV8;
  } else {
    apbpre2 = RCC_HCLK_DIV16;
  }

  // Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers
  RCC_ClkInitStruct.ClockType = ( RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 );
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = ahbpre;
  RCC_ClkInitStruct.APB1CLKDivider = apbpre1;
  RCC_ClkInitStruct.APB2CLKDivider = apbpre2;

  ret = HAL_RCC_ClockConfig( &RCC_ClkInitStruct, STM32F_FLASH_LATENCY );

  if ( ret != HAL_OK ) {
      //FIXME:: Handle this error
  }

  return RTEMS_SUCCESSFUL;
}

static void configure_external_memories( void )
{
#ifdef EXTERNAL_SDRAM
  MPU_Config();
  BSP_SDRAM_Config();
#endif
}


void BSP_START_TEXT_SECTION bsp_start_hook_0( void )
{
  // enable the CPU Cache (if available)
  CPU_CACHE_Enable();

  // basic initialization code
  HAL_Init();

  // configure all system clocks
  (void) set_system_clk( HZ_TO_MHZ( SYSCLK_FREQUENCY ),
    HZ_TO_MHZ( HSE_VALUE ),
    HSE_AVAILABLE );

  configure_external_memories();
}

void BSP_START_TEXT_SECTION bsp_start_hook_1( void )
{
  bsp_start_copy_sections();
  bsp_start_clear_bss();

  /* At this point we can use objects outside the .start section */
}
