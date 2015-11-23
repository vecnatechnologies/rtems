/**
 * @file sdram.c
 * @author Jay M. Doyle
 *
 * @ingroup sdram
 *
 * @brief A set of utility functions used to configure external SDRAM
 *
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */

#include <hal-utils.h>
#include <stm32f-processor-specific.h>
#include <hal-sdram-interface.h>
#include <hal-error.h>
#include <bspopts.h>

#include stm_processor_header( TARGET_STM_PROCESSOR_PREFIX )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, dma )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, sdram )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, cortex )
#include stm_ll_header( TARGET_STM_PROCESSOR_PREFIX, fmc )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, gpio )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, rcc_ex )

#if ( STM32_SDRAM_SIZE > 0 )

#define MINIMUM_MPU_REGION_SIZE 32
#define MAXIMUM_MPU_REGION_SIZE (0x100000000) // 4GB

// The following static data must be placed in a section that resides
// in internal SRAM since they are used during SDRAM initialization.
static SDRAM_HandleTypeDef      hsdram       __attribute__((section(".bsp_fast_data")));
static FMC_SDRAM_TimingTypeDef  SDRAM_Timing __attribute__((section(".bsp_fast_data")));
static FMC_SDRAM_CommandTypeDef command      __attribute__((section(".bsp_fast_data")));

/**
 *  @brief Calculate the MPU region size setting which is
 *    is log2(x) - 1.
 *
 *    This function assumes that the memory size is perfect power
 *    of 2.
 *
 *  @param region_size_in_bytes The sizeof the memory region in bytes
 *  @return The correct MPU setting for the given memory size
 */
uint8_t MPU_Get_Region_Size( const uint64_t region_size_in_bytes )
{
  uint8_t mpu_setting = (uint8_t) 0;
  uint8_t i;

  // The size of the region must be 32 or larger.
  if (( region_size_in_bytes >= MINIMUM_MPU_REGION_SIZE ) && (region_size_in_bytes <= MAXIMUM_MPU_REGION_SIZE)){
    for ( i = 0; i <= 32; i++ ) {
      if ( ( ( region_size_in_bytes >> i ) & 0x1UL ) != 0 ) {
        if ( mpu_setting == 0 ) {
          mpu_setting = i - 1;
        } else {
          // Invalid input number
          mpu_setting = 0;
          break;
        }
      }
    }
  }

  return mpu_setting;
}

/**
 * @brief  Configure the MPU attributes as Write Through for SRAM1/2.
 * @note   The Base Address is 0x20010000 since this memory interface is the AXI.
 *         The Region Size is 256KB, it is related to SRAM1 and SRAM2  memory size.
 * @param  None
 * @retval None
 */
void MPU_Config( void )
{
  MPU_Region_InitTypeDef MPU_InitStruct;

  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Configure the MPU attributes as WT for SRAM */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = STM32_START_INTERNAL_SRAM;
  MPU_InitStruct.Size = MPU_Get_Region_Size(STM32_INTERNAL_SRAM_SIZE);
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  // Check for valid memory region sizes first
  if (MPU_InitStruct.Size == 0 ) {
    stm32f_error_handler_with_reason("Invalid memory region size specified for internal SRAM");
  }

  HAL_MPU_ConfigRegion( &MPU_InitStruct );

  /* Configure the MPU attributes as WT and WA for SDRAM */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = STM32_START_SDRAM;
  MPU_InitStruct.Size = MPU_Get_Region_Size(STM32_SDRAM_SIZE);
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;

  // Check for valid memory region sizes first
  if (MPU_InitStruct.Size == 0 ) {
    stm32f_error_handler_with_reason("Invalid memory region size specified for external SDRAM");
  }

  HAL_MPU_ConfigRegion( &MPU_InitStruct );

  /* Enable the MPU */
  HAL_MPU_Enable( MPU_PRIVILEGED_DEFAULT );
}


#ifdef STM32F7_DISCOVERY

/**
 * @brief  Perform the SDRAM external memory initialization sequence
 * @param  hsdram: SDRAM handle
 * @param  Command: Pointer to SDRAM command structure
 * @retval None
 */
static void BSP_SDRAM_Initialization_Sequence(
  SDRAM_HandleTypeDef      *hsdram,
  FMC_SDRAM_CommandTypeDef *Command
)
{
  __IO uint32_t tmpmrd = 0;

  /* Step 3:  Configure a clock configuration enable command */
  Command->CommandMode = FMC_SDRAM_CMD_CLK_ENABLE;
  Command->CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
  Command->AutoRefreshNumber = 1;
  Command->ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand( hsdram, Command, SDRAM_TIMEOUT );

  /* Step 4: Insert 100 us minimum delay */
  /* Inserted delay is equal to 1 ms due to systick time base unit (ms) */
  HAL_Delay( 1 );

  /* Step 5: Configure a PALL (precharge all) command */
  Command->CommandMode = FMC_SDRAM_CMD_PALL;
  Command->CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
  Command->AutoRefreshNumber = 1;
  Command->ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand( hsdram, Command, SDRAM_TIMEOUT );

  /* Step 6 : Configure a Auto-Refresh command */
  Command->CommandMode = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
  Command->CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
  Command->AutoRefreshNumber = 8;
  Command->ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand( hsdram, Command, SDRAM_TIMEOUT );

  /* Step 7: Program the external memory mode register */
  tmpmrd = (uint32_t) SDRAM_MODEREG_BURST_LENGTH_1 |
           SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL |
           SDRAM_MODEREG_CAS_LATENCY_2 |
           SDRAM_MODEREG_OPERATING_MODE_STANDARD |
           SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;

  Command->CommandMode = FMC_SDRAM_CMD_LOAD_MODE;
  Command->CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
  Command->AutoRefreshNumber = 1;
  Command->ModeRegisterDefinition = tmpmrd;

  /* Send the command */
  HAL_SDRAM_SendCommand( hsdram, Command, SDRAM_TIMEOUT );

  /* Step 8: Set the refresh rate counter */
  /* (15.62 us x Freq) - 20 */
  /* Set the device refresh counter */
  hsdram->Instance->SDRTR |= ( (uint32_t) ( ( 1292 ) << 1 ) );
}

void BSP_SDRAM_Config( void )
{
  /*##-1- Configure the SDRAM device #########################################*/
  /* SDRAM device configuration */
  hsdram.Instance = FMC_SDRAM_DEVICE;

  SDRAM_Timing.LoadToActiveDelay = 2;
  SDRAM_Timing.ExitSelfRefreshDelay = 7;
  SDRAM_Timing.SelfRefreshTime = 4;
  SDRAM_Timing.RowCycleDelay = 7;
  SDRAM_Timing.WriteRecoveryTime = 2;
  SDRAM_Timing.RPDelay = 2;
  SDRAM_Timing.RCDDelay = 2;

  hsdram.Init.SDBank = FMC_SDRAM_BANK1;
  hsdram.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram.Init.MemoryDataWidth = SDRAM_MEMORY_WIDTH;
  hsdram.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_2;
  hsdram.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram.Init.SDClockPeriod = SDCLOCK_PERIOD;
  hsdram.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
  hsdram.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;

  /* Initialize the SDRAM controller */
  if ( HAL_SDRAM_Init( &hsdram, &SDRAM_Timing ) != HAL_OK ) {
    /* Initialization Error */
    stm32f_error_handler();
  }

  /* Program the SDRAM external device */
  BSP_SDRAM_Initialization_Sequence( &hsdram, &command );
}

/**
 * @brief SDRAM MSP Initialization
 *        This function configures the hardware resources used in this example:
 *           - Peripheral's clock enable
 *           - Peripheral's GPIO Configuration
 * @param hsram: SDRAM handle pointer
 * @retval None
 */
void HAL_SDRAM_MspInit( SDRAM_HandleTypeDef *hsdram )
{
  GPIO_InitTypeDef GPIO_Init_Structure;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO clocks */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /* Enable FMC clock */
  __HAL_RCC_FMC_CLK_ENABLE();

  /*##-2- Configure peripheral GPIO ##########################################*/
  GPIO_Init_Structure.Mode = GPIO_MODE_AF_PP;
  GPIO_Init_Structure.Pull = GPIO_PULLUP;
  GPIO_Init_Structure.Speed = GPIO_SPEED_FAST;
  GPIO_Init_Structure.Alternate = GPIO_AF12_FMC;

  /* GPIOC configuration */
  GPIO_Init_Structure.Pin = GPIO_PIN_3;
  HAL_GPIO_Init( GPIOC, &GPIO_Init_Structure );

  /* GPIOD configuration */
  GPIO_Init_Structure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_8 |
                            GPIO_PIN_9 |
                            GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init( GPIOD, &GPIO_Init_Structure );

  /* GPIOE configuration */
  GPIO_Init_Structure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_7 | GPIO_PIN_8 |
                            GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |
                            GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init( GPIOE, &GPIO_Init_Structure );

  /* GPIOF configuration */
  GPIO_Init_Structure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                            GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_11 | GPIO_PIN_12 |
                            GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init( GPIOF, &GPIO_Init_Structure );

  /* GPIOG configuration */
  GPIO_Init_Structure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 |
                            GPIO_PIN_8 | GPIO_PIN_15;
  HAL_GPIO_Init( GPIOG, &GPIO_Init_Structure );

  /* GPIOH configuration */
  GPIO_Init_Structure.Pin = GPIO_PIN_3 | GPIO_PIN_5;
  HAL_GPIO_Init( GPIOH, &GPIO_Init_Structure );
}


/**
 * @brief SDRAM MSP De-Initialization
 *        This function frees the hardware resources used in this example:
 *          - Disable the Peripheral's clock
 *          - Revert GPIO configuration to their default state
 * @param hsram: SDRAM handle pointer
 * @retval None
 */
void HAL_SDRAM_MspDeInit( SDRAM_HandleTypeDef *hsdram )
{
  /*## Disable peripherals and GPIO Clocks ###################################*/
  /* Configure FMC as alternate function  */
  HAL_GPIO_DeInit( GPIOD,
    GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 |
    GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |
    GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 );

  HAL_GPIO_DeInit( GPIOE,
    GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_7 |
    GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |
    GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 );

  HAL_GPIO_DeInit( GPIOF,
    GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |
    GPIO_PIN_5 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 );

  HAL_GPIO_DeInit( GPIOG,
    GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |
    GPIO_PIN_5 | GPIO_PIN_10 );
}

#elif STM32F7_EVAL2
void BSP_SDRAM_Config( void )
{
  register uint32_t tmpreg = 0, timeout = 0xFFFF;
  register uint32_t index;

  /* Enable GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH and GPIOI interface
      clock */
  RCC->AHB1ENR |= 0x000001F8;

  /* Connect PDx pins to FMC Alternate function */
  GPIOD->AFR[0]  = 0x000000CC;
  GPIOD->AFR[1]  = 0xCC000CCC;
  /* Configure PDx pins in Alternate function mode */
  GPIOD->MODER   = 0xA02A000A;
  /* Configure PDx pins speed to 50 MHz */
  GPIOD->OSPEEDR = 0xA02A000A;
  /* Configure PDx pins Output type to push-pull */
  GPIOD->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PDx pins */
  GPIOD->PUPDR   = 0x00000000;

  /* Connect PEx pins to FMC Alternate function */
  GPIOE->AFR[0]  = 0xC00000CC;
  GPIOE->AFR[1]  = 0xCCCCCCCC;
  /* Configure PEx pins in Alternate function mode */
  GPIOE->MODER   = 0xAAAA800A;
  /* Configure PEx pins speed to 50 MHz */
  GPIOE->OSPEEDR = 0xAAAA800A;
  /* Configure PEx pins Output type to push-pull */
  GPIOE->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PEx pins */
  GPIOE->PUPDR   = 0x00000000;

  /* Connect PFx pins to FMC Alternate function */
  GPIOF->AFR[0]  = 0xCCCCCCCC;
  GPIOF->AFR[1]  = 0xCCCCCCCC;
  /* Configure PFx pins in Alternate function mode */
  GPIOF->MODER   = 0xAA800AAA;
  /* Configure PFx pins speed to 50 MHz */
  GPIOF->OSPEEDR = 0xAA800AAA;
  /* Configure PFx pins Output type to push-pull */
  GPIOF->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PFx pins */
  GPIOF->PUPDR   = 0x00000000;

  /* Connect PGx pins to FMC Alternate function */
  GPIOG->AFR[0]  = 0xCCCCCCCC;
  GPIOG->AFR[1]  = 0xCCCCCCCC;
  /* Configure PGx pins in Alternate function mode */
  GPIOG->MODER   = 0xAAAAAAAA;
  /* Configure PGx pins speed to 50 MHz */
  GPIOG->OSPEEDR = 0xAAAAAAAA;
  /* Configure PGx pins Output type to push-pull */
  GPIOG->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PGx pins */
  GPIOG->PUPDR   = 0x00000000;

  /* Connect PHx pins to FMC Alternate function */
  GPIOH->AFR[0]  = 0x00C0CC00;
  GPIOH->AFR[1]  = 0xCCCCCCCC;
  /* Configure PHx pins in Alternate function mode */
  GPIOH->MODER   = 0xAAAA08A0;
  /* Configure PHx pins speed to 50 MHz */
  GPIOH->OSPEEDR = 0xAAAA08A0;
  /* Configure PHx pins Output type to push-pull */
  GPIOH->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PHx pins */
  GPIOH->PUPDR   = 0x00000000;

  /* Connect PIx pins to FMC Alternate function */
  GPIOI->AFR[0]  = 0xCCCCCCCC;
  GPIOI->AFR[1]  = 0x00000CC0;
  /* Configure PIx pins in Alternate function mode */
  GPIOI->MODER   = 0x0028AAAA;
  /* Configure PIx pins speed to 50 MHz */
  GPIOI->OSPEEDR = 0x0028AAAA;
  /* Configure PIx pins Output type to push-pull */
  GPIOI->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PIx pins */
  GPIOI->PUPDR   = 0x00000000;

/*-- FMC Configuration ------------------------------------------------------*/
  /* Enable the FMC interface clock */
  RCC->AHB3ENR |= 0x00000001;

  /* Configure and enable SDRAM bank1 */
  FMC_Bank5_6->SDCR[0] = 0x000019E0;
  FMC_Bank5_6->SDTR[0] = 0x01115351;

  /* SDRAM initialization sequence */
  /* Clock enable command */
  FMC_Bank5_6->SDCMR = 0x00000011;
  tmpreg = FMC_Bank5_6->SDSR & 0x00000020;
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020;
  }

  /* Delay */
  for (index = 0; index<1000; index++);

  /* PALL command */
  FMC_Bank5_6->SDCMR = 0x00000012;
  timeout = 0xFFFF;
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020;
  }

  /* Auto refresh command */
  FMC_Bank5_6->SDCMR = 0x00000073;
  timeout = 0xFFFF;
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020;
  }

  /* MRD register program */
  FMC_Bank5_6->SDCMR = 0x00046014;
  timeout = 0xFFFF;
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020;
  }

  /* Set refresh count */
  tmpreg = FMC_Bank5_6->SDRTR;
  FMC_Bank5_6->SDRTR = (tmpreg | (0x0000027C<<1));

  /* Disable write protection */
  tmpreg = FMC_Bank5_6->SDCR[0];
  FMC_Bank5_6->SDCR[0] = (tmpreg & 0xFFFFFDFF);

}
#endif




#endif
