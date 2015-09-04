/**
 * @file hal-i2c-init.c
 *
 * @ingroup can
 *
 * @brief I2C driver for initializing the I2C instance on the STM32F4x series processors.
 *
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */

//================== STMF32 Support Functions =================================
#include <rtems.h>
#include <hal-utils.h>
#include <stm32f-processor-specific.h>
#include <bspopts.h>

#include stm_processor_header(TARGET_STM_PROCESSOR_PREFIX)
#include stm_header(TARGET_STM_PROCESSOR_PREFIX, dma)
#include stm_header(TARGET_STM_PROCESSOR_PREFIX, i2c)
#include stm_header(TARGET_STM_PROCESSOR_PREFIX, rcc)
#include stm_header(TARGET_STM_PROCESSOR_PREFIX, gpio)

#include <hal-i2c.h>
#include <hal-i2c-init.h>

/************************ Defines ******************************/

/**
 *  Device Address for each I2C instances
 */
#define I2C1_ADDRESS        			0x30F
#define I2C2_ADDRESS        			0x31F
#define I2C3_ADDRESS       				0x32F

/**
 *  Definition for I2C1 Pins
 */
#define I2C1_SCL_PIN                    GPIO_PIN_6
#define I2C1_SCL_GPIO_PORT              GPIOB
#define I2C1_SDA_PIN                    GPIO_PIN_7
#define I2C1_SDA_GPIO_PORT              GPIOB

/**
 *  Definition for I2C2 Pins
 */
#define I2C2_SCL_PIN                    GPIO_PIN_10
#define I2C2_SCL_GPIO_PORT              GPIOB
#define I2C2_SDA_PIN                    GPIO_PIN_11
#define I2C2_SDA_GPIO_PORT              GPIOB

/**
 *  Definition for I2C3 Pins
 */
#define I2C3_SCL_PIN                    GPIO_PIN_8
#define I2C3_SCL_GPIO_PORT              GPIOA
#define I2C3_SDA_PIN                    GPIO_PIN_9
#define I2C3_SDA_GPIO_PORT              GPIOC

/* I2C Vector numbers */
const rtems_vector_number I2C_IRQ_VEC[MAX_I2C_INSTANCES] = { I2C1_EV_IRQn, \
															 I2C2_EV_IRQn, \
															 I2C3_EV_IRQn };

const bool I2C_Enable[MAX_I2C_INSTANCES] = {
#if (STM32F4_ENABLE_I2C1)
										true,
#else
										false,
#endif
#if (STM32F4_ENABLE_I2C2)
										true,
#else
										false,
#endif
#if (STM32F4_ENABLE_I2C3)
										true
#else
										false
#endif
};


/**
 * Initialize GPIO pins
 */
void stm32_i2c_gpio_init(stm32_i2c_bus * bus)
{

	GPIO_InitTypeDef  GPIO_InitStruct;

	switch(bus->instance)
	{
	case I2C_ONE: 	__HAL_RCC_I2C1_CLK_ENABLE();
					break;

	case I2C_TWO: 	__HAL_RCC_I2C2_CLK_ENABLE();
					break;

	case I2C_THREE: __HAL_RCC_I2C3_CLK_ENABLE();
					break;
	}

	/* Initialize GPIO Clock */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	  GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
	  GPIO_InitStruct.Pull      = GPIO_PULLUP;
	  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
	  GPIO_InitStruct.Alternate = I2C_AF;

	  if(bus->instance == I2C_ONE)
	  {
		  /* I2C TX GPIO pin configuration  */
		  GPIO_InitStruct.Pin       = I2C1_SCL_PIN;
		  HAL_GPIO_Init(I2C1_SCL_GPIO_PORT, &GPIO_InitStruct);

		  /* I2C RX GPIO pin configuration  */
		  GPIO_InitStruct.Pin = I2C1_SDA_PIN;
		  HAL_GPIO_Init(I2C1_SDA_GPIO_PORT, &GPIO_InitStruct);
	  }
	  else if(bus->instance == I2C_TWO)
	  {
		  /* I2C TX GPIO pin configuration  */
		  GPIO_InitStruct.Pin = I2C2_SCL_PIN;
		  HAL_GPIO_Init(I2C2_SCL_GPIO_PORT, &GPIO_InitStruct);

		  /* I2C RX GPIO pin configuration  */
		  GPIO_InitStruct.Pin = I2C2_SDA_PIN;
		  HAL_GPIO_Init(I2C2_SDA_GPIO_PORT, &GPIO_InitStruct);
	  }
	  else if(bus->instance == I2C_THREE)
	  {
		  /* I2C TX GPIO pin configuration  */
		  GPIO_InitStruct.Pin = I2C3_SCL_PIN;
		  HAL_GPIO_Init(I2C3_SCL_GPIO_PORT, &GPIO_InitStruct);

		  /* I2C RX GPIO pin configuration  */
		  GPIO_InitStruct.Pin = I2C3_SDA_PIN;
		  HAL_GPIO_Init(I2C3_SDA_GPIO_PORT, &GPIO_InitStruct);
	  }
}

/**
 *  Initialize the I2C instance
 */
int stm32_i2c_init (stm32_i2c_bus * bus)
{
	  if(bus->instance == I2C_ONE)
	  {
		  bus->handle.Instance             = I2C1;
		  bus->handle.Init.OwnAddress1     = I2C1_ADDRESS;
		  bus->handle.Init.OwnAddress2     = 0xFE;
	  }
	  else if(bus->instance == I2C_TWO)
	  {
		  bus->handle.Instance             = I2C2;
		  bus->handle.Init.OwnAddress1     = I2C2_ADDRESS;
		  bus->handle.Init.OwnAddress2     = 0xFE;
	  }
	  else if(bus->instance == I2C_THREE)
	  {
		  bus->handle.Instance             = I2C3;
		  bus->handle.Init.OwnAddress1     = I2C3_ADDRESS;
		  bus->handle.Init.OwnAddress2     = 0xFE;
	  }

	bus->handle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
	bus->handle.Init.ClockSpeed      = 100000;
	bus->handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	bus->handle.Init.DutyCycle       = I2C_DUTYCYCLE_16_9;
	bus->handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	bus->handle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

	return (HAL_I2C_Init(&bus->handle));
}

