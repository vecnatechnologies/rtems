/**
 * @file hal-i2c.c
 *
 * @ingroup can
 *
 * @brief I2C driver for the STM32xxxx series processors. Provides a
 * I2C driver that registers with cpukit/dev/i2c as a node in the
 * filesystem.
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

#include <math.h>
#include <rtems/irq-extension.h>

stm32_i2c_bus *bus[MAX_I2C_INSTANCES];
I2C_HandleTypeDef I2cHandle;

stm32_i2c_pin_config i2c_pin_config[MAX_I2C_INSTANCES] = {
#if (STM32_ENABLE_I2C1)
													{.port 		= I2C1_PORT,
													.sck_pin 	= I2C1_SCK_PIN,
													.sda_pin 	= I2C1_SDA_PIN,
													.alternate_func = GPIO_AF4_I2C1},
#endif
#if (STM32_ENABLE_I2C2)
													{.port 		= I2C2_PORT,
													.sck_pin 	= I2C2_SCK_PIN,
													.sda_pin 	= I2C2_SDA_PIN,
													.alternate_func = GPIO_AF4_I2C2},
#endif
#if (STM32_ENABLE_I2C3)
													{.port 		= I2C3_PORT,
													.sck_pin 	= I2C3_SCK_PIN,
													.sda_pin 	= I2C3_SDA_PIN,
													.alternate_func = GPIO_AF4_I2C3},
#endif
#if (STM32_ENABLE_I2C4)
													{.port 		= I2C4_PORT,
													.sck_pin 	= I2C4_SCK_PIN,
													.sda_pin 	= I2C4_SDA_PIN,
													.alternate_func = GPIO_AF4_I2C4},
#endif
};

const bool I2C_Enable[MAX_I2C_INSTANCES] = {
#ifndef STM32_ENABLE_I2C1
		false,
#else
		STM32_ENABLE_I2C1,
#endif
#ifndef STM32_ENABLE_I2C2
		false,
#else
		STM32_ENABLE_I2C2,
#endif
#ifndef STM32_ENABLE_I2C3
		false,
#else
		STM32_ENABLE_I2C3,
#endif
#ifndef STM32_ENABLE_I2C4
		false,
#else
		STM32_ENABLE_I2C4,
#endif
};

/**
 * I2C Own Device Addresses
 */
uint32_t I2C_Address[MAX_I2C_INSTANCES] = {
#if (STM32_ENABLE_I2C1)
		STM32_I2C1_ADDRESS,
#else
		0x00,
#endif
#if (STM32_ENABLE_I2C2)
		STM32_I2C2_ADDRESS,
#else
		0x00,
#endif
#if (STM32_ENABLE_I2C3)
		STM32_I2C3_ADDRESS,
#else
		0x00,
#endif
#if (STM32_ENABLE_I2C4)
		STM32_I2C4_ADDRESS,
#else
		0x00,
#endif
};

/*********** Private Funvtions **********************/


/**
 * I2C get instance based vector number
 */
IRQn_Type stm32_get_i2c_vec_num(I2C_Instance i2c_instance)
{
	switch(i2c_instance)
	{
	case I2C_ONE:	return (IRQn_Type)GET_I2C_VEC((I2C_ONE++));
					break;
	case I2C_TWO:	return (IRQn_Type)GET_I2C_VEC(I2C_TWO++);
					break;
	case I2C_THREE:	return (IRQn_Type)GET_I2C_VEC(I2C_THREE++);
					break;
	case I2C_FOUR:	return (IRQn_Type)GET_I2C_VEC(I2C_FOUR++);
					break;
	}
}

/**
 * Initialize I2C clock
 */
void stm32_i2c_initialize_i2c_clock(I2C_Instance i2c_instance)
{
	switch(i2c_instance)
	{
	case I2C_ONE:	__HAL_RCC_I2C1_CLK_ENABLE();
					break;
	case I2C_TWO:	__HAL_RCC_I2C2_CLK_ENABLE();
					break;
	case I2C_THREE:	__HAL_RCC_I2C3_CLK_ENABLE();
					break;
#if (STM32_ENABLE_I2C4)
	case I2C_FOUR:	__HAL_RCC_I2C4_CLK_ENABLE();
					break;
#endif
	}
}

/**
 * Returns the base address of the I2C instances
 */
I2C_TypeDef* stm32_i2c_get_i2c_instance(I2C_Instance i2c_instance)
{
	switch(i2c_instance)
	{
	case I2C_ONE:	return I2C1;
					break;
	case I2C_TWO:	return I2C2;
					break;
	case I2C_THREE:	return I2C3;
					break;
#if (STM32_ENABLE_I2C4)
	case I2C_FOUR:	return I2C4;
					break;
#endif
	}
}

/**
 * I2C Interrupt Event ISR
 */
static void stm32_i2c_event_irq(void* arg)
{
	rtems_status_code sc;
	stm32_i2c_bus *bus;

	HAL_I2C_EV_IRQHandler((I2C_HandleTypeDef *) arg);

	sc = rtems_event_transient_send(bus->task_id);
	_Assert(sc == RTEMS_SUCCESSFUL);
}

/**
 * I2C Initiate Transfer
 */
static int stm32_i2c_transfer(
  i2c_bus *base,
  i2c_msg *msgs,
  uint32_t msg_count
)
{
	rtems_status_code sc = RTEMS_SUCCESSFUL;
	stm32_i2c_bus *bus = (stm32_i2c_bus *) base;

	i2c_msg *message = msgs->buf;

	/* Wait for the I2C bus to be ready */
	while (HAL_I2C_GetState(&bus->handle) != HAL_I2C_STATE_READY);

	if(msgs->flags == 0)
	{
		if(HAL_I2C_Master_Transmit_IT(&bus->handle, (uint16_t)message->addr, (uint8_t*)message->buf, msgs->len)!= HAL_OK){
			return -1;
		}
	}
	else if(msgs->flags == I2C_M_RD)
	{
		if(HAL_I2C_Master_Receive_IT(&bus->handle, (uint16_t)message->addr, (uint8_t*)message->buf, msgs->len) != HAL_OK){
			return -1;
		}
	}

	bus->task_id = rtems_task_self();

	sc = rtems_event_transient_receive(RTEMS_WAIT, 20);

	if (sc != RTEMS_SUCCESSFUL) {
		return -ETIMEDOUT;
	}
}

/**
 * I2C Bus Destroy and De-initialise
  */
static int stm32_i2c_deinit_destroy(stm32_i2c_bus * bus)
{
	rtems_status_code sc;

	sc = rtems_interrupt_handler_remove(stm32_get_i2c_vec_num(bus->instance), stm32_i2c_event_irq, &bus->handle);
	_Assert(sc == RTEMS_SUCCESSFUL);
	(void) sc;

	i2c_bus_destroy_and_free(&bus->base);
	return (HAL_I2C_DeInit(&bus->handle));
}

/**
 * Initialize GPIO pins
 */
void stm32_i2c_gpio_init(stm32_i2c_bus * bus)
{

	GPIO_InitTypeDef  GPIO_InitStruct;

	stm32_i2c_initialize_i2c_clock(bus->instance);

	/* Initialize GPIO Clock */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull      = GPIO_PULLUP;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = i2c_pin_config[bus->instance].alternate_func;

	/* I2C SCL GPIO pin configuration  */
	GPIO_InitStruct.Pin       = i2c_pin_config[bus->instance].sck_pin;
	HAL_GPIO_Init(i2c_pin_config[bus->instance].port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin       = i2c_pin_config[bus->instance].sda_pin;
	HAL_GPIO_Init(i2c_pin_config[bus->instance].port, &GPIO_InitStruct);
}

/* Register I2C Driver to RTEMS */
int stm32_bsp_register_i2c(void)
{
	rtems_status_code sc;
	int err,inst_num=0;
	char bus_path[12];

	for(inst_num=0; inst_num < MAX_I2C_INSTANCES; inst_num++)
	{
		if(true == I2C_Enable[inst_num]) // Check if to enable this instance or not
		{
			bus[inst_num] = (stm32_i2c_bus *) i2c_bus_alloc_and_init(sizeof(*bus[inst_num]));

			if (bus[inst_num] == NULL) {
				return -ENOMEM;
			}

/*********************** Initialize I2C ***************************/

			bus[inst_num]->instance = (I2C_Instance)inst_num;

			stm32_i2c_gpio_init(bus[inst_num]);

			if(stm32_i2c_init(bus[inst_num]) != HAL_OK) {

				(*bus[inst_num]->base.destroy)(&bus[inst_num]->base);
				rtems_set_errno_and_return_minus_one(EIO);
			}

/******************************************************************/

/********************* Install I2C vectors ***********************/

			sc = rtems_interrupt_handler_install(
					stm32_get_i2c_vec_num(inst_num),
					"I2C Event Insturrpt",
					RTEMS_INTERRUPT_UNIQUE,
					stm32_i2c_event_irq,
					&bus[inst_num]->handle
			);

			if (sc != RTEMS_SUCCESSFUL) {

				(*bus[inst_num]->base.destroy)(&bus[inst_num]->base);
				rtems_set_errno_and_return_minus_one(EIO);
			}
/******************************************************************/

			bus[inst_num]->base.transfer 	= stm32_i2c_transfer;
			bus[inst_num]->base.destroy  	= stm32_i2c_deinit_destroy;
			bus[inst_num]->base.set_clock 	= stm32_i2c_set_clock;
			sprintf(bus_path, "/dev/i2c%d",inst_num+1);

			err = i2c_bus_register(&bus[inst_num]->base, bus_path);
		}
	}
	return err;
}
