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

stm32_i2c_bus *bus[I2C_INSTANCES];
I2C_HandleTypeDef I2cHandle;


/******************** Externs **********************/
extern rtems_vector_number I2C_IRQ_VEC[MAX_I2C_INSTANCES];
extern bool I2C_Enable[MAX_I2C_INSTANCES];

/*********** Private Funvtions **********************/

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

	  sc = rtems_interrupt_handler_remove(I2C_IRQ_VEC[bus->instance], stm32_i2c_event_irq, &bus->handle);
	  _Assert(sc == RTEMS_SUCCESSFUL);
	  (void) sc;

	  i2c_bus_destroy_and_free(&bus->base);
	return (HAL_I2C_DeInit(&bus->handle));
}

/* Register I2C Driver to RTEMS */
int stm32_bsp_register_i2c(void)
{
	rtems_status_code sc;
	int err,inst_num=0;
	char bus_path[12];

	_Assert(I2C_INSTANCES <= MAX_I2C_INSTANCES);

	for(inst_num=0; inst_num < I2C_INSTANCES; inst_num++)
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
					I2C_IRQ_VEC[inst_num],
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

			bus[inst_num]->base.transfer = stm32_i2c_transfer;
			bus[inst_num]->base.destroy  = stm32_i2c_deinit_destroy;

			sprintf(bus_path, "/dev/i2c%d",inst_num+1);

			err = i2c_bus_register(&bus[inst_num]->base, bus_path);
		}
	}
	return err;
}
