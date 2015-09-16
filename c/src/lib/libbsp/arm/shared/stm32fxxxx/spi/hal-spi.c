/**
 * @file hal-spi.c
 *
 * @ingroup spi
 *
 * @brief SPI driver for the STM32xxxx series processors. Provides a
 * SPI driver that registers with cpukit/dev/spi as a node in the
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
#include stm_header(TARGET_STM_PROCESSOR_PREFIX, spi)
#include stm_header(TARGET_STM_PROCESSOR_PREFIX, rcc)
#include stm_header(TARGET_STM_PROCESSOR_PREFIX, gpio)

#include <hal-spi.h>
#include <dev/spi/spi.h>

#include <math.h>
#include <rtems/irq-extension.h>

/**
 * SPI Instance enabled / disabled
 */
bool SPI_Enable[MAX_SPI_INSTANCES] =
{
#ifndef STM32_ENABLE_SPI1
      false,
#else
      STM32_ENABLE_SPI1,
#endif
#ifndef STM32_ENABLE_SPI2
      false,
#else
      STM32_ENABLE_SPI2,
#endif
#ifndef STM32_ENABLE_SPI3
      false,
#else
      STM32_ENABLE_SPI3,
#endif
#ifndef STM32_ENABLE_SPI4
      false,
#else
      STM32_ENABLE_SPI4,
#endif
#ifndef STM32_ENABLE_SPI5
      false,
#else
      STM32_ENABLE_SPI5,
#endif
#ifndef STM32_ENABLE_SPI6
      false,
#else
      STM32_ENABLE_SPI6,
#endif
};

/**
 * SPI pin configuration for each instances.
 * Pin and port details to be populated in configure.ac file
 */
stm32_spi_pin_config pin_config[MAX_SPI_INSTANCES] = {
#if (STM32_ENABLE_SPI1)
 {.port       = SPI1_PORT,
  .sck_pin    = SPI1_SCK_PIN,
  .miso_pin    = SPI1_MISO_PIN,
  .mosi_pin    = SPI1_MOSI_PIN,
  .alternate_func = GPIO_AF5_SPI1},
#endif
#if (STM32_ENABLE_SPI2)
 {.port       = SPI2_PORT,
  .sck_pin    = SPI2_SCK_PIN,
  .miso_pin    = SPI2_MISO_PIN,
  .mosi_pin    = SPI2_MOSI_PIN,
  .alternate_func = GPIO_AF5_SPI2},
#endif
#if (STM32_ENABLE_SPI3)
  {.port       = SPI3_PORT,
  .sck_pin    = SPI3_SCK_PIN,
  .miso_pin    = SPI3_MISO_PIN,
  .mosi_pin    = SPI3_MOSI_PIN,
  .alternate_func = GPIO_AF5_SPI3},
#endif
#if (STM32_ENABLE_SPI4)
 {.port       = SPI4_PORT,
  .sck_pin    = SPI4_SCK_PIN,
  .miso_pin    = SPI4_MISO_PIN,
  .mosi_pin    = SPI4_MOSI_PIN,
  .alternate_func = GPIO_AF5_SPI4},
#endif
#if (STM32_ENABLE_SPI5)
 {.port       = SPI5_PORT,
  .sck_pin    = SPI5_SCK_PIN,
  .miso_pin    = SPI5_MISO_PIN,
  .mosi_pin    = SPI5_MOSI_PIN,
  .alternate_func = GPIO_AF5_SPI5},
#endif
#if (STM32_ENABLE_SPI6)
 {.port       = SPI6_PORT,
  .sck_pin    = SPI6_SCK_PIN,
  .miso_pin    = SPI6_MISO_PIN,
  .mosi_pin    = SPI6_MOSI_PIN,
  .alternate_func = GPIO_AF5_SPI6},
#endif
};

/**
 * SPI Bus control
 */
stm32_spi_bus *bus[MAX_SPI_INSTANCES];

/**
 * Returns the base address of the port
 * TODO:: this function is to be pushed to gpio file and used from there
 */
GPIO_TypeDef* stm32_spi_get_gpio_port(gpio_port port)
{
   switch(port)
   {
   case PORTA:   return GPIOA;
               break;
   case PORTB:   return GPIOB;
               break;
   case PORTC:   return GPIOC;
               break;
   case PORTD:   return GPIOD;
               break;
   case PORTE:   return GPIOE;
               break;
   }
}

/**
 * Returns the base address of the SPI instances
 */
SPI_TypeDef* stm32_spi_get_spi_instance(SPI_Instance spi_instance)
{
   switch(spi_instance)
   {
   case SPI_ONE:   return SPI1;
               break;
   case SPI_TWO:   return SPI2;
               break;
   case SPI_THREE:   return SPI3;
               break;
#if (STM32_ENABLE_SPI4)
   case SPI_FOUR:   return SPI4;
               break;
#endif
#if (STM32_ENABLE_SPI5)
   case SPI_FIVE:   return SPI5;
               break;
#endif
#if (STM32_ENABLE_SPI6)
   case SPI_SIX:   return SPI6;
               break;
#endif
   }
}

/**
 * Initialize SPI clock
 */
void stm32_spi_initialize_spi_clock(SPI_Instance spi_instance)
{
   switch(spi_instance)
   {
   case SPI_ONE:   __HAL_RCC_SPI1_CLK_ENABLE();
               break;
   case SPI_TWO:   __HAL_RCC_SPI2_CLK_ENABLE();
               break;
   case SPI_THREE:   __HAL_RCC_SPI3_CLK_ENABLE();
               break;
#if (STM32_ENABLE_SPI4)
   case SPI_FOUR:   __HAL_RCC_SPI4_CLK_ENABLE();
               break;
#endif
#if (STM32_ENABLE_SPI5)
   case SPI_FIVE:   __HAL_RCC_SPI5_CLK_ENABLE();
               break;
#endif
#if (STM32_ENABLE_SPI6)
   case SPI_SIX:   __HAL_RCC_SPI6_CLK_ENABLE();
               break;
#endif
   }
}

/**
 * SPI get instance based vector number
 */
IRQn_Type stm32_get_spi_vec_num(SPI_Instance spi_instance)
{
   switch(spi_instance)
   {
   case SPI_ONE:   return (IRQn_Type)GET_SPI_VEC((SPI_ONE++));
               break;
   case SPI_TWO:   return (IRQn_Type)GET_SPI_VEC(SPI_TWO++);
               break;
   case SPI_THREE:   return (IRQn_Type)GET_SPI_VEC(SPI_THREE++);
               break;
   case SPI_FOUR:   return (IRQn_Type)GET_SPI_VEC(SPI_FOUR++);
               break;
   case SPI_FIVE:   return (IRQn_Type)GET_SPI_VEC(SPI_FIVE++);
               break;
   case SPI_SIX:   return (IRQn_Type)GET_SPI_VEC(SPI_SIX++);
               break;
   }
}


/**
 * SPI IOCTL
 */
static int stm32_spi_ioctl(
  spi_bus *base,
  uint32_t ioctl_cmd,
  unsigned long arg
)
{
   rtems_status_code sc = RTEMS_SUCCESSFUL;
   stm32_spi_bus *bus = (stm32_spi_bus *) base;

   _Assert(base != NULL);

   switch(ioctl_cmd)
   {
      /* Two line half-duplex mode */
      case SPI_IOCTL_DIR_2LINE:    __HAL_SPI_ENABLE(bus->handle);
                           bus->handle->Instance->CR1 &= ~(SPI_CR1_BIDIMODE);
                           __HAL_SPI_DISABLE(bus->handle);
                           return 0;
                           break;

      /* bi-direction line full-duplex mode */
      case SPI_IOCTL_DIR_1LINE:   __HAL_SPI_ENABLE(bus->handle);
                           bus->handle->Instance->CR1 |= SPI_CR1_BIDIMODE;
                           __HAL_SPI_DISABLE(bus->handle);
                           return 0;
                           break;

      /* Configure SPI instance to Master mode */
      case SPI_IOCTL_MODE_MASTER: __HAL_SPI_DISABLE(bus->handle);
                           bus->handle->Instance->CR1 |= (SPI_CR1_MSTR | SPI_CR1_SSI);
                           __HAL_SPI_ENABLE(bus->handle);\
                           return 0;
                           break;

      /* Configure SPI instance to Slave mode */
      case SPI_IOCTL_MODE_SLAVE:   __HAL_SPI_DISABLE(bus->handle);
                           bus->handle->Instance->CR1 &= ~(SPI_CR1_MSTR | SPI_CR1_SSI);
                           __HAL_SPI_ENABLE(bus->handle);
                           return 0;
                           break;

      /* Enable CRC Calculation */
      case SPI_IOCTL_ENABLE_CRC:   __HAL_SPI_DISABLE(bus->handle);
                           bus->handle->Instance->CR1 |= SPI_CR1_CRCEN;
                           __HAL_SPI_ENABLE(bus->handle);
                           return 0;
                           break;
      /* Disable CRC Calculation */
      case SPI_IOCTL_DISABLE_CRC:   __HAL_SPI_DISABLE(bus->handle);
                           bus->handle->Instance->CR1 &= ~(SPI_CR1_CRCEN);
                           __HAL_SPI_DISABLE(bus->handle);
                           return 0;
                           break;

      default:               return -1;
   }
}

/**
 * SPI Interrupt Event ISR
 */
static void stm32_spi_event_irq(void* arg)
{
   rtems_status_code sc;
   stm32_spi_bus *bus;

   HAL_SPI_IRQHandler((SPI_HandleTypeDef *) arg);

   sc = rtems_event_transient_send(bus->task_id);
   _Assert(sc == RTEMS_SUCCESSFUL);
}

/**
 * SPI Initiate Transfer
 */
static int stm32_spi_transfer(
  spi_bus *base,
  spi_msg *msgs,
  uint32_t msg_count
)
{
   rtems_status_code sc = RTEMS_SUCCESSFUL;
   stm32_spi_bus *bus = (stm32_spi_bus *) base;
   rtems_interval tickstart;

   _Assert(base != NULL);

   spi_msg *message = msgs;

   tickstart = rtems_clock_get_ticks_since_boot();

   /* Wait for the SPI bus to be ready */
   while (HAL_SPI_GetState(&bus->handle) != HAL_SPI_STATE_READY)
   {
      if((rtems_clock_get_ticks_since_boot() - tickstart) >  SPI_WAIT_TIMEOUT)
         return -1;
   }

   /* Hold CS Pin Low to activate Slave */
   HAL_GPIO_WritePin(stm32_spi_get_gpio_port(message->slave_select.port), GET_PIN(message->slave_select.pin), GPIO_PIN_RESET);

   /* Onlt Write */
   if(msgs->flags == SPI_M_WR)
   {
      if(HAL_SPI_Transmit_IT(&bus->handle, (uint8_t*)message->pTxBuf, message->len)!= HAL_OK){
         return -1;
      }
   }
   /* Only Read */
   else if(msgs->flags == SPI_M_RD)
   {
      if(HAL_SPI_Receive_IT(&bus->handle, (uint8_t*)message->pRxBuf, message->len)!= HAL_OK){
         return -1;
      }
   }
   /* Write followed by read */
   else if (msgs->flags == SPI_M_WR_RD)
   {
      if(HAL_SPI_TransmitReceive_IT(&bus->handle, (uint8_t*)message->pTxBuf, (uint8_t*)message->pRxBuf, message->len)!= HAL_OK){
         return -1;
      }
   }

   /* Hold CS Pin High to de-activate Slave */
   HAL_GPIO_WritePin(stm32_spi_get_gpio_port(message->slave_select.port), GET_PIN(message->slave_select.pin), GPIO_PIN_SET);

   bus->task_id = rtems_task_self();

   sc = rtems_event_transient_receive(RTEMS_WAIT, 20);

   if (sc != RTEMS_SUCCESSFUL) {
      return -ETIMEDOUT;
   }
}

/**
 * Initalize GPIO Pins
 * Note: If software based slave select, gpio pin for the slave select pin
 *      has to be configured and initialized by the application.
 */
void stm32_spi_gpio_init(SPI_Instance spi_instance)
{
   GPIO_InitTypeDef  GPIO_InitStruct;

   /* Initalize SPI Clock */
   stm32_spi_initialize_spi_clock(spi_instance);

   /* Initalize GPIO Clock */
   //TODO:: Initialize port based clock. To be implemented in gpio driver
   __HAL_RCC_GPIOA_CLK_ENABLE();
   __HAL_RCC_GPIOB_CLK_ENABLE();
   __HAL_RCC_GPIOC_CLK_ENABLE();
   __HAL_RCC_GPIOD_CLK_ENABLE();
   __HAL_RCC_GPIOE_CLK_ENABLE();

  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = pin_config[spi_instance].alternate_func;

  /* SPI SCK Pin */
  GPIO_InitStruct.Pin       = pin_config[spi_instance].sck_pin;
  HAL_GPIO_Init(pin_config[spi_instance].port, &GPIO_InitStruct);

  /* SPI MOSI Pin */
  GPIO_InitStruct.Pin       = pin_config[spi_instance].mosi_pin;
  HAL_GPIO_Init(pin_config[spi_instance].port, &GPIO_InitStruct);

  /* SPI MISO Pin */
  GPIO_InitStruct.Pin       = pin_config[spi_instance].miso_pin;
  HAL_GPIO_Init(pin_config[spi_instance].port, &GPIO_InitStruct);
}


/**
 * SPI Initalize Instance
 */
static int stm32_spi_init(stm32_spi_bus * bus)
{
   rtems_status_code sc;

   _Assert(bus != NULL);

   /* Initialize GPIO pins for SPI */
   stm32_spi_gpio_init(bus->instance);

/*********************** Initialize SPI ***************************/
   /* Set the SPI parameters */
   bus->handle->Instance               = stm32_spi_get_spi_instance(bus->instance);

   bus->handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
   bus->handle->Init.Direction         = SPI_DIRECTION_1LINE;
   bus->handle->Init.CLKPhase          = SPI_PHASE_1EDGE;
   bus->handle->Init.CLKPolarity       = SPI_POLARITY_LOW;
   bus->handle->Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
   bus->handle->Init.CRCPolynomial     = 7;
   bus->handle->Init.DataSize          = SPI_DATASIZE_16BIT;
   bus->handle->Init.FirstBit          = SPI_FIRSTBIT_MSB;
   bus->handle->Init.NSS               = SPI_NSS_SOFT;
   bus->handle->Init.TIMode            = SPI_TIMODE_DISABLE;

   bus->handle->Init.Mode = SPI_MODE_MASTER;
   if(HAL_SPI_Init(&bus->handle) != HAL_OK){

      (*bus->base.destroy)(&bus->base);
      rtems_set_errno_and_return_minus_one(EIO);
   }
/******************************************************************/

/********************* Install SPI vectors ***********************/
   sc = rtems_interrupt_handler_install(
         stm32_get_spi_vec_num(bus->instance),
         "SPI Event Insturrpt",
         RTEMS_INTERRUPT_UNIQUE,
         stm32_spi_event_irq,
         &bus->handle
   );

   if (sc != RTEMS_SUCCESSFUL) {

      (*bus->base.destroy)(&bus->base);
      rtems_set_errno_and_return_minus_one(EIO);
   }
/******************************************************************/
   return 0;
}

/**
 * spi Bus Destroy and De-initialise
  */
static int stm32_spi_deinit_destroy(stm32_spi_bus * bus)
{
     rtems_status_code sc;

     _Assert(bus != NULL);

     sc = rtems_interrupt_handler_remove(stm32_get_spi_vec_num(bus->instance), stm32_spi_event_irq, &bus->handle);
     _Assert(sc == RTEMS_SUCCESSFUL);
     (void) sc;

     spi_bus_destroy_and_free(&bus->base);
   return (HAL_SPI_DeInit(&bus->handle));
}


/* Register SPI Driver to RTEMS */
int stm32_bsp_register_spi(void)
{
   rtems_status_code sc;
   int err,inst_num=0;
   char bus_path[12];

   for(inst_num=0; inst_num < MAX_SPI_INSTANCES; inst_num++)
   {
      if(true == SPI_Enable[inst_num]) // Check if to enable this instance or not
      {
         bus[inst_num] = (stm32_spi_bus *) spi_bus_alloc_and_init(sizeof(*bus[inst_num]));

         if (bus[inst_num] == NULL) {
            return -ENOMEM;
         }
         bus[inst_num]->base.init       = stm32_spi_init;
         bus[inst_num]->base.transfer   = stm32_spi_transfer;
         bus[inst_num]->base.destroy      = stm32_spi_deinit_destroy;
         bus[inst_num]->base.de_init      = stm32_spi_deinit_destroy;
         bus[inst_num]->base.ioctl      = stm32_spi_ioctl;

         bus[inst_num]->instance = (SPI_Instance)inst_num;

         sprintf(bus_path, "/dev/spi%d",inst_num+1);

         err = spi_bus_register(&bus[inst_num]->base, bus_path);
         }
      }
   return err;
}
