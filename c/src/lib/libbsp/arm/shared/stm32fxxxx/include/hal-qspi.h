/**
 * @file hal-qspi.h
 *
 * @ingroup qspi
 *
 * @brief Public QSPI driver datatypes
 *
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * Author: Sudarshan Rajagopalan
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */

#ifndef INCLUDE_HAL_QSPI_H_
#define INCLUDE_HAL_QSPI_H_

#include <rtems.h>


/**
 * QSPI Command
 */
typedef struct
{
  /* QSPI Instruction */
  uint32_t instruction;

  /* Address of interest */
  uint32_t addr;

  /* Number of bytes to write or read */
  uint32_t num_bytes;

/* Pointer the the data buffer */
  uint8_t *pBuf;

} stm32_qspi_command;


rtems_status_code stm32_qspi_init(void);

rtems_status_code stm32_qspi_erase(stm32_qspi_command command);

uint32_t stm32_qspi_write(stm32_qspi_command command);

uint32_t stm32_qspi_read(stm32_qspi_command command);

void stm32_qspi_memory_mapped (void);

#endif /* INCLUDE_HAL_QSPI_H_ */
