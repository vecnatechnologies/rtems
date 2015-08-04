#ifndef STM32_CAN_H
#define STM32_CAN_H
#include <stdint.h>
#include <stdbool.h>

/* CAN Status*/
typedef enum
{
  CAN_OK  = 0,
  CAN_ERROR,
  CAN_BUSY,
  CAN_TIMEOUT

} CAN_Status;

/*CAN Instance*/
typedef enum
{
  CAN_ONE= 0,
  CAN_TWO

} CAN_Instance;

/* Interrupt Enale */
typedef enum
{
  DISABLE_INT= 0,
  ENABLE_INT

} CAN_InterruptEnable;

/* CAN Hardware FIFOs*/
typedef enum
{
  FIFO0= 0x0,
  FIFO1

} CAN_FIFO;

typedef struct
{
  uint32_t s1;
  uint32_t s2;
  uint8_t prescaler;
  bool error;

} CAN_Timing_Values;

typedef struct
{
  uint32_t MsgID;
  uint8_t MsgLen;
  uint8_t MsgData[8];

} CAN_MessageFrame;

int stm32_bsp_register_can(void);

#endif
