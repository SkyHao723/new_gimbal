#ifndef G431_BOARD_CONFIG_H
#define G431_BOARD_CONFIG_H

#include "stm32g4xx_hal.h"

/* Driver pins verified in DRV8316C_原理图审核记录.md (2026-10-01). */
#define DRIVER_PORT GPIOB
#define DRIVER_SLEEP_PIN GPIO_PIN_0
#define DRIVER_OFF_PIN GPIO_PIN_1
#define DRIVER_FAULT_PIN GPIO_PIN_2
#define DRIVER_FAULT_IRQn EXTI2_IRQn

/* CN1 pin 1=PB7/SDA, pin 2=PB6/SCL, pin 3=GND, pin 4=3.3V.
   G431 PB6 has no I2C1_SCL alternate function: use GPIO bit-banging. */
#define BOARD_ENCODER_PORT GPIOB
#define BOARD_ENCODER_SCL_PIN GPIO_PIN_6
#define BOARD_ENCODER_SDA_PIN GPIO_PIN_7
#define BOARD_ENCODER_GPIO_CLOCK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()

/* TJA1042: PB8/RX, PB9/TX, PB5/STB (high=standby). */
/* 多板共用一条总线时每块板必须有不同的节点号（1..7）。
   命令 ID = 0x200 + NODE，回复 ID = 0x280 + NODE。
   0x200 是广播 ID：所有节点都接收（D = 全体急停最有用；T/E 会同时作用到全部板子，慎用）。
   烧录前按板上标签改这里的 BOARD_CAN_NODE。 */
#define BOARD_CAN_NODE 1u
#define BOARD_CAN_COMMAND_ID (0x200u + BOARD_CAN_NODE)
#define BOARD_CAN_REPLY_ID (0x280u + BOARD_CAN_NODE)
#define BOARD_CAN_BROADCAST_ID 0x200u
#endif
