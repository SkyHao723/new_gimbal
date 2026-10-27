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
#define BOARD_CAN_COMMAND_ID 0x201u
#define BOARD_CAN_REPLY_ID 0x281u
#endif
