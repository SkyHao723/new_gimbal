#ifndef BOARD_H
#define BOARD_H
#include <stdint.h>

void board_init(void);
uint32_t board_micros(void);
uint32_t board_millis(void);
void board_led(int on);
int board_driver_set(int enabled);
int board_driver_wake(void);
void board_pwm(float a, float b, float c);
uint32_t board_pwm_low_window(void);
int board_take_stop(void);
int board_get_command(char *dst);
void board_discard_commands(void);
void board_request_stop(unsigned int reason);

#define BOARD_STOP_COMMAND 1u
#define BOARD_STOP_DRIVER 2u
#define BOARD_STOP_CAN 4u
#define BOARD_STOP_CURRENT 8u
#define BOARD_STOP_SENSOR 16u

#define BOARD_COMMAND_LENGTH 40
#endif
