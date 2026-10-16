#ifndef BOARD_H
#define BOARD_H
#include <stdint.h>

void board_init(void);
uint32_t board_micros(void);
uint32_t board_millis(void);
void board_led(int on);
int board_driver_set(int enabled);
void board_pwm(float a, float b, float c);
int board_take_stop(void);
int board_get_command(char *dst);
void board_discard_commands(void);

#define BOARD_COMMAND_LENGTH 40
#endif
