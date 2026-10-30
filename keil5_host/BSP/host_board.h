#ifndef HOST_BOARD_H
#define HOST_BOARD_H

#include <stdint.h>

/* 上位机板的板级初始化：LED、ESP32 串口（暂缓） */
void host_board_init(void);
void host_led(int on);
void host_led_toggle(void);
#endif
