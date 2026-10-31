#ifndef HOST_CAN_H
#define HOST_CAN_H

#include <stdint.h>

/* FDCAN1（PB8=RX / PB9=TX，AF9）作为三块驱动板的 CAN 主机。
   参数与驱动板固件完全一致：500 kbit/s、经典帧、PLLQ/2、采样点 82.35%。 */

typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
} CanFrame;

int can_host_init(void);
/* 发一帧标准帧；返回 1 表示已进入发送 FIFO。 */
int can_host_send(uint32_t id, const uint8_t *data, uint8_t len);
/* 取一帧收到的报文，没有返回 0。 */
int can_host_recv(CanFrame *frame);
/* bus-off 之后自动重启：主循环里每 ~20 ms 调一次即可。 */
void can_host_service(void);

/* 统计 */
uint32_t can_host_rx_count(void);
uint32_t can_host_tx_dropped(void);
uint32_t can_host_bus_off_count(void);
uint32_t can_host_error_count(void);
uint32_t can_host_psr(void);

/* 把 ASCII 命令编成驱动板能懂的帧：
   E/D/?/R/H  -> 1 字节；
   T/V/L/M    -> 操作码 + float32 小端。
   node = 1..7 发给对应驱动板，node = 0 表示广播 0x200。
   返回 1 成功，0 命令格式不对。 */
int can_host_send_command(uint8_t node, const char *text);

/* Internal-loopback self test (register level, no HAL re-init). */
int can_host_loopback_test(uint32_t *out);
int can_host_external_loopback_test(uint32_t *out);

#endif
