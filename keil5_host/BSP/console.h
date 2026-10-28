#ifndef HOST_CONSOLE_H
#define HOST_CONSOLE_H

#include <stdint.h>

/* USART1 (PA9=TX, PA10=RX) <-> CH343P <-> 电脑。
   发送用轮询（调试输出量很小），接收用中断 + 环形缓冲，非阻塞。 */
void console_init(uint32_t baud);
void console_puts(const char *text);
/* 取一行（以 \r 或 \n 结束），有整行返回 1，否则 0。line 至少 64 字节。 */
int console_read_line(char *line, uint32_t size);
/* 清掉接收缓冲（例如上电时 CH343P 抖动产生的垃圾字节）。 */
void console_flush_input(void);
uint32_t console_dropped_bytes(void);

#endif
