#ifndef HOST_CONSOLE_H
#define HOST_CONSOLE_H

#include <stdint.h>

/* USART1 (PA9=TX, PA10=RX) <-> CH343P <-> 电脑。
   发送用轮询（调试输出量很小），接收用中断 + 环形缓冲，非阻塞。 */
void console_init(uint32_t baud);
void console_puts(const char *text);
/* 取一行（以 \r 或 \n 结束），有整行返回 1，否则 0。line 至少 96 字节。
   从 USB 串口输入时会做本地回显与行编辑：退格删一个字符、Ctrl-U 清整行、
   Ctrl-C 放弃整行、方向键等 ESC 序列被吞掉。 */
int console_read_line(char *line, uint32_t size);
/* 打印提示符 "> "（只走 USB 口，供有线串口终端使用）。 */
void console_prompt(void);
/* 清掉接收缓冲（例如上电时 CH343P 抖动产生的垃圾字节）。 */
void console_flush_input(void);
uint32_t console_dropped_bytes(void);

/* Output routing: replies go back to the link the request came from.
   CONSOLE_OUT_BOTH is the default (boot banner and asynchronous logs). */
#define CONSOLE_OUT_USB  1
#define CONSOLE_OUT_ESP  2
#define CONSOLE_OUT_BOTH 3
void console_set_route(int route);
int  console_route(void);
#endif
