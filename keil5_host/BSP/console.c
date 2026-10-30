#include "stm32g4xx_hal.h"
#include "console.h"
#include "host_config.h"
#include "esp_link.h"
#include <stdio.h>

/* USART1 直接写寄存器：上位机板只需要一条调试串口，用 HAL 的 UART 驱动会多带
   一份与驱动板无关的代码。这里发射走轮询（TXE 等待，输出量小），接收走 RXNE
   中断 + 环形缓冲，主循环从不阻塞在串口上。 */

#define RX_RING 256u
#define LINE_MAX 96u

static uint8_t rx_ring[RX_RING];
static volatile uint16_t rx_head, rx_tail;
static volatile uint32_t rx_dropped;
static char line_buf[LINE_MAX];
static uint32_t line_len;
static volatile int out_route = CONSOLE_OUT_BOTH;
static int esc_state;                    /* 0=none 1=saw ESC 2=saw ESC [ */
static int swallow_lf;                   /* after a CR put out a fresh prompt once */

/* Write one byte to the selected sink(s). ESP is skipped until esp_link_init()
   has run, so an early failure cannot hang on a disabled USART2. */
static void console_put_byte(char ch)
{
    int to_esp = (out_route & CONSOLE_OUT_ESP) != 0;
    int to_usb = (out_route & CONSOLE_OUT_USB) != 0;

    if (to_esp && !esp_link_ready()) {          /* fall back so nothing is lost */
        to_esp = 0;
        to_usb = 1;
    }
    if (to_usb) {
        while (!(USART1->ISR & USART_ISR_TXE)) { }
        USART1->TDR = (uint8_t)ch;
    }
    if (to_esp) esp_link_putc(ch);
}

void console_set_route(int route)
{
    out_route = (route == CONSOLE_OUT_USB || route == CONSOLE_OUT_ESP) ? route : CONSOLE_OUT_BOTH;
}

int console_route(void)
{
    return out_route;
}

static void console_usart_init(uint32_t baud)
{
    uint32_t pclk2, brr;

    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    (void)RCC->APB2ENR;

    /* PA9 = TX, PA10 = RX, AF7 */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    (void)RCC->AHB2ENR;
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE9 | GPIO_MODER_MODE10)) |
                   (2u << GPIO_MODER_MODE9_Pos) | (2u << GPIO_MODER_MODE10_Pos);
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT9 | GPIO_OTYPER_OT10);
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED9 | GPIO_OSPEEDR_OSPEED10;
    GPIOA->PUPDR = (GPIOA->PUPDR & ~(GPIO_PUPDR_PUPD9 | GPIO_PUPDR_PUPD10)) |
                   GPIO_PUPDR_PUPD9_0;                    /* RX 上拉，悬空线不产生假字节 */
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~((0xFu << 4) | (0xFu << 8))) |
                    (7u << 4) | (7u << 8);                 /* AF7 = USART1 */

    USART1->CR1 = 0;
    USART1->CR2 = 0;
    USART1->CR3 = 0;
    /* OVER8 = 0 (16 倍采样)：BRR = PCLK / baud，170 MHz / 115200 = 1476。 */
    pclk2 = HAL_RCC_GetPCLK2Freq();
    brr = (pclk2 + baud / 2u) / baud;
    USART1->BRR = (uint16_t)brr;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    HAL_NVIC_SetPriority(USART1_IRQn, 2u, 0u);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void console_init(uint32_t baud)
{
    rx_head = rx_tail = 0;
    rx_dropped = 0;
    line_len = 0;
    esc_state = 0;
    swallow_lf = 0;
    console_usart_init(baud);
}

/* ---- 本地回显辅助：只写 USB 口（有线终端），不影响 ESP 那一路 ---- */

static void usb_write_char(char ch)
{
    while (!(USART1->ISR & USART_ISR_TXE)) { }
    USART1->TDR = (uint8_t)ch;
}

static void usb_puts(const char *text)
{
    while (*text) usb_write_char(*text++);
}

static void usb_erase_chars(uint32_t count)
{
    while (count-- > 0u) {
        usb_write_char('\b');
        usb_write_char(' ');
        usb_write_char('\b');
    }
}

void console_flush_input(void)
{
    rx_tail = rx_head;
    line_len = 0;
    esc_state = 0;
    /* 清掉可能残留的溢出/噪声标志 */
    USART1->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
}

void USART1_IRQHandler(void)
{
    uint32_t isr = USART1->ISR;
    if (isr & USART_ISR_RXNE) {
        uint8_t ch = (uint8_t)USART1->RDR;
        uint16_t next = (uint16_t)((rx_head + 1u) % RX_RING);
        if (next == rx_tail) ++rx_dropped;
        else { rx_ring[rx_head] = ch; rx_head = next; }
    }
    if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) {
        USART1->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
    }
}

int console_read_line(char *line, uint32_t size)
{
    while (rx_tail != rx_head) {
        char ch = (char)rx_ring[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1u) % RX_RING);

        if (swallow_lf != 0) {
            swallow_lf = 0;
            if (ch == '\n') continue;                  /* CRLF 只处理一次 */
        }

        /* 方向键 / 功能键发送的是 ESC 序列（ESC [ A 之类）：整段吞掉，
           否则它们会变成命令里的垃圾字符。 */
        if (esc_state != 0) {
            if (esc_state == 1) {
                esc_state = (ch == '[' || ch == 'O') ? 2 : 0;
                continue;
            }
            if (ch >= 0x40 && ch <= 0x7E) esc_state = 0;   /* sequence end */
            continue;
        }
        if (ch == 0x1B) { esc_state = 1; continue; }

        if (ch == '\r' || ch == '\n') {
            uint32_t n;
            if (line_len == 0) {                       /* 空行：换行并重新给提示符 */
                usb_puts("\r\n> ");
                swallow_lf = (ch == '\r');
                continue;
            }
            usb_puts("\r\n");
            n = line_len;
            if (n >= size) n = size - 1u;                  /* 调用方缓冲不足时截断 */
            for (uint32_t i = 0; i < n; ++i) line[i] = line_buf[i];
            line[n] = 0;
            line_len = 0;
            return 1;
        }
        if (ch == 8u || ch == 127u) {                      /* 退格：删一个字符并回删显示 */
            if (line_len > 0) --line_len;
            usb_erase_chars(1u);
            continue;
        }
        if (ch == 3u) {                                    /* Ctrl-C：放弃整行 */
            usb_erase_chars(line_len);
            line_len = 0;
            usb_puts("^C\r\n> ");
            continue;
        }
        if (ch == 21u) {                                   /* Ctrl-U：清空整行 */
            usb_erase_chars(line_len);
            line_len = 0;
            continue;
        }
        if (ch == '\t') ch = ' ';                           /* TAB 当空格 */
        if (ch < 0x20 || ch > 0x7E) continue;               /* 其余控制字节丢掉 */
        if (line_len + 1u < size && line_len + 1u < LINE_MAX) {
            line_buf[line_len++] = ch;
            usb_write_char(ch);                             /* 本地回显 */
        }
    }
    return 0;
}

void console_prompt(void)
{
    usb_puts("> ");
}

void console_puts(const char *text)
{
    while (*text) console_put_byte(*text++);
}

uint32_t console_dropped_bytes(void)
{
    return rx_dropped;
}

/* printf 重定向：Keil 的 printf 通过 fputc 输出（工程里勾了 MicroLIB）。 */
int fputc(int ch, FILE *stream)
{
    (void)stream;
    console_put_byte((char)ch);
    return ch;
}
