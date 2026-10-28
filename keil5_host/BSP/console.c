#include "stm32g4xx_hal.h"
#include "console.h"
#include "host_config.h"
#include <stdio.h>

/* USART1 直接写寄存器：上位机板只需要一条调试串口，用 HAL 的 UART 驱动会多带
   一份与驱动板无关的代码。这里发射走轮询（TXE 等待，输出量小），接收走 RXNE
   中断 + 环形缓冲，主循环从不阻塞在串口上。 */

#define RX_RING 256u
#define LINE_MAX 64u

static uint8_t rx_ring[RX_RING];
static volatile uint16_t rx_head, rx_tail;
static volatile uint32_t rx_dropped;
static char line_buf[LINE_MAX];
static uint32_t line_len;

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
    console_usart_init(baud);
}

void console_flush_input(void)
{
    rx_tail = rx_head;
    line_len = 0;
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
        if (ch == '\r' || ch == '\n') {
            uint32_t n;
            if (line_len == 0) continue;              /* 忽略空行 */
            n = line_len;
            if (n >= size) n = size - 1u;             /* 调用方缓冲不足时截断 */
            for (uint32_t i = 0; i < n; ++i) line[i] = line_buf[i];
            line[n] = 0;
            line_len = 0;
            return 1;
        }
        if (ch == 8u || ch == 127u) {                 /* 退格 */
            if (line_len > 0) --line_len;
            continue;
        }
        if (line_len + 1u < size && line_len + 1u < LINE_MAX) line_buf[line_len++] = ch;
    }
    return 0;
}

void console_puts(const char *text)
{
    while (*text) {
        while (!(USART1->ISR & USART_ISR_TXE)) { }
        USART1->TDR = (uint8_t)*text++;
    }
}

uint32_t console_dropped_bytes(void)
{
    return rx_dropped;
}

/* printf 重定向：Keil 的 printf 通过 fputc 输出（工程里勾了 MicroLIB）。 */
int fputc(int ch, FILE *stream)
{
    (void)stream;
    while (!(USART1->ISR & USART_ISR_TXE)) { }
    USART1->TDR = (uint8_t)ch;
    return ch;
}
