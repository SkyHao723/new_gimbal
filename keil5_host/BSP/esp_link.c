#include "stm32g4xx_hal.h"
#include "esp_link.h"
#include "host_config.h"

/* USART2 <-> ESP32-S3 on PA2 (TX) / PA3 (RX), AF7.
   Split out of host_board.c because the link is now a real command input, not
   just an idle peripheral: RX uses the RXNE interrupt + ring buffer. */

#define ESP_RX_RING 256u
#define ESP_LINE_MAX 64u

static uint8_t rx_ring[ESP_RX_RING];
static volatile uint16_t rx_head, rx_tail;
static volatile uint32_t rx_dropped;
static volatile uint32_t rx_bytes;
static volatile uint32_t bad_lines;
static volatile uint32_t bad_bytes;
static volatile uint32_t ignored_lines;
static uint32_t muted_until_ms;
static uint32_t rate_ms;
static uint32_t rate_bytes;
static volatile uint32_t muted_drops;
static char line_buf[ESP_LINE_MAX];
static uint32_t line_len;
static uint8_t line_bad;
static int initialised;

static void esp_gpio_usart_init(uint32_t baud)
{
    uint32_t pclk1, brr;

    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
    (void)RCC->APB1ENR1;

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    (void)RCC->AHB2ENR;

    /* PA2 = TX, PA3 = RX, both AF7 */
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3)) |
                   (2u << GPIO_MODER_MODE2_Pos) | (2u << GPIO_MODER_MODE3_Pos);
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT2 | GPIO_OTYPER_OT3);
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED2 | GPIO_OSPEEDR_OSPEED3;
    GPIOA->PUPDR = (GPIOA->PUPDR & ~(GPIO_PUPDR_PUPD2 | GPIO_PUPDR_PUPD3)) |
                   GPIO_PUPDR_PUPD3_0;                /* keep RX pulled up while the ESP is idle */
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~((0xFu << 8) | (0xFu << 12))) |
                    (7u << 8) | (7u << 12);           /* AF7 = USART2 */

    USART2->CR1 = 0u;
    USART2->CR2 = 0u;
    USART2->CR3 = 0u;
    pclk1 = HAL_RCC_GetPCLK1Freq();
    brr = (pclk1 + baud / 2u) / baud;
    USART2->BRR = (uint16_t)brr;
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    HAL_NVIC_SetPriority(USART2_IRQn, 2u, 0u);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

void esp_link_init(uint32_t baud)
{
    rx_head = rx_tail = 0;
    rx_dropped = 0;
    rx_bytes = 0;
    line_len = 0;
    line_bad = 0;
    bad_lines = 0;
    bad_bytes = 0;
    ignored_lines = 0;
    muted_until_ms = 0u;
    rate_ms = 0u;
    rate_bytes = 0u;
    muted_drops = 0u;
    esp_gpio_usart_init(baud);
    initialised = 1;
}

int esp_link_ready(void)
{
    return initialised;
}

void esp_link_flush_input(void)
{
    rx_tail = rx_head;
    line_len = 0;
    line_bad = 0;
    USART2->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
}

void USART2_IRQHandler(void)
{
    uint32_t isr = USART2->ISR;
    if (isr & USART_ISR_RXNE) {
        uint8_t ch = (uint8_t)USART2->RDR;
        uint16_t next = (uint16_t)((rx_head + 1u) % ESP_RX_RING);
        ++rx_bytes;
        if (next == rx_tail) ++rx_dropped;
        else { rx_ring[rx_head] = ch; rx_head = next; }
    }
    if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) {
        USART2->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
    }
}

int esp_link_read_line(char *line, uint32_t size)
{
    while (rx_tail != rx_head) {
        uint8_t raw = rx_ring[rx_tail];
        char ch = (char)raw;
        rx_tail = (uint16_t)((rx_tail + 1u) % ESP_RX_RING);
        if (ch == '\r' || ch == '\n') {
            uint32_t n;
            if (line_bad) {                           /* discard the whole line */
                line_bad = 0;
                line_len = 0;
                ++bad_lines;
                continue;
            }
            if (line_len == 0) continue;              /* ignore empty lines */
            n = line_len;
            if (n >= size) n = size - 1u;
            /* ESP commands must start with '@'. Without this gate a shorted or
               looped TX/RX (or a chatty ESP32) would feed our own reply text back
               in as new commands and run away. */
            if (line_buf[0] != '@') {
                line_len = 0;
                ++ignored_lines;
                continue;
            }
            for (uint32_t i = 0; i < n; ++i) line[i] = line_buf[i + 1u];
            line[n - 1u] = 0;
            line_len = 0;
            return 1;
        }
        if (ch == 8u || ch == 127u) {                 /* backspace */
            if (line_len > 0) --line_len;
            continue;
        }
        /* Only printable ASCII is accepted. A floating / unconnected RX line
           produces garbage bytes; without this filter the main loop would
           execute random "commands" and flood the reply path. */
        if (raw < 0x20u || raw > 0x7Eu) {
            line_bad = 1;
            line_len = 0;
            ++bad_bytes;
            continue;
        }
        if (line_len + 1u < size && line_len + 1u < ESP_LINE_MAX) line_buf[line_len++] = ch;
    }
    return 0;
}

void esp_link_putc(char ch)
{
    if (!initialised) return;
    while (!(USART2->ISR & USART_ISR_TXE)) { }
    USART2->TDR = (uint8_t)ch;
}

void esp_link_puts(const char *text)
{
    while (*text) esp_link_putc(*text++);
}

uint32_t esp_link_dropped_bytes(void)
{
    return rx_dropped;
}

uint32_t esp_link_rx_bytes(void)
{
    return rx_bytes;
}

uint32_t esp_link_bad_lines(void)
{
    return bad_lines;
}

uint32_t esp_link_bad_bytes(void)
{
    return bad_bytes;
}

uint32_t esp_link_ignored_lines(void)
{
    return ignored_lines;
}

/* Noise guard: the ESP line is only trusted when it carries a plausible byte
   rate. A floating or looped line can dump >10 kB/s of text, which would burn
   the main loop and drown real input; mute it for a while instead. */
int esp_link_guard(uint32_t now_ms)
{
    uint32_t delta;

    if ((int32_t)(now_ms - rate_ms) < 0) rate_ms = now_ms;
    if ((now_ms - rate_ms) < 500u) return 0;
    delta = rx_bytes - rate_bytes;
    rate_bytes = rx_bytes;
    rate_ms = now_ms;
    if (muted_until_ms != 0u && (int32_t)(now_ms - muted_until_ms) < 0) {
        ++muted_drops;
        return 1;
    }
    muted_until_ms = 0u;
    if (delta > 1500u) {                       /* > 3 kB/s -> not real commands */
        muted_until_ms = now_ms + 5000u;
        ++muted_drops;
        return 2;                              /* caller may warn once */
    }
    return 0;
}

int esp_link_muted(void)
{
    return muted_until_ms != 0u;
}

uint32_t esp_link_muted_drops(void)
{
    return muted_drops;
}

void esp_link_pin_test(uint32_t out[4])
{
    uint32_t i;

    /* Idle levels with both pins as plain inputs (pull-up on PA3). */
    GPIOA->MODER = (GPIOA->MODER & ~((3u << (2u * 2u)) | (3u << (3u * 2u))));
    GPIOA->PUPDR = (GPIOA->PUPDR & ~((3u << (2u * 2u)) | (3u << (3u * 2u)))) | (2u << (3u * 2u));
    for (i = 0; i < 20000u; ++i) { __NOP(); }
    out[0] = (GPIOA->IDR & GPIO_PIN_2) ? 1u : 0u;
    out[1] = (GPIOA->IDR & GPIO_PIN_3) ? 1u : 0u;

    /* PA2 as push-pull output, PA3 input: drive low then high. */
    GPIOA->MODER = (GPIOA->MODER & ~((3u << (2u * 2u)) | (3u << (3u * 2u)))) | (1u << (2u * 2u));
    GPIOA->BRR = GPIO_PIN_2;
    for (i = 0; i < 20000u; ++i) { __NOP(); }
    out[2] = (GPIOA->IDR & GPIO_PIN_3) ? 1u : 0u;
    GPIOA->BSRR = GPIO_PIN_2;
    for (i = 0; i < 20000u; ++i) { __NOP(); }
    out[3] = (GPIOA->IDR & GPIO_PIN_3) ? 1u : 0u;

    /* Back to USART2 alternate function. */
    GPIOA->MODER = (GPIOA->MODER & ~((3u << (2u * 2u)) | (3u << (3u * 2u)))) | (2u << (2u * 2u)) | (2u << (3u * 2u));
    GPIOA->PUPDR = (GPIOA->PUPDR & ~((3u << (2u * 2u)) | (3u << (3u * 2u)))) | (1u << (2u * 2u)) | (2u << (3u * 2u));
}
