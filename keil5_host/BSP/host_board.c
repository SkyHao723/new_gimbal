#include "stm32g4xx_hal.h"
#include "host_board.h"
#include "host_config.h"

void host_led(int on)
{
#if HOST_LED_ACTIVE_LOW
    if (on) HOST_LED_PORT->BSRR = (uint32_t)HOST_LED_PIN << 16;
    else    HOST_LED_PORT->BSRR = HOST_LED_PIN;
#else
    if (on) HOST_LED_PORT->BSRR = HOST_LED_PIN;
    else    HOST_LED_PORT->BSRR = (uint32_t)HOST_LED_PIN << 16;
#endif
}

void host_led_toggle(void)
{
    static int state;
    state = !state;
    host_led(state);
}

void host_board_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA5 = 工作指示灯 */
    gpio.Pin = HOST_LED_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(HOST_LED_PORT, &gpio);
    host_led(0);
}

void host_esp_uart_init(uint32_t baud)
{
    GPIO_InitTypeDef gpio = {0};
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    /* PA2 = USART2_TX (AF7), PA3 = USART2_RX (AF7) */
    gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gpio);

    USART2->CR1 = 0u;
    USART2->CR2 = 0u;
    USART2->CR3 = 0u;
    USART2->BRR = (uint16_t)((pclk1 + baud / 2u) / baud);
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;   /* 先不收，避免噪声字节 */
}
