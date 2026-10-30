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

/* HAL_Init() 打开 SysTick 中断后，必须有人递增 HAL tick。
   驱动板工程里这件事在 BSP/board.c；上位机板不编 board.c，所以放在这里。
   踩过的坑：缺这个函数时向量表落到启动文件的默认死循环，表现是串口打印到一半就停、
   指示灯也不闪 —— 第一次 1 ms tick 就把 CPU 卡死了。 */
void SysTick_Handler(void)
{
    HAL_IncTick();
}
