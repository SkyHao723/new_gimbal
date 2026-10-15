/* ============================================================================
 * clock.c -- system clock setup (shared by all test firmwares)
 *
 * The original reference project's SystemInit() does NOT configure the PLL,
 * so the chip actually runs at HSI 8MHz. That makes PWM 2.8kHz instead of
 * 25kHz and breaks I2C timing assumptions. This sets 72MHz explicitly.
 * Falls back to HSI/2*16 = 64MHz if the 8MHz HSE crystal does not start.
 * ==========================================================================*/
#include "stm32f10x.h"

void RCC_Configuration(void)
{
    ErrorStatus hse_ok;
    uint32_t t;

    RCC_DeInit();
    RCC_HSEConfig(RCC_HSE_ON);
    hse_ok = RCC_WaitForHSEStartUp();

    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    FLASH_SetLatency(FLASH_Latency_2);
    RCC_HCLKConfig(RCC_SYSCLK_Div1);      /* AHB  = 72MHz */
    RCC_PCLK2Config(RCC_HCLK_Div1);       /* APB2 = 72MHz */
    RCC_PCLK1Config(RCC_HCLK_Div2);       /* APB1 = 36MHz */

    if(hse_ok == SUCCESS)
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);    /* 8*9  = 72MHz */
    else
        RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_16);   /* 4*16 = 64MHz */

    RCC_PLLCmd(ENABLE);
    t = 0;
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET && ++t < 0x200000);
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    while(RCC_GetSYSCLKSource() != 0x08);
    SystemCoreClockUpdate();
}
