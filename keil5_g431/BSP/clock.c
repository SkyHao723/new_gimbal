#include "stm32g4xx_hal.h"
#include "clock.h"

/* The supplied schematic has an 8 MHz crystal: / 2 * 85 / 2 = 170 MHz.
   Range 1 boost and four Flash wait states are required at this frequency. */
int clock_init(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef bus = {0};
    if (HAL_Init() != HAL_OK) return 0;
    __HAL_RCC_PWR_CLK_ENABLE();
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK) return 0;
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = RCC_PLLM_DIV2;
    osc.PLL.PLLN = 85;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = RCC_PLLQ_DIV4; /* 85 MHz, FDCAN divides by two. */
    osc.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) return 0;
    bus.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    bus.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    /* ST's HAL inserts the required transition delay through HCLK / 2. */
    bus.AHBCLKDivider = RCC_SYSCLK_DIV1;
    bus.APB1CLKDivider = RCC_HCLK_DIV1;
    bus.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&bus, FLASH_LATENCY_4) != HAL_OK) return 0;
    return HAL_RCC_GetHCLKFreq() == 170000000UL;
}
