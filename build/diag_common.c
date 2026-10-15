/* 诊断固件：时钟配置 + 定时器中断处理
 * 仅用于排查"PWM 无输出"，不参与正式固件逻辑 */
#include "stm32f10x.h"

volatile uint32_t time1_cntr;
volatile uint32_t time2_cntr;

/* ------------------------------------------------------------------
 * 把系统时钟配到 72MHz（HSE 8MHz x 9）
 * ★ 原工程 SystemInit() 没有配置 PLL，默认跑在 HSI 8MHz
 *   这会让 PWM 频率从 25kHz 变成 2.8kHz，串口和延时也全靠"两边一起错"才对上
 * 这里先确认 HSE 是否工作，不行就退回 HSI/2 x 16 = 64MHz
 * ------------------------------------------------------------------ */
void RCC_Configuration(void)
{
	ErrorStatus hes_ok;
	uint32_t t;

	RCC_DeInit();
	RCC_HSEConfig(RCC_HSE_ON);
	hes_ok = RCC_WaitForHSEStartUp();

	if(hes_ok == SUCCESS)
	{
		/* Flash: 2 个等待周期 */
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
		FLASH_SetLatency(FLASH_Latency_2);

		RCC_HCLKConfig(RCC_SYSCLK_Div1);     /* AHB  = 72MHz */
		RCC_PCLK2Config(RCC_HCLK_Div1);      /* APB2 = 72MHz */
		RCC_PCLK1Config(RCC_HCLK_Div2);      /* APB1 = 36MHz */

		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);   /* 8x9 = 72MHz */
		RCC_PLLCmd(ENABLE);
		t = 0; while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET && ++t < 0x200000);
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		while(RCC_GetSYSCLKSource() != 0x08);
	}
	else
	{
		/* HSE 没起振，用 HSI/2 x 16 = 64MHz */
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
		FLASH_SetLatency(FLASH_Latency_2);
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		RCC_PCLK2Config(RCC_HCLK_Div1);
		RCC_PCLK1Config(RCC_HCLK_Div2);
		RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_16);
		RCC_PLLCmd(ENABLE);
		t = 0; while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET && ++t < 0x200000);
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		while(RCC_GetSYSCLKSource() != 0x08);
	}
	SystemCoreClockUpdate();
}

/* TIM4 1ms 中断：对应 user/stm32f10x_it.c 里的实现 */
void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{
		time1_cntr++;
		time2_cntr++;
	}
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
}
