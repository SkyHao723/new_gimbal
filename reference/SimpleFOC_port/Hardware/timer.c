
/* ============================================================================
 * timer.c —— 蓝板适配版（TIM1 方案）
 *   TIM1_CH1/CH2/CH3 -> PA8/PA9/PA10 : 三路中心对齐 PWM, 25kHz
 *   TIM4             -> 1ms 时基中断（LED 计时、状态计时）
 *
 * ★★ TIM1 是高级定时器，除了 TIM_Cmd() 之外还必须调用
 *      TIM_CtrlPWMOutputs(TIM1, ENABLE);
 *    打开主输出(MOE)，否则寄存器全配好了，PA8/PA9/PA10 上一点波形都没有。
 *
 * 注意：PA9/PA10 被 PWM 占用后，串口必须改用 USART2(PA2/PA3)，
 *       对应 user/MyProject.h 里 USE_USART1 = 0
 * ==========================================================================*/

#include "stm32f10x.h"
#include "timer.h"

/***************************************************************************/
void TIM2_PWM_Init(void)      /* 函数名沿用原工程，内部已改为 TIM1 */
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef   TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef         TIM_OCInitStructure;
	
	/* TIM1 挂在 APB2 上 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_TIM1,ENABLE);
	
	/* PA8=PWM1, PA9=PWM2, PA10=PWM3 —— 接驱动小板的三路 PWM 输入 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
	TIM_DeInit(TIM1);
	TIM_TimeBaseInitStructure.TIM_Prescaler=0;
	TIM_TimeBaseInitStructure.TIM_Period=PWM_Period-1;      //25KHz @72MHz (中心对齐)
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_CenterAligned1;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;      //高级定时器专用，必须赋值
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1; 
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse=0;                        //上电占空比 0，电机不转
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputNState=TIM_OutputState_Disable;   //不用互补输出
	TIM_OCInitStructure.TIM_OCNPolarity=TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState=TIM_OCNIdleState_Reset;
	TIM_OC1Init(TIM1,&TIM_OCInitStructure);
	TIM_OC2Init(TIM1,&TIM_OCInitStructure);
	TIM_OC3Init(TIM1,&TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);
	
	/* 注意：没有用互补/刹车，先把自动输出使能关掉，避免异常时出错 */
	TIM_ARRPreloadConfig(TIM1,ENABLE);
	TIM_Cmd(TIM1,ENABLE);
	
	/* ★ 高级定时器必须开主输出，否则 PA8/PA9/PA10 无波形 */
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}
/***************************************************************************/
void TIM4_1ms_Init(void)  //(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure); 
	
	TIM_TimeBaseInitStructure.TIM_Period = 1000-1;      //1ms
	TIM_TimeBaseInitStructure.TIM_Prescaler=72-1;       //72分频=1MHz
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM4,ENABLE);
}
/***************************************************************************/
