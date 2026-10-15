/* ============================================================================
 * MINI_TEST —— 完全不依赖 SimpleFOC 库的最小测试固件
 *
 * 为什么重写：之前几个版本混用了 SimpleFOC 库（BLDCMotor/FOCMotor/...），
 * 全局变量和状态机互相干扰，反复出现卡死 / NaN / CCR 不更新。
 * 这个版本只依赖 SPL，逻辑简单到可以逐行检查。
 *
 * 引脚： PWM PA8/PA9/PA10(TIM1) | EN PB0 | 串口 PA2/PA3 | CS PA0/PA1/PB1
 *
 * 命令（ASCII，\r\n 结尾）：
 *   ?        状态
 *   E / D    使能 / 失能
 *   V<x>     电压上限 (V)
 *   L<x>     过流阈值 (A)
 *   C0/C1    过流保护 关/开
 *   S<x>     六步换相速度（每步 ms，默认 20）
 *   R        开始六步换相旋转（带电流保护，到时间自动停）
 *   Z        零电流测试（固定磁场，测电流）
 *   I        读电流
 *   K        清电流零点
 * ==========================================================================*/
#include "stm32f10x.h"
#include <stdio.h>
#include <stdlib.h>

extern void RCC_Configuration(void);

/* ---------------- 自包含串口（不依赖工程里的 usart.c）---------------- */
#define RX_BUF_LEN 128
static volatile unsigned char rx_buf[RX_BUF_LEN];
static volatile unsigned short rx_sta = 0;      /* bit15=完成 bit14=收到CR */

void _write_impl(int fd, const char *buf, int len)
{
	int i; (void)fd;
	for(i=0;i<len;i++){
		while((USART2->SR & 0x40) == 0);
		USART2->DR = (unsigned char)buf[i];
	}
	while((USART2->SR & 0x40) == 0);
}

static void uart_init(unsigned long baud)
{
	GPIO_InitTypeDef gi;
	NVIC_InitTypeDef ni;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	gi.GPIO_Pin = GPIO_Pin_2; gi.GPIO_Mode = GPIO_Mode_AF_PP;
	gi.GPIO_Speed = GPIO_Speed_50MHz; GPIO_Init(GPIOA, &gi);
	gi.GPIO_Pin = GPIO_Pin_3; gi.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &gi);
	/* PCLK1 = SysClk/2 */
	USART2->BRR = (SystemCoreClock/2 + baud/2) / baud;
	USART2->CR1 = 0x200C;                      /* UE|TE|RE */
	ni.NVIC_IRQChannel = USART2_IRQn;
	ni.NVIC_IRQChannelPreemptionPriority = 3;
	ni.NVIC_IRQChannelSubPriority = 3;
	ni.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&ni);
	USART2->CR1 |= 0x0020;                     /* RXNEIE */
}

void USART2_IRQHandler(void)
{
	if(USART2->SR & 0x20)
	{
		unsigned char r = (unsigned char)(USART2->DR & 0xFF);
		if((rx_sta & 0x8000) == 0)
		{
			if(rx_sta & 0x4000){
				if(r == 0x0A){ rx_sta |= 0x8000; rx_buf[rx_sta & 0x3FFF] = 0; }
				else rx_sta = 0;
			} else {
				if(r == 0x0D) rx_sta |= 0x4000;
				else {
					rx_buf[rx_sta & 0x3FFF] = r;
					rx_sta++;
					if(rx_sta > RX_BUF_LEN-1) rx_sta = 0;
				}
			}
		}
	}
}

/* ---------------- 基础 ---------------- */
#define PWM_ARR 1440
static uint32_t clk_hz = 72000000;

static void dly_ms(uint32_t ms)
{
	volatile uint32_t n = (clk_hz / 4000) * ms;
	while(n--) __asm volatile("nop");
}

/* ---------------- 引脚 ---------------- */
static void gpio_init(void)
{
	GPIO_InitTypeDef gi;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|
	                       RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	gi.GPIO_Pin = GPIO_Pin_13; gi.GPIO_Mode = GPIO_Mode_Out_PP;
	gi.GPIO_Speed = GPIO_Speed_10MHz; GPIO_Init(GPIOC, &gi);
	gi.GPIO_Pin = GPIO_Pin_0;  gi.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &gi);
	GPIO_ResetBits(GPIOB, GPIO_Pin_0);
}

/* ---------------- PWM ---------------- */
static void pwm_init(void)
{
	GPIO_InitTypeDef gi;
	TIM_TimeBaseInitTypeDef tb;
	TIM_OCInitTypeDef oc;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_TIM1, ENABLE);
	gi.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;
	gi.GPIO_Mode = GPIO_Mode_AF_PP; gi.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gi);

	TIM_DeInit(TIM1);
	tb.TIM_Prescaler = 0;
	tb.TIM_Period = PWM_ARR - 1;
	tb.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	tb.TIM_ClockDivision = TIM_CKD_DIV1;
	tb.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &tb);

	oc.TIM_OCMode = TIM_OCMode_PWM1;
	oc.TIM_OutputState = TIM_OutputState_Enable;
	oc.TIM_Pulse = 0;
	oc.TIM_OCPolarity = TIM_OCPolarity_High;
	oc.TIM_OutputNState = TIM_OutputState_Disable;
	oc.TIM_OCNPolarity = TIM_OCNPolarity_High;
	oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
	oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	TIM_OC1Init(TIM1,&oc); TIM_OC2Init(TIM1,&oc); TIM_OC3Init(TIM1,&oc);
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM1, ENABLE);
	TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

static void duty(uint16_t a, uint16_t b, uint16_t c)
{
	TIM1->CCR1 = a; TIM1->CCR2 = b; TIM1->CCR3 = c;
}
static void duty_off(void) { duty(0,0,0); }

/* ---------------- ADC ---------------- */
static volatile uint16_t adc_raw[3];
static float adc_zero[3] = {2048,2048,2048};
static float A_per_cnt = 0.00269f;
static float I_max = 1.5f;
static uint8_t prot_on = 0;

static void adc_init(void)
{
	GPIO_InitTypeDef gi;
	ADC_InitTypeDef ai;
	DMA_InitTypeDef di;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_ADC1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	gi.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1; gi.GPIO_Mode = GPIO_Mode_AIN; GPIO_Init(GPIOA, &gi);
	gi.GPIO_Pin = GPIO_Pin_1;            gi.GPIO_Mode = GPIO_Mode_AIN; GPIO_Init(GPIOB, &gi);

	DMA_DeInit(DMA1_Channel1);
	di.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	di.DMA_MemoryBaseAddr = (uint32_t)adc_raw;
	di.DMA_DIR = DMA_DIR_PeripheralSRC;
	di.DMA_BufferSize = 3;
	di.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	di.DMA_MemoryInc = DMA_MemoryInc_Enable;
	di.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	di.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	di.DMA_Mode = DMA_Mode_Circular;
	di.DMA_Priority = DMA_Priority_High;
	di.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &di);
	DMA_Cmd(DMA1_Channel1, ENABLE);

	ADC_DeInit(ADC1);
	ai.ADC_Mode = ADC_Mode_Independent;
	ai.ADC_ScanConvMode = ENABLE;
	ai.ADC_ContinuousConvMode = ENABLE;
	ai.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ai.ADC_DataAlign = ADC_DataAlign_Right;
	ai.ADC_NbrOfChannel = 3;
	ADC_Init(ADC1, &ai);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 3, ADC_SampleTime_55Cycles5);
	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);
	ADC_ResetCalibration(ADC1); while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1); while(ADC_GetCalibrationStatus(ADC1));
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

static void adc_zero_cal(void)
{
	uint32_t i; float s0=0,s1=0,s2=0;
	for(i=0;i<256;i++){ s0+=adc_raw[0]; s1+=adc_raw[1]; s2+=adc_raw[2]; dly_ms(1); }
	adc_zero[0]=s0/256.0f; adc_zero[1]=s1/256.0f; adc_zero[2]=s2/256.0f;
}

static float cur_max(void)
{
	float a,b,c,m;
	a = ((float)adc_raw[0]-adc_zero[0])*A_per_cnt; if(a<0)a=-a;
	b = ((float)adc_raw[1]-adc_zero[1])*A_per_cnt; if(b<0)b=-b;
	c = ((float)adc_raw[2]-adc_zero[2])*A_per_cnt; if(c<0)c=-c;
	m = a; if(b>m)m=b; if(c>m)m=c;
	return m;
}

static uint8_t overcurrent(void)
{
	if(!prot_on) return 0;
	if(cur_max() > I_max)
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_0);
		duty_off();
		prot_on = 0;
		printf("\r\n!! OVERCURRENT > %.2fA -- EN OFF !!\r\n", I_max);
		return 1;
	}
	return 0;
}

/* ---------------- 六步换相 ----------------
 * 每步：一相接正、一相接负、一相悬空（占空比 0）
 * 电压用占空比控制：duty = V/supply * 50% 偏置
 * 简单起见直接用百分比 */
static void six_step(int step, uint16_t d)
{
	switch(step % 6)
	{
		case 0: duty(d, 0, 0); break;
		case 1: duty(d, d, 0); break;
		case 2: duty(0, d, 0); break;
		case 3: duty(0, d, d); break;
		case 4: duty(0, 0, d); break;
		default:duty(d, 0, d); break;
	}
}

static float v_supply = 24.0f;
static float v_limit  = 2.0f;
static uint16_t ms_per_step = 20;

static uint16_t duty_from_volts(float v)
{
	float pct = v / v_supply;          /* 目标相电压/母线 */
	if(pct > 0.95f) pct = 0.95f;
	if(pct < 0.0f)  pct = 0.0f;
	return (uint16_t)(pct * PWM_ARR);
}

static void cmd_rotate(void)
{
	int i;
	uint16_t d;
	if(v_limit > v_supply*0.5f) v_limit = v_supply*0.5f;
	d = duty_from_volts(v_limit);
	printf("ROTATE: V=%.2fV duty=%u, %u ms/step, %u steps\r\n",
		v_limit, (unsigned)d, (unsigned)ms_per_step, (unsigned)(ms_per_step*0 + 200));
	GPIO_SetBits(GPIOB, GPIO_Pin_0);
	for(i = 0; i < 200; i++)
	{
		six_step(i, d);
		dly_ms(ms_per_step);
		if((i % 5) == 0 && overcurrent()) break;
		if((i % 50) == 0)
			printf("  step=%3d I=%.3fA CCR %u/%u/%u\r\n", i, cur_max(),
				(unsigned)TIM1->CCR1,(unsigned)TIM1->CCR2,(unsigned)TIM1->CCR3);
	}
	duty_off();
	printf("ROTATE done\r\n");
}

static void cmd_zero(void)
{
	float v;
	GPIO_SetBits(GPIOB, GPIO_Pin_0);
	printf("ZERO: 固定磁场, 电压从 0.5V 起\r\n");
	for(v = 0.5f; v <= v_limit; v += 0.5f)
	{
		duty(duty_from_volts(v), duty_from_volts(v), 0);   /* 固定两相加压 */
		dly_ms(600);
		printf("  V=%.1fV  I=%.3fA  CCR %u/%u/%u\r\n", v, cur_max(),
			(unsigned)TIM1->CCR1,(unsigned)TIM1->CCR2,(unsigned)TIM1->CCR3);
		if(overcurrent()) break;
	}
	duty_off();
	printf("ZERO done\r\n");
}

int main(void)
{
	RCC_Configuration();
	clk_hz = SystemCoreClock;

	gpio_init();
	uart_init(115200);
	dly_ms(200);

	printf("\r\n=== MINI TEST ===\r\n");
	printf("SysClk=%lu Hz\r\n", (unsigned long)clk_hz);

	GPIO_SetBits(GPIOB, GPIO_Pin_0);   /* EN on, 便于 CSA 工作 */
	dly_ms(300);

	adc_init();
	dly_ms(100);
	adc_zero_cal();

	printf("adc raw %u/%u/%u  zero %.0f/%.0f/%.0f\r\n",
		(unsigned)adc_raw[0],(unsigned)adc_raw[1],(unsigned)adc_raw[2],
		adc_zero[0],adc_zero[1],adc_zero[2]);

	pwm_init();
	duty_off();

	printf("TIM1 BDTR=%08lX CCER=%08lX ARR=%lu\r\n",
		(unsigned long)TIM1->BDTR,(unsigned long)TIM1->CCER,(unsigned long)TIM1->ARR);
	printf("cmd: ? E D V<x> L<x> C0/C1 S<ms> R Z I K\r\n");

	for(;;)
	{
		if((rx_sta & 0x8000) != 0)
		{
			switch(rx_buf[0])
			{
				case '?':
					printf("EN=%d prot=%d Vlim=%.2f Imax=%.2f\r\n",
						GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_0),
						prot_on, v_limit, I_max);
					printf("adc %u/%u/%u zero %.0f/%.0f/%.0f I=%.3fA\r\n",
						(unsigned)adc_raw[0],(unsigned)adc_raw[1],(unsigned)adc_raw[2],
						adc_zero[0],adc_zero[1],adc_zero[2], cur_max());
					printf("CCR %u/%u/%u ARR=%u step=%ums\r\n",
						(unsigned)TIM1->CCR1,(unsigned)TIM1->CCR2,(unsigned)TIM1->CCR3,
						(unsigned)TIM1->ARR,(unsigned)ms_per_step);
					break;
				case 'E': GPIO_SetBits(GPIOB,GPIO_Pin_0);   printf("EN ON\r\n");  break;
				case 'D': GPIO_ResetBits(GPIOB,GPIO_Pin_0); duty_off(); printf("EN OFF\r\n"); break;
				case 'V': v_limit = atof((const char *)(rx_buf+1));
						  printf("Vlim=%.2fV\r\n", v_limit); break;
				case 'L': I_max = atof((const char *)(rx_buf+1));
						  printf("Imax=%.2fA\r\n", I_max); break;
				case 'J': A_per_cnt = atof((const char *)(rx_buf+1));
						  printf("A/cnt=%.5f\r\n", A_per_cnt); break;
				case 'C': prot_on = (rx_buf[1]=='1')?1:0;
						  printf("prot %s\r\n", prot_on?"ON":"OFF"); break;
				case 'S': ms_per_step = (uint16_t)atoi((const char *)(rx_buf+1));
						  if(ms_per_step < 2) ms_per_step = 2;
						  printf("step=%ums\r\n",(unsigned)ms_per_step); break;
				case 'I': printf("I=%.3fA adc %u/%u/%u\r\n", cur_max(),
						  (unsigned)adc_raw[0],(unsigned)adc_raw[1],(unsigned)adc_raw[2]); break;
				case 'K': adc_zero_cal();
						  printf("zero %.0f/%.0f/%.0f\r\n",adc_zero[0],adc_zero[1],adc_zero[2]); break;
				case 'R': cmd_rotate(); break;
				case 'Z': cmd_zero();   break;
				default: break;
			}
			rx_sta = 0;
		}
	}
}

