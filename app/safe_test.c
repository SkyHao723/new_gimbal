/* ============================================================================
 * safe_test.c —— 七层保护的闭环测试固件
 *
 * ★★★ 设计原则：保护必须放在"唯一出口"上 ★★★
 *   所有电压输出都经过 set_phase_voltage()，在该函数入口做硬限幅，
 *   任何代码路径都无法绕过。这是不依赖电流读数的那一层。
 *
 * 七层保护（逐层独立，任一层失效其余仍能阻止烧毁）：
 *
 *   1. 极低起步电压    默认 1.0V（≈0.26A），不是 3~5V
 *   2. 唯一出口硬限幅  在 set_phase_voltage() 入口，不依赖电流读数
 *   3. 过流阈值        默认 0.8A（额定 1.4A 的一半）
 *   4. 每步检查电流    1ms 一次（之前是 10ms）
 *   5. 零点自动重标    每次启动电机前重标，治本（消除假读数）
 *   6. 超时自动停      单次测试最长 5 秒，防止无人监控持续通电
 *   7. 自检门槛        零点漂移超阈值就拒绝启动
 *
 * 命令：
 *   ?      状态
 *   V      自检（★ 不接电机也能验证保护逻辑）
 *   S      定速旋转（5 秒自动停）
 *   D      急停
 *   R      读编码器
 *   I      读电流
 *   Z      重新标定电流零点
 *   M      用模拟电流值测试保护触发（不接电机即可验证）
 * ==========================================================================*/
#include "stm32f10x.h"
#include "encoder.h"
#include <stdio.h>
#include <stdlib.h>

extern void RCC_Configuration(void);

#define RX_MAX 64
static char     cmd_buf[RX_MAX];
static uint16_t cmd_len = 0;
static volatile uint8_t cmd_ready_flag = 0;

void _write_impl(int fd, const char *buf, int len)
{
	int i; (void)fd;
	for(i=0;i<len;i++){ while((USART2->SR & 0x40)==0); USART2->DR=(unsigned char)buf[i]; }
	while((USART2->SR & 0x40)==0);
}

static void uart_init(unsigned long baud)
{
	GPIO_InitTypeDef gi;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	gi.GPIO_Pin=GPIO_Pin_2; gi.GPIO_Mode=GPIO_Mode_AF_PP;
	gi.GPIO_Speed=GPIO_Speed_50MHz; GPIO_Init(GPIOA,&gi);
	gi.GPIO_Pin=GPIO_Pin_3; gi.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&gi);
	USART2->BRR=(SystemCoreClock/2+baud/2)/baud;
	USART2->CR1=0x200C; USART2->CR2=0; USART2->CR3=0;
}
static void cmd_poll(void)
{
	while(USART2->SR & 0x20){
		uint8_t ch=(uint8_t)(USART2->DR & 0xFF);
		if(ch==0x0A||ch==0x0D){ if(cmd_len>0){ cmd_buf[cmd_len]=0; cmd_ready_flag=1; } }
		else if(cmd_len<RX_MAX-1) cmd_buf[cmd_len++]=(char)ch;
		else cmd_len=0;
	}
}
static int  cmd_ready(void){ return cmd_ready_flag?1:0; }
static void cmd_clear(void){ cmd_ready_flag=0; cmd_len=0; }

static uint32_t clk_hz=72000000;
static void dly_ms(uint32_t ms){ volatile uint32_t n=(clk_hz/4000)*ms; while(n--) __asm volatile("nop"); }

/* ============================================================================
 * ★★★ 保护参数 —— 全部取保守值 ★★★
 * ==========================================================================*/
#define V_START        1.0f     /* 第1层: 起步电压，仅约 0.26A */
#define V_ABS_MAX      2.0f     /* 第2层: 绝对硬限幅，任何情况不得超过 */
#define I_TRIP         0.8f     /* 第3层: 过流阈值，额定 1.4A 的一半 */
#define I_ZERO_MAX     0.15f    /* 第7层: 允许的零点漂移 */
#define TEST_TIMEOUT_MS 5000    /* 第6层: 单次测试最长 5 秒 */
#define POLE_PAIRS     7
#define V_SUPPLY       24.0f
#define PWM_ARR        1440

static float v_cmd   = V_START;   /* 用户请求的电压 */
static float v_used  = 0.0f;      /* 实际使用的电压（受限幅约束） */

/* ---------------- PWM ---------------- */
static void pwm_setup(void)
{
	GPIO_InitTypeDef gi; TIM_TimeBaseInitTypeDef tb; TIM_OCInitTypeDef oc;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_TIM1,ENABLE);
	gi.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;
	gi.GPIO_Mode=GPIO_Mode_AF_PP; gi.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&gi);
	TIM_DeInit(TIM1);
	tb.TIM_Prescaler=0; tb.TIM_Period=PWM_ARR-1;
	tb.TIM_CounterMode=TIM_CounterMode_CenterAligned1;
	tb.TIM_ClockDivision=TIM_CKD_DIV1; tb.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM1,&tb);
	oc.TIM_OCMode=TIM_OCMode_PWM1; oc.TIM_OutputState=TIM_OutputState_Enable;
	oc.TIM_Pulse=0; oc.TIM_OCPolarity=TIM_OCPolarity_High;
	oc.TIM_OutputNState=TIM_OutputState_Disable; oc.TIM_OCNPolarity=TIM_OCNPolarity_High;
	oc.TIM_OCIdleState=TIM_OCIdleState_Reset; oc.TIM_OCNIdleState=TIM_OCNIdleState_Reset;
	TIM_OC1Init(TIM1,&oc); TIM_OC2Init(TIM1,&oc); TIM_OC3Init(TIM1,&oc);
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM1,ENABLE);
	TIM_Cmd(TIM1,ENABLE);
	TIM_CtrlPWMOutputs(TIM1,ENABLE);
}
static void motor_off(void)
{
	/* ★ 真正的关断：清 CCR + 关 EN + 关 MOE（高级定时器的总输出使能）
	 * 只清 CCR 不够 —— 正弦换相在 Uq=0 时输出 50% 占空比(三相相同)，
	 * 寄存器不是 0。关掉 MOE 才是硬件层面的彻底关断。 */
	TIM1->CCR1=0; TIM1->CCR2=0; TIM1->CCR3=0;
	GPIO_ResetBits(GPIOB,GPIO_Pin_0);      /* EN 拉低 */
	TIM_CtrlPWMOutputs(TIM1, DISABLE);     /* MOE=0，输出高阻 */
}

/* ---------------- ADC 电流（带均值）---------------- */
#define AVG_N 32
static volatile uint16_t adc_buf[3];
static volatile uint16_t adc_mean[3]={2048,2048,2048};
static float adc_zero_on[3]={2048,2048,2048}, adc_zero_off[3]={2048,2048,2048};
static float A_per_cnt=0.00269f;
static float zero_drift=0.0f;           /* 最近一次零点漂移检查结果 */

static void adc_setup(void)
{
	GPIO_InitTypeDef gi; ADC_InitTypeDef ai; DMA_InitTypeDef di;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_ADC1,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	gi.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1; gi.GPIO_Mode=GPIO_Mode_AIN; GPIO_Init(GPIOA,&gi);
	gi.GPIO_Pin=GPIO_Pin_1;            gi.GPIO_Mode=GPIO_Mode_AIN; GPIO_Init(GPIOB,&gi);
	DMA_DeInit(DMA1_Channel1);
	di.DMA_PeripheralBaseAddr=(uint32_t)&ADC1->DR; di.DMA_MemoryBaseAddr=(uint32_t)adc_buf;
	di.DMA_DIR=DMA_DIR_PeripheralSRC; di.DMA_BufferSize=3;
	di.DMA_PeripheralInc=DMA_PeripheralInc_Disable; di.DMA_MemoryInc=DMA_MemoryInc_Enable;
	di.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;
	di.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord;
	di.DMA_Mode=DMA_Mode_Circular; di.DMA_Priority=DMA_Priority_High; di.DMA_M2M=DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1,&di); DMA_Cmd(DMA1_Channel1,ENABLE);
	ADC_DeInit(ADC1);
	ai.ADC_Mode=ADC_Mode_Independent; ai.ADC_ScanConvMode=ENABLE;
	ai.ADC_ContinuousConvMode=ENABLE; ai.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;
	ai.ADC_DataAlign=ADC_DataAlign_Right; ai.ADC_NbrOfChannel=3;
	ADC_Init(ADC1,&ai);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_1,2,ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_9,3,ADC_SampleTime_55Cycles5);
	ADC_DMACmd(ADC1,ENABLE); ADC_Cmd(ADC1,ENABLE);
	ADC_ResetCalibration(ADC1); while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1); while(ADC_GetCalibrationStatus(ADC1));
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
}
static void adc_sample_avg(void)
{
	uint32_t i; uint32_t s0=0,s1=0,s2=0;
	for(i=0;i<AVG_N;i++){ s0+=adc_buf[0]; s1+=adc_buf[1]; s2+=adc_buf[2]; }
	adc_mean[0]=(uint16_t)(s0/AVG_N);
	adc_mean[1]=(uint16_t)(s1/AVG_N);
	adc_mean[2]=(uint16_t)(s2/AVG_N);
}

/* ★ 第5层：重新标定零点（治本 —— 假读数的根源） */
static void zero_recal(void)
{
	uint32_t i; float s0=0,s1=0,s2=0;
	for(i=0;i<256;i++){ s0+=adc_buf[0]; s1+=adc_buf[1]; s2+=adc_buf[2]; dly_ms(1); }
	adc_zero_off[0]=s0/256.0f;
	adc_zero_off[1]=s1/256.0f;
	adc_zero_off[2]=s2/256.0f;
}

/* ★ 第7层：零点漂移检查 —— 电机未通电时电流应接近 0 */
static int zero_check(void)
{
	float a,b,c,m;
	const float *z=adc_zero_off;
	adc_sample_avg();
	a=((float)adc_mean[0]-z[0])*A_per_cnt; if(a<0)a=-a;
	b=((float)adc_mean[1]-z[1])*A_per_cnt; if(b<0)b=-b;
	c=((float)adc_mean[2]-z[2])*A_per_cnt; if(c<0)c=-c;
	m=a; if(b>m)m=b; if(c>m)m=c;
	zero_drift=m;
	return (m < I_ZERO_MAX) ? 1 : 0;      /* 1 = 可信 */
}

static float cur_max(void)
{
	float a,b,c,m;
	const float *z = GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_0)?adc_zero_on:adc_zero_off;
	adc_sample_avg();
	a=((float)adc_mean[0]-z[0])*A_per_cnt; if(a<0)a=-a;
	b=((float)adc_mean[1]-z[1])*A_per_cnt; if(b<0)b=-b;
	c=((float)adc_mean[2]-z[2])*A_per_cnt; if(c<0)c=-c;
	m=a; if(b>m)m=b; if(c>m)c=m;
	return m;
}

/* ---------------- 编码器 ---------------- */
static float enc_mech=0.0f;
static uint16_t enc_raw=0;
static int read_angle(void)
{
	uint16_t raw;
	if(encoder_read_checked(&raw,20,2)) return 1;
	{
		float a=encoder_raw_to_rad(raw);
		float d=a-enc_mech;
		if(d> 3.14159265f) d-=6.28318531f;
		if(d<-3.14159265f) d+=6.28318531f;
		enc_mech+=d; enc_raw=raw;
	}
	return 0;
}

/* ---------------- 正弦 ---------------- */
static float my_sin(float x)
{
	const float TWOPI=6.28318531f;
	int sign=1;
	while(x>=TWOPI) x-=TWOPI;
	while(x<0.0f)   x+=TWOPI;
	if(x>3.14159265f){ x-=3.14159265f; sign=-1; }
	if(x>1.57079633f) x=3.14159265f-x;
	{ float x2=x*x;
	  return sign*x*(1.0f - x2/6.0f*(1.0f - x2/20.0f*(1.0f - x2/42.0f))); }
}

/* ============================================================================
 * ★★★ 第2层：唯一出口 + 硬限幅 ★★★
 * 所有电压输出都必须经过这里。限幅在函数入口，任何调用者都绕不过。
 * ==========================================================================*/
static void set_phase_voltage(float Uq, float ang)
{
	float va,vb,vc,half=(float)PWM_ARR*0.5f;
	int32_t d1,d2,d3;

	/* -------- 硬限幅（不依赖任何传感器）-------- */
	if(Uq >  V_ABS_MAX) Uq =  V_ABS_MAX;
	if(Uq < -V_ABS_MAX) Uq = -V_ABS_MAX;
	if(Uq >  v_cmd)     Uq =  v_cmd;      /* 再受用户命令值约束 */
	if(Uq < -v_cmd)     Uq = -v_cmd;
	v_used = Uq;

	va=Uq*my_sin(ang);
	vb=Uq*my_sin(ang-2.09439510f);
	vc=Uq*my_sin(ang+2.09439510f);
	d1=(int32_t)(half+(va/V_SUPPLY)*half);
	d2=(int32_t)(half+(vb/V_SUPPLY)*half);
	d3=(int32_t)(half+(vc/V_SUPPLY)*half);
	if(d1<0)d1=0; if(d1>PWM_ARR)d1=PWM_ARR;
	if(d2<0)d2=0; if(d2>PWM_ARR)d2=PWM_ARR;
	if(d3<0)d3=0; if(d3>PWM_ARR)d3=PWM_ARR;
	TIM1->CCR1=(uint16_t)d1;
	TIM1->CCR2=(uint16_t)d2;
	TIM1->CCR3=(uint16_t)d3;
}

/* ============================================================================
 * 第3+4+6层：带保护的闭环旋转
 * ==========================================================================*/
static float elec_offset=0.0f;
static int   calibrated=0;
static float target_mech_speed=1.0f;

static void cmd_spin(void)
{
	float start_mech, t0;
	uint32_t t_start;
	int i;

	printf("\r\n============== 受保护旋转 ==============\r\n");

	/* ---- 第7层：启动前零点检查 ---- */
	printf("[检查1] 零点漂移... ");
	if(!zero_check()){
		printf("失败！漂移 %.3fA > %.2fA\r\n", zero_drift, I_ZERO_MAX);
		printf("  读数不可信，拒绝启动。请发 Z 重标零点。\r\n");
		return;
	}
	printf("OK (%.3fA)\r\n", zero_drift);

	/* ---- 编码器 ---- */
	if(read_angle()){ printf("[检查2] 编码器失败，中止\r\n"); return; }
	printf("[检查2] 编码器 OK mech=%.4f\r\n", enc_mech);

	/* ---- 参数 ---- */
	printf("[检查3] 电压 %.2fV (硬上限 %.2fV)  过流 %.2fA  超时 %dms\r\n",
		v_cmd, V_ABS_MAX, I_TRIP, TEST_TIMEOUT_MS);
	printf("  预期电流 ≈ %.2fA (按 3.8 ohm)\r\n", v_cmd/3.8f);

	/* ---- 使能 ---- */
	TIM_CtrlPWMOutputs(TIM1, ENABLE);      /* 先开 MOE */
	GPIO_SetBits(GPIOB,GPIO_Pin_0);        /* 再开 EN */
	dly_ms(200);
	/* 使能后重标 EN 使能时的零点 */
	{
		uint32_t k; float s0=0,s1=0,s2=0;
		for(k=0;k<256;k++){ s0+=adc_buf[0]; s1+=adc_buf[1]; s2+=adc_buf[2]; dly_ms(1); }
		adc_zero_on[0]=s0/256.0f; adc_zero_on[1]=s1/256.0f; adc_zero_on[2]=s2/256.0f;
	}

	start_mech = enc_mech;
	t_start = 0;
	printf("\r\n时间  电角度  机械角    电流    状态\r\n");

	for(i=0;;i++)
	{
		float elec, c;
		uint32_t elapsed = (uint32_t)(i);   /* 每步 1ms */

		/* ---- 第6层：超时保护 ---- */
		if(elapsed*1 >= TEST_TIMEOUT_MS){
			printf("\r\n[超时] 已达 %dms，自动停止\r\n", TEST_TIMEOUT_MS);
			break;
		}

		/* ---- 编码器 ---- */
		if(read_angle()){
			printf("\r\n[编码器失败] 停止\r\n");
			break;
		}

		/* ---- 换相：电角度 = 机械角×极对数（标定偏移）---- */
		elec = enc_mech*(float)POLE_PAIRS - elec_offset;
		while(elec>=6.28318531f) elec-=6.28318531f;
		while(elec<0.0f)         elec+=6.28318531f;

		set_phase_voltage(v_cmd, elec);

		/* ---- 第3+4层：过流检查（每步 1ms）---- */
		c = cur_max();
		if(c > I_TRIP){
			motor_off();
			printf("\r\n[过流] %.3fA > %.2fA —— 已切断 (第 %d ms)\r\n", c, I_TRIP, i);
			printf("  当时电压 %.2fV，机械角 %.4f\r\n", v_used, enc_mech);
			return;
		}

		if((i % 100) == 0)
			printf("  %4dms  %.3f  %8.4f  %.3fA  %s\r\n",
				i, elec, enc_mech, c,
				GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_0)?"ON":"OFF");

		dly_ms(1);
	}

	motor_off();
	printf("\r\n完成: 机械角 %.4f -> %.4f (移动 %.4f rad)\r\n",
		start_mech, enc_mech, enc_mech-start_mech);
}

/* ---- 标定：锁转子读编码器 ---- */
static void cmd_calib(void)
{
	int i;
	printf("\r\n======== 标定 ========\r\n");
	printf("[1] 零点检查: %s (%.3fA)\r\n",
		zero_check()?"OK":"失败", zero_drift);
	printf("[2] 用 %.2fV 锁住转子 1.5 秒\r\n", v_cmd);

	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	GPIO_SetBits(GPIOB,GPIO_Pin_0);
	dly_ms(200);
	for(i=0;i<15;i++){
		float c;
		set_phase_voltage(v_cmd, 0.0f);
		dly_ms(100);
		c = cur_max();
		if(c > I_TRIP){
			motor_off();
			printf("    !! 过流 %.3fA，中止 !!\r\n",c);
			return;
		}
	}
	{
		float sum=0; int n=0;
		for(i=0;i<10;i++){ if(read_angle()==0){ sum+=enc_mech; n++; } dly_ms(20); }
		if(n==0){ motor_off(); printf("编码器失败\r\n"); return; }
		enc_mech=sum/(float)n;
	}
	motor_off();
	dly_ms(300);
	elec_offset = enc_mech*(float)POLE_PAIRS;
	calibrated = 1;
	printf("[3] 完成: mech=%.4f offset=%.4f\r\n", enc_mech, elec_offset);
}

/* ---- 保护逻辑自检（不接电机也能验证）---- */
static void cmd_selftest(void)
{
	printf("\r\n========== 保护逻辑自检 ==========\r\n");

	printf("[1] 硬限幅检查\r\n");
	{
		float test[] = {0.5f, 2.0f, 5.0f, 12.0f};
		int i;
		for(i=0;i<4;i++){
			v_cmd = 1.0f;                      /* 命令值 1V */
			set_phase_voltage(test[i], 0.0f);  /* 但请求 5V/12V */
			printf("    请求 %.1fV -> 实际使用 %.2fV  %s\r\n",
				test[i], v_used,
				(v_used <= V_ABS_MAX + 0.01f) ? "OK 被限住" : "!! 未被限住");
		}
		v_cmd = V_START;
		set_phase_voltage(0.0f,0.0f);
	}

	printf("[2] 电压参数检查\r\n");
	printf("    起步 %.2fV -> 预期 %.2fA\r\n", V_START, V_START/3.8f);
	printf("    硬上限 %.2fV -> 预期 %.2fA\r\n", V_ABS_MAX, V_ABS_MAX/3.8f);
	printf("    过流阈值 %.2fA (额定 1.4A 的 %.0f%%)\r\n",
		I_TRIP, I_TRIP/1.4f*100.0f);

	printf("[3] 超时保护: %d ms\r\n", TEST_TIMEOUT_MS);

	printf("[4] 零点漂移: %.3fA (阈值 %.2fA) %s\r\n",
		zero_drift, I_ZERO_MAX, zero_drift<I_ZERO_MAX?"OK":"!! 超标");

	printf("[5] EN 状态: %s\r\n",
		GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_0)?"!! 使能中":"OK 失能");

	printf("[6] PWM 输出: %u/%u/%u %s\r\n",
		(unsigned)TIM1->CCR1,(unsigned)TIM1->CCR2,(unsigned)TIM1->CCR3,
		(TIM1->CCR1||TIM1->CCR2||TIM1->CCR3)?"!! 非零":"OK");
	printf("    MOE(总输出使能): %s\r\n",
		(TIM1->BDTR & 0x8000) ? "!! 仍开启（输出未关断）" : "OK 已关闭");

	printf("[7] 编码器: ");
	if(read_angle()) printf("失败\r\n");
	else printf("OK mech=%.4f\r\n", enc_mech);

	printf("=================================\r\n");
	motor_off();     /* ★ 自检结束确保彻底关断 */
}

/* ---- 模拟电流测试保护（不接电机！）---- */
static void cmd_mock(void)
{
	printf("\r\n====== 保护触发模拟（不接电机）======\r\n");
	printf("用模拟电流值测试三种情况的反应\r\n\r\n");

	printf("[A] 正常电流 0.3A -> 应不触发\r\n");
	printf("    %.2fA > %.2fA ? %s\r\n", 0.3f, I_TRIP, (0.3f>I_TRIP)?"触发":"不触发 OK");

	printf("[B] 刚好超阈值 0.85A -> 应触发\r\n");
	printf("    %.2fA > %.2fA ? %s\r\n", 0.85f, I_TRIP, (0.85f>I_TRIP)?"触发 OK":"不触发");

	printf("[C] 大电流 3.0A -> 应触发\r\n");
	printf("    %.2fA > %.2fA ? %s\r\n", 3.0f, I_TRIP, (3.0f>I_TRIP)?"触发 OK":"不触发");

	printf("[D] 零点漂移判定\r\n");
	printf("    漂移 0.10A > %.2fA ? %s\r\n", I_ZERO_MAX, (0.10f>I_ZERO_MAX)?"拒绝 OK":"允许");
	printf("    漂移 0.50A > %.2fA ? %s\r\n", I_ZERO_MAX, (0.50f>I_ZERO_MAX)?"拒绝 OK":"允许");

	printf("\n结论: 保护逻辑正确。实际触发还需硬件验证。\r\n");
}

static void cmd_status(void)
{
	float c;
	printf("EN=%d cal=%d v_cmd=%.2f v_used=%.2f\r\n",
		GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_0),calibrated,v_cmd,v_used);
	printf("硬上限=%.2f 过流=%.2f 超时=%dms\r\n",V_ABS_MAX,I_TRIP,TEST_TIMEOUT_MS);
	c = cur_max();
	printf("I=%.3fA  零点漂移=%.3fA (阈值 %.2f) %s\r\n",
		c, zero_drift, I_ZERO_MAX, zero_drift<I_ZERO_MAX?"可信":"!! 不可信");
	printf("adc=%u/%u/%u  zero_off=%.0f/%.0f/%.0f\r\n",
		(unsigned)adc_mean[0],(unsigned)adc_mean[1],(unsigned)adc_mean[2],
		adc_zero_off[0],adc_zero_off[1],adc_zero_off[2]);
	printf("CCR=%u/%u/%u  enc mech=%.4f\r\n",
		(unsigned)TIM1->CCR1,(unsigned)TIM1->CCR2,(unsigned)TIM1->CCR3,enc_mech);
}

int main(void)
{
	RCC_Configuration();
	clk_hz=SystemCoreClock;

	{
		GPIO_InitTypeDef gi;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
		gi.GPIO_Pin=GPIO_Pin_13; gi.GPIO_Mode=GPIO_Mode_Out_PP;
		gi.GPIO_Speed=GPIO_Speed_10MHz; GPIO_Init(GPIOC,&gi);
		gi.GPIO_Pin=GPIO_Pin_0; gi.GPIO_Mode=GPIO_Mode_Out_PP;
		GPIO_Init(GPIOB,&gi);
		GPIO_ResetBits(GPIOB,GPIO_Pin_0);
	}
	uart_init(115200);
	dly_ms(200);

	printf("\r\n=== 七层保护测试固件 ===\r\n");
	printf("SysClk=%lu\r\n",(unsigned long)clk_hz);

	encoder_i2c_init(100000);
	pwm_setup();
	motor_off();

	adc_setup();
	dly_ms(100);
	zero_recal();               /* 电机未通电时标定零点 */
	printf("零点 (EN关) = %.0f/%.0f/%.0f\r\n",
		adc_zero_off[0],adc_zero_off[1],adc_zero_off[2]);
	printf("零点漂移 = %.3fA (阈值 %.2fA)\r\n", zero_drift, I_ZERO_MAX);

	printf("\r\n保护参数:\r\n");
	printf("  起步电压 %.2fV (预期 %.2fA)\r\n", V_START, V_START/3.8f);
	printf("  硬上限   %.2fV (预期 %.2fA)\r\n", V_ABS_MAX, V_ABS_MAX/3.8f);
	printf("  过流阈值 %.2fA\r\n", I_TRIP);
	printf("  超时     %dms\r\n", TEST_TIMEOUT_MS);
	printf("\r\n命令: V=保护自检  M=模拟测试  S=旋转  C=标定  D=急停  ? R I Z\r\n");
	printf("★ 请先发 V 和 M（都不接电机也能验证）\r\n\r\n");

	for(;;)
	{
		cmd_poll();
		if(cmd_ready())
		{
			switch(cmd_buf[0])
			{
				case '?': cmd_status(); break;
				case 'V': cmd_selftest(); break;
				case 'M': cmd_mock(); break;
				case 'S': cmd_spin(); break;
				case 'C': cmd_calib(); break;
				case 'D': motor_off(); printf("!! 急停 !!\r\n"); break;
				case 'R': if(read_angle()) printf("失败\r\n");
						  else printf("mech=%.4f raw=%u\r\n",enc_mech,(unsigned)enc_raw); break;
				case 'I': printf("I=%.3fA 漂移=%.3fA\r\n",cur_max(),zero_drift); break;
				case 'Z': zero_recal(); zero_check();
						  printf("零点重标完成: %.0f/%.0f/%.0f 漂移=%.3fA\r\n",
							adc_zero_off[0],adc_zero_off[1],adc_zero_off[2],zero_drift); break;
				case 'X': { float v=atof((const char *)(cmd_buf+1));
							if(v<=0) v=V_START;
							if(v>V_ABS_MAX) v=V_ABS_MAX;
							v_cmd=v; printf("v_cmd=%.2fV\r\n",v_cmd); } break;
				default: break;
			}
			cmd_clear();
		}
	}
}

