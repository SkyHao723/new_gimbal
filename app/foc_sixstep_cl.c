/* ============================================================================
 * foc_sixstep_cl.c —— 用六步换相实现的闭环（基于已验证能转的方案）
 *
 * 为什么用六步换相而不是正弦 FOC：
 *   实测六步换相能让电机稳定转动（角度持续变化、电流 0.15A），
 *   而三相正弦 FOC 在当前硬件上转子不跟随（开环推进电角度 3.6rad，
 *   机械角几乎不变）。六步换相是已被证明可行的路径，先用它把闭环跑通。
 *
 * 闭环原理（六步换相 + 绝对编码器）：
 *   1. 标定：给固定电角度锁转子 -> 读编码器 -> 算偏移
 *   2. 闭环：由编码器算电角度 -> 查表决定 6 个换相状态之一
 *           -> 电角度每变化 60° 就换一次相
 *   这样换相完全由转子实际位置决定，是真正的闭环。
 *
 * 命令：
 *   X 自检   C 标定   R 闭环旋转   D 急停   A 读角度   I 读电流
 *   V<x> 限压  L<x> 过流  S<x> 目标速度
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
	USART2->CR1=0x200C; USART2->CR2=0; USART2->CR3=0;   /* 轮询，无中断 */
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

/* ---------------- PWM ---------------- */
#define PWM_ARR 1440
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
static void pwm_off(void){ TIM1->CCR1=0; TIM1->CCR2=0; TIM1->CCR3=0; }

/* ---------------- ADC ---------------- */
static volatile uint16_t adc_buf[3];
static float adc_zero_on[3]={2048,2048,2048}, adc_zero_off[3]={2048,2048,2048};
static float A_per_cnt=0.00269f;

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
static void adc_cal_into(float *d)
{
	uint32_t i; float s0=0,s1=0,s2=0;
	for(i=0;i<256;i++){ s0+=adc_buf[0]; s1+=adc_buf[1]; s2+=adc_buf[2]; dly_ms(1); }
	d[0]=s0/256.0f; d[1]=s1/256.0f; d[2]=s2/256.0f;
}
static void adc_cal_both(void)
{
	GPIO_SetBits(GPIOB,GPIO_Pin_0);  dly_ms(300); adc_cal_into(adc_zero_on);
	GPIO_ResetBits(GPIOB,GPIO_Pin_0); dly_ms(300); adc_cal_into(adc_zero_off);
}
static float cur_max(void)
{
	float a,b,c,m;
	const float *z = GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_0)?adc_zero_on:adc_zero_off;
	a=((float)adc_buf[0]-z[0])*A_per_cnt; if(a<0)a=-a;
	b=((float)adc_buf[1]-z[1])*A_per_cnt; if(b<0)b=-b;
	c=((float)adc_buf[2]-z[2])*A_per_cnt; if(c<0)c=-c;
	m=a; if(b>m)m=b; if(c>m)m=c;
	return m;
}

/* ---------------- 状态 ---------------- */
static float v_limit=3.0f;
static float i_limit=1.5f;
static const float V_SUPPLY=24.0f;
static int   pole_pairs=7;

static float enc_mech=0.0f;
static uint16_t enc_raw=0;
static float elec_offset=0.0f;
static int   sensor_dir=1;
static int   calibrated=0;
static float target_speed=2.0f;

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

/* 六步换相：st = 0..5
 * 每个状态给两相加压（一相正、一相负），第三相悬空
 * 用 50% 占空比做基准，正相加、负相减，幅度由电压决定 */
static void six_step(int st, float volts)
{
	int32_t d = (int32_t)((volts / V_SUPPLY) * (float)PWM_ARR);
	int32_t hi = (PWM_ARR/2) + d;
	int32_t lo = (PWM_ARR/2) - d;
	if(hi > PWM_ARR) hi = PWM_ARR;
	if(hi < 0) hi = 0;
	if(lo > PWM_ARR) lo = PWM_ARR;
	if(lo < 0) lo = 0;

	switch(st & 7)
	{
		case 0: TIM1->CCR1=(uint16_t)hi; TIM1->CCR2=(uint16_t)lo; TIM1->CCR3=PWM_ARR/2; break;
		case 1: TIM1->CCR1=(uint16_t)hi; TIM1->CCR2=PWM_ARR/2;    TIM1->CCR3=(uint16_t)lo; break;
		case 2: TIM1->CCR1=PWM_ARR/2;    TIM1->CCR2=(uint16_t)hi; TIM1->CCR3=(uint16_t)lo; break;
		case 3: TIM1->CCR1=(uint16_t)lo; TIM1->CCR2=(uint16_t)hi; TIM1->CCR3=PWM_ARR/2; break;
		case 4: TIM1->CCR1=(uint16_t)lo; TIM1->CCR2=PWM_ARR/2;    TIM1->CCR3=(uint16_t)hi; break;
		default:TIM1->CCR1=PWM_ARR/2;    TIM1->CCR2=(uint16_t)lo; TIM1->CCR3=(uint16_t)hi; break;
	}
}

/* 由编码器算电角度 */
static float elec_from_encoder(void)
{
	float e=(enc_mech*(float)pole_pairs - elec_offset)*(float)sensor_dir;
	while(e>=6.28318531f) e-=6.28318531f;
	while(e<0.0f)         e+=6.28318531f;
	return e;
}

/* ---------------- 标定 ---------------- */
static void cmd_calibrate(void)
{
	int i;
	float lock_elec=0.0f;
	int lock_st = 0;              /* 固定一个换相状态锁转子 */

	printf("\r\n===== 标定 =====\r\n");
	printf("步骤1: 固定换相状态锁转子 (V=%.2fV)\r\n", v_limit);
	if(read_angle()){ printf("!! 编码器失败\r\n"); return; }

	/* ★ 官方做法：把转子锁在电角度 270°，而不是 0°
	 * 原因：闭环第一次换相时  T ∝ sin(θ_field - θ_rotor)
	 *   锁在 0°   -> T ∝ sin(0)   = 0  死点！给多大电压都不动
	 *   锁在 270° -> T ∝ sin(270°) = -1 满力矩，立刻起转 */
	lock_st = 4;                       /* 六步的第 4 个状态对应 270° 附近 */

	GPIO_SetBits(GPIOB,GPIO_Pin_0);
	for(i=0;i<80;i++){                 /* 80*20ms = 1.6s */
		six_step(lock_st, v_limit);
		dly_ms(20);
		if(cur_max()>i_limit){
			GPIO_ResetBits(GPIOB,GPIO_Pin_0); pwm_off();
			printf("!! 过流 %.3fA\r\n",cur_max()); return;
		}
	}
	{
		float sum=0; int n=0;
		for(i=0;i<10;i++){ if(read_angle()==0){ sum+=enc_mech; n++; } dly_ms(20); }
		if(n==0){ GPIO_ResetBits(GPIOB,GPIO_Pin_0); pwm_off();
			printf("!! 编码器失败\r\n"); return; }
		enc_mech=sum/(float)n;
	}
	pwm_off(); GPIO_ResetBits(GPIOB,GPIO_Pin_0); dly_ms(300);

	/* 状态 0 的电角度是 0（a相正、b相负），所以偏移 = mech*pp */
	lock_elec = 0.0f;
	elec_offset = enc_mech*(float)pole_pairs - lock_elec;
	sensor_dir = 1;
	calibrated = 1;

	printf("步骤2: 锁住后 mech=%.4f raw=%u\r\n", enc_mech,(unsigned)enc_raw);
	printf("步骤3: offset=%.4f pp=%d\r\n", elec_offset, pole_pairs);
	printf("完成，可发 R\r\n");
}

/* ---------------- 闭环旋转 ---------------- */
static void cmd_closedloop(void)
{
	int i, st, last_st = -999;
	int enc_fail = 0;
	int N = 1200;                       /* 1200 * 15ms ≈ 18s */
	float volts = v_limit;

	if(!calibrated){ printf("!! 请先发 C\r\n"); return; }
	if(read_angle()){ printf("!! 编码器失败\r\n"); return; }

	printf("\r\n===== 六步闭环 =====\r\n");
	printf("V=%.2fV Ilim=%.2fA pp=%d offset=%.4f\r\n",
		v_limit,i_limit,pole_pairs,elec_offset);
	printf("★ 起动策略: 先加 90 度电角度偏离（sin=1 力矩最大），\r\n");
	printf("   避免标定后 elec=0 时 sin(0)=0 产生【死点】\r\n");

	GPIO_SetBits(GPIOB,GPIO_Pin_0);

	for(i=0;i<N;i++)
	{
		float e, c;
		if(read_angle()) enc_fail++; else enc_fail=0;
		if(enc_fail>10){
			GPIO_ResetBits(GPIOB,GPIO_Pin_0); pwm_off();
			printf("!! 编码器失败\r\n"); return;
		}

		e = elec_from_encoder();
		/* ★ 前 300 步加 90 度偏离，把转子从死点拉出来 */
		if(i < 300) e += 1.57079633f;
		while(e >= 6.28318531f) e -= 6.28318531f;

		st = (int)(e / 1.04719755f);          /* 每个扇区 60 度 */
		if(st>5) st=5; if(st<0) st=0;

		six_step(st, volts);

		c = cur_max();
		if(c > i_limit){
			GPIO_ResetBits(GPIOB,GPIO_Pin_0); pwm_off();
			printf("!! 过流 %.3fA (step=%d) !!\r\n",c,i); return;
		}

		if(st != last_st || (i%40)==0){
			printf("  i=%4d mech=%8.4f elec=%.3f st=%d I=%.3fA\r\n",
				i,enc_mech,e,st,c);
			last_st = st;
		}
		dly_ms(15);
	}
	pwm_off(); GPIO_ResetBits(GPIOB,GPIO_Pin_0);
	printf("完成: mech=%.4f ok=%lu fail=%lu\r\n",
		enc_mech,(unsigned long)enc_stat.ok,(unsigned long)enc_stat.fail);
}

/* 电流标定扫描：电压从 0.2V 缓慢上升到 3V，每个电压点保持 600ms 后读电流
 * 用已知电阻 (6.5 ohm) 反推电流，和读数对比，找出失真点 */
static void cmd_scan(void)
{
	float v;
	int st = 0;
	printf("\r\n===== 电流标定扫描 =====\r\n");
	printf("固定换相状态 st=0，电压逐步上升\r\n");
	printf("电压   理论电流   实测电流   反推电阻\r\n");
	GPIO_SetBits(GPIOB,GPIO_Pin_0);
	for(v = 0.2f; v <= 3.05f; v += 0.2f)
	{
		float i_th = v / 6.5f;
		float i_meas, r_eff;
		six_step(st, v);
		dly_ms(600);
		i_meas = cur_max();
		r_eff = (i_meas > 0.001f) ? (v / i_meas) : 0.0f;
		printf("  %.2fV   %.3fA    %.3fA    %.1f ohm\r\n", v, i_th, i_meas, r_eff);
		if(i_meas > 2.0f){ printf("  !! 超过 2A，停止扫描 !!\r\n"); break; }
	}
	pwm_off(); GPIO_ResetBits(GPIOB,GPIO_Pin_0);
	printf("扫描完成\r\n");
}

/* ★ 关键测试：给固定电角度偏离，看转子是否产生力矩并移动
 * 这是验证"死点"理论的直接实验：
 *   off=0    -> sin(0)=0    应该不动
 *   off=90°  -> sin(90°)=1  应该动
 *   off=180° -> sin(180°)=0 应该不动
 *   off=270° -> sin(270°)=-1 应该反向动
 */
static void cmd_torque_test(void)
{
	int k;
	float offs[4] = {0.0f, 1.57079633f, 3.14159265f, 4.71238898f};
	const char *names[4] = {"0deg", "90deg", "180deg", "270deg"};
	float settle;

	printf("\r\n===== 力矩/死点验证测试 =====\r\n");
	printf("V=%.2fV  每个角度保持 2 秒，观察转子是否移动\r\n", v_limit);
	if(!calibrated){ printf("!! 请先发 C 标定\r\n"); return; }

	GPIO_SetBits(GPIOB,GPIO_Pin_0);

	for(k=0;k<4;k++)
	{
		int i;
		float m0, m1;
		if(read_angle()){ printf("编码器失败\r\n"); break; }
		m0 = enc_mech;

		printf("\r\n-- off=%s --\r\n", names[k]);
		for(i=0;i<100;i++)        /* 100*20ms = 2s */
		{
			float e = elec_from_encoder() + offs[k];
			while(e >= 6.28318531f) e -= 6.28318531f;
			{
				int st = (int)(e / 1.04719755f);
				if(st>5) st=5; if(st<0) st=0;
				six_step(st, v_limit);
			}
			dly_ms(20);
			if(cur_max() > i_limit){
				GPIO_ResetBits(GPIOB,GPIO_Pin_0); pwm_off();
				printf("  !! 过流 %.3fA\r\n",cur_max()); return;
			}
			if(read_angle()==0) m1 = enc_mech;
		}
		read_angle();
		printf("  mech: %.4f -> %.4f   移动 %.4f rad    I=%.3fA\r\n",
			m0, enc_mech, enc_mech - m0, cur_max());
	}

	pwm_off(); GPIO_ResetBits(GPIOB,GPIO_Pin_0);
	printf("\r\n结论: off=90/270 应该移动明显, off=0/180 应该几乎不动\r\n");
}

static void cmd_selftest(void)
{
	printf("\r\n===== 自检 =====\r\n");
	printf("1) EN: %s\r\n", GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_0)?"!! 使能":"OK 失能");
	printf("2) PWM: %u/%u/%u %s\r\n",
		(unsigned)TIM1->CCR1,(unsigned)TIM1->CCR2,(unsigned)TIM1->CCR3,
		(TIM1->CCR1||TIM1->CCR2||TIM1->CCR3)?"!! 非零":"OK");
	printf("3) MOE: %s\r\n",(TIM1->BDTR&0x8000)?"OK":"!! 关闭");
	printf("4) 编码器: ");
	if(read_angle()) printf("!! 失败\r\n");
	else printf("OK mech=%.4f raw=%u\r\n",enc_mech,(unsigned)enc_raw);
	printf("5) 电流: %.3fA %s\r\n",cur_max(),(cur_max()>0.2f)?"!! 偏大":"OK");
	printf("6) 标定: %s\r\n", calibrated?"已标定":"未标定（发 C）");
	printf("7) Vlim=%.2f(->%.2fA) Ilim=%.2fA pp=%d\r\n",
		v_limit,v_limit/6.5f,i_limit,pole_pairs);
	printf("==============\r\n");
}

static void cmd_status(void)
{
	printf("EN=%d cal=%d Vlim=%.2f(->%.2fA) Ilim=%.2f\r\n",
		GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_0),calibrated,
		v_limit,v_limit/6.5f,i_limit);
	printf("enc mech=%.4f raw=%u",enc_mech,(unsigned)enc_raw);
	if(calibrated) printf("  elec=%.4f st=%d",elec_from_encoder(),
		(int)(elec_from_encoder()/1.04719755f));
	printf("\r\nI=%.3fA adc=%u/%u/%u  CCR=%u/%u/%u\r\n",
		cur_max(),(unsigned)adc_buf[0],(unsigned)adc_buf[1],(unsigned)adc_buf[2],
		(unsigned)TIM1->CCR1,(unsigned)TIM1->CCR2,(unsigned)TIM1->CCR3);
	printf("enc ok=%lu fail=%lu abort=%lu\r\n",
		(unsigned long)enc_stat.ok,(unsigned long)enc_stat.fail,
		(unsigned long)enc_stat.abort);
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

	printf("\r\n=== 六步闭环 ===\r\n");
	printf("SysClk=%lu\r\n",(unsigned long)clk_hz);

	encoder_i2c_init(100000);
	dly_ms(10);
	if(read_angle()) printf("!! 编码器初始读失败\r\n");
	else printf("编码器 OK mech=%.4f raw=%u\r\n",enc_mech,(unsigned)enc_raw);

	pwm_setup(); pwm_off();
	GPIO_SetBits(GPIOB,GPIO_Pin_0);
	dly_ms(300);
	adc_setup(); dly_ms(100);
	adc_cal_both();
	pwm_off();

	printf("电流零点 EN开 %.0f/%.0f/%.0f  EN关 %.0f/%.0f/%.0f\r\n",
		adc_zero_on[0],adc_zero_on[1],adc_zero_on[2],
		adc_zero_off[0],adc_zero_off[1],adc_zero_off[2]);
	printf("Vlim=%.2f(->%.2fA) Ilim=%.2fA pp=%d\r\n",
		v_limit,v_limit/6.5f,i_limit,pole_pairs);
	printf("\r\n命令: X 自检  C 标定  R 闭环  D 急停  A I  V L S\r\n\r\n");

	for(;;)
	{
		cmd_poll();
		if(cmd_ready())
		{
			switch(cmd_buf[0])
			{
				case '?': cmd_status(); break;
				case 'X': cmd_selftest(); break;
				case 'T': cmd_torque_test(); break;
				case 'K': cmd_scan(); break;
				case 'C': cmd_calibrate(); break;
				case 'R': cmd_closedloop(); break;
				case 'D': GPIO_ResetBits(GPIOB,GPIO_Pin_0); pwm_off();
						  printf("!! 急停 !!\r\n"); break;
				case 'A': if(read_angle()) printf("失败\r\n");
						  else printf("mech=%.4f raw=%u\r\n",enc_mech,(unsigned)enc_raw); break;
				case 'I': printf("I=%.3fA\r\n",cur_max()); break;
				case 'V': v_limit=atof((const char *)(cmd_buf+1));
						  if(v_limit>8.0f) v_limit=8.0f;
						  printf("Vlim=%.2f(->%.2fA)\r\n",v_limit,v_limit/6.5f); break;
				case 'L': i_limit=atof((const char *)(cmd_buf+1));
						  printf("Ilim=%.2fA\r\n",i_limit); break;
				case 'S': target_speed=atof((const char *)(cmd_buf+1));
						  printf("target=%.2f\r\n",target_speed); break;
				case 'P': pole_pairs=atoi((const char *)(cmd_buf+1));
						  if(pole_pairs<1) pole_pairs=1; calibrated=0;
						  printf("pp=%d\r\n",pole_pairs); break;
				default: break;
			}
			cmd_clear();
		}
	}
}




