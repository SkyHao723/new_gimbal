/* ============================================================================
 * spin2.c —— 阶段 3：带完整保护的闭环旋转测试（干净版）
 *
 * 基于已验证可用的 enc_probe.c 骨架（编码器读取零失败），加入：
 *   - TIM1 三路 PWM (PA8/PA9/PA10)
 *   - ADC 电流采样 (CS1/CS2/CS3 -> PA0/PA1/PB1)
 *   - 软启动 + 过流硬切断 + 编码器失联保护
 *
 * ★ 安全（多层）
 *   1. 电压上限默认 3.0V（约 0.46A，额定的 1/3）
 *   2. 过流阈值 1.5A，超限立即拉低 EN
 *   3. 软启动：0.5V 起，缓慢升到目标
 *   4. 编码器连续失败 > 20 次 → 判定失控，立即停
 *   5. 急停命令 D
 *   6. 无看门狗（调试阶段，避免复位干扰）
 *
 * ★ 与历史上失败的测试的关键区别
 *   之前给 10~12V 且磁场转速过快 → 转子失步 → 电流冲到堵转值（145W 事故）
 *   现在：Uq 逐步加、转速可调、每步查电流
 *
 * 命令：
 *   ?    状态
 *   X    安全自检（不动电机）
 *   S    软启动 + 旋转（★ 主命令）
 *   D    急停
 *   Z    零电流测试（锁定转子测电阻）
 *   A    读编码器角度
 *   I    读电流
 *   V<x> 电压上限    L<x> 过流阈值    F<x> 磁场转速 rad/s
 * ==========================================================================*/
#include "stm32f10x.h"
#include "encoder.h"
#include <stdio.h>
#include <stdlib.h>

extern void RCC_Configuration(void);

/* ---------------- 串口 ---------------- */

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
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	gi.GPIO_Pin = GPIO_Pin_2; gi.GPIO_Mode = GPIO_Mode_AF_PP;
	gi.GPIO_Speed = GPIO_Speed_50MHz; GPIO_Init(GPIOA, &gi);
	gi.GPIO_Pin = GPIO_Pin_3; gi.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &gi);
	USART2->BRR = (SystemCoreClock/2 + baud/2) / baud;
	/* ★ 只用轮询，绝对不要开 RXNEIE！
	 * 若开了中断但 cmd_poll() 用轮询读走数据、从不响应中断，
	 * RXNE 中断会无限触发，主循环永远得不到执行 ——
	 * 表现就是：启动信息正常，但所有命令无响应。 */
	USART2->CR1 = 0x200C;                 /* UE|TE|RE，无中断 */
	USART2->CR2 = 0;
	USART2->CR3 = 0;
}


/* ============================================================================
 * 命令接收 —— 轮询方式
 *
 * ★ 为什么不用中断
 *   中断方式在这个工程里反复出问题（标志位不同步、NVIC 状态异常），
 *   而轮询只需要读 USART2->SR 的 RXNE 位，简单且没有任何隐藏状态。
 *   主循环本来就在跑，轮询完全够用。
 * ==========================================================================*/
#define RX_MAX 64
static char     cmd_buf[RX_MAX];
static uint16_t cmd_len = 0;
static volatile uint8_t cmd_ready_flag = 0;

/* 每圈调用一次；收到完整一行（\n 结尾）就把 cmd_ready_flag 置 1 */
static void cmd_poll(void)
{
	while(USART2->SR & 0x20)              /* RXNE = bit5 */
	{
		uint8_t ch = (uint8_t)(USART2->DR & 0xFF);
		if(ch == 0x0A || ch == 0x0D)
		{
			if(cmd_len > 0){
				cmd_buf[cmd_len] = 0;
				cmd_ready_flag = 1;
			}
		}
		else if(cmd_len < RX_MAX-1)
		{
			cmd_buf[cmd_len++] = (char)ch;
		}
		else cmd_len = 0;                  /* 溢出则丢弃 */
	}
}

static int  cmd_ready(void){ return cmd_ready_flag ? 1 : 0; }
static void cmd_clear(void){ cmd_ready_flag = 0; cmd_len = 0; }
static uint32_t clk_hz = 72000000;
static void dly_ms(uint32_t ms){ volatile uint32_t n=(clk_hz/4000)*ms; while(n--) __asm volatile("nop"); }

/* ---------------- 正弦（自包含） ---------------- */
static float my_sin(float x)
{
	const float TWOPI = 6.28318531f;
	int sign = 1;
	while(x >= TWOPI) x -= TWOPI;
	while(x < 0.0f)   x += TWOPI;
	if(x > 3.14159265f){ x -= 3.14159265f; sign = -1; }
	if(x > 1.57079633f) x = 3.14159265f - x;
	{
		float x2 = x*x;
		return sign * x * (1.0f - x2/6.0f * (1.0f - x2/20.0f * (1.0f - x2/42.0f)));
	}
}

/* ---------------- PWM (TIM1) ---------------- */
#define PWM_ARR 1440

static void pwm_setup(void)
{
	GPIO_InitTypeDef gi;
	TIM_TimeBaseInitTypeDef tb;
	TIM_OCInitTypeDef oc;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_TIM1, ENABLE);
	gi.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;
	gi.GPIO_Mode = GPIO_Mode_AF_PP; gi.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gi);
	TIM_DeInit(TIM1);
	tb.TIM_Prescaler = 0; tb.TIM_Period = PWM_ARR - 1;
	tb.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	tb.TIM_ClockDivision = TIM_CKD_DIV1; tb.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &tb);
	oc.TIM_OCMode = TIM_OCMode_PWM1; oc.TIM_OutputState = TIM_OutputState_Enable;
	oc.TIM_Pulse = 0; oc.TIM_OCPolarity = TIM_OCPolarity_High;
	oc.TIM_OutputNState = TIM_OutputState_Disable; oc.TIM_OCNPolarity = TIM_OCNPolarity_High;
	oc.TIM_OCIdleState = TIM_OCIdleState_Reset; oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	TIM_OC1Init(TIM1,&oc); TIM_OC2Init(TIM1,&oc); TIM_OC3Init(TIM1,&oc);
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM1, ENABLE);
	TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

static void pwm_off(void){ TIM1->CCR1=0; TIM1->CCR2=0; TIM1->CCR3=0; }

/* ---------------- ADC 电流 ---------------- */
static volatile uint16_t adc_buf[3];
/* ★ 两套零点：CSA 在 EN 使能前后基线不同（实测差约 285 counts ≈ 0.77A）
 *   adc_zero_on  = EN 使能时的零点（电机运行时用这套）
 *   adc_zero_off = EN 失能时的零点（自检/停机时用这套） */
static float adc_zero_on[3]  = {2048,2048,2048};
static float adc_zero_off[3] = {2048,2048,2048};
static float A_per_cnt = 0.00269f;

static const float* cur_zero(void)
{
	return GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_0) ? adc_zero_on : adc_zero_off;
}

static void adc_setup(void)
{
	GPIO_InitTypeDef gi; ADC_InitTypeDef ai; DMA_InitTypeDef di;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_ADC1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	gi.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1; gi.GPIO_Mode = GPIO_Mode_AIN; GPIO_Init(GPIOA,&gi);
	gi.GPIO_Pin = GPIO_Pin_1;            gi.GPIO_Mode = GPIO_Mode_AIN; GPIO_Init(GPIOB,&gi);
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

static void adc_cal_zero_into(float *dst)
{
	uint32_t i; float s0=0,s1=0,s2=0;
	for(i=0;i<256;i++){ s0+=adc_buf[0]; s1+=adc_buf[1]; s2+=adc_buf[2]; dly_ms(1); }
	dst[0]=s0/256.0f; dst[1]=s1/256.0f; dst[2]=s2/256.0f;
}

/* 分别标定 EN 开/关两种状态下的零点 */
static void adc_cal_both(void)
{
	GPIO_SetBits(GPIOB, GPIO_Pin_0);      /* EN 开 */
	dly_ms(300);
	adc_cal_zero_into(adc_zero_on);
	GPIO_ResetBits(GPIOB, GPIO_Pin_0);    /* EN 关 */
	dly_ms(300);
	adc_cal_zero_into(adc_zero_off);
}

static float cur_max(void)
{
	float a,b,c,m;
	{ const float *z = cur_zero();
	  a=((float)adc_buf[0]-z[0])*A_per_cnt; if(a<0)a=-a;
	  b=((float)adc_buf[1]-z[1])*A_per_cnt; if(b<0)b=-b;
	  c=((float)adc_buf[2]-z[2])*A_per_cnt; if(c<0)c=-c; }
	m=a; if(b>m)m=b; if(c>m)m=c;
	return m;
}

/* ---------------- 状态 ---------------- */
static float v_limit = 3.0f;
static float i_limit = 1.5f;
static float field_speed = 2.0f;
static const float V_SUPPLY = 24.0f;
static float enc_angle = 0.0f;
static uint16_t enc_raw = 0;

static int read_angle(void)
{
	uint16_t raw;
	if(encoder_read_checked(&raw, 20, 2)) return 1;
	{
		float a = encoder_raw_to_rad(raw);
		float d = a - enc_angle;
		if(d >  3.14159265f) d -= 6.28318531f;
		if(d < -3.14159265f) d += 6.28318531f;
		enc_angle += d;
		enc_raw = raw;
	}
	return 0;
}

/* 三相正弦换相（simple 版，已通过六步换相验证同类思路）
 *   Vx = V * sin(angle)
 *   duty_x = 50% + (Vx / Vbus) * 50%
 * 三相各差 120 度 -> 合成旋转磁场
 * 这样每个桥臂都工作在 50% 附近，不会有电压矢量计算错误。 */
static void set_phase_voltage(float Uq, float angle_el)
{
	float va, vb, vc;
	float half = (float)PWM_ARR * 0.5f;

	if(Uq >  v_limit) Uq =  v_limit;
	if(Uq < -v_limit) Uq = -v_limit;

	va = Uq * my_sin(angle_el);
	vb = Uq * my_sin(angle_el - 2.09439510f);   /* -120 deg */
	vc = Uq * my_sin(angle_el + 2.09439510f);   /* +120 deg */

	/* duty = 50% + (V/Vbus)*50%  -> 围绕半占空比摆动，安全 */
	{
		int32_t d1 = (int32_t)(half + (va / V_SUPPLY) * half);
		int32_t d2 = (int32_t)(half + (vb / V_SUPPLY) * half);
		int32_t d3 = (int32_t)(half + (vc / V_SUPPLY) * half);
		if(d1 < 0) d1 = 0; if(d1 > PWM_ARR) d1 = PWM_ARR;
		if(d2 < 0) d2 = 0; if(d2 > PWM_ARR) d2 = PWM_ARR;
		if(d3 < 0) d3 = 0; if(d3 > PWM_ARR) d3 = PWM_ARR;
		TIM1->CCR1 = (uint16_t)d1;
		TIM1->CCR2 = (uint16_t)d2;
		TIM1->CCR3 = (uint16_t)d3;
	}
}
/* ---------------- 软启动旋转 ---------------- */
static void cmd_spin(void)
{
	float elec = 0.0f, v_now = 0.5f;
	float start_angle = enc_angle;
	int i, enc_fail = 0;
	const int STEP_MS = 20;

	printf("\r\n===== SPIN TEST =====\r\n");
	printf("目标电压 %.2fV  磁场转速 %.2f rad/s  过流 %.2fA\r\n",
		v_limit, field_speed, i_limit);
	printf("软启动: 0.5V 起，缓慢升到 %.2fV\r\n\r\n", v_limit);

	if(read_angle()){ printf("!! 编码器读失败，中止\r\n"); return; }
	start_angle = enc_angle;
	printf("起始角度 %.4f rad\r\n", start_angle);

	GPIO_SetBits(GPIOB, GPIO_Pin_0);

	for(i = 0; i < 3000; i++)
	{
		float c;

		if(v_now < v_limit) v_now += 0.25f * ((float)STEP_MS / 400.0f);

		elec += field_speed * ((float)STEP_MS * 1e-3f);
		while(elec >= 6.28318531f) elec -= 6.28318531f;

		set_phase_voltage(v_now, elec);

		c = cur_max();
		if(c > i_limit){
			GPIO_ResetBits(GPIOB, GPIO_Pin_0); pwm_off();
			printf("!! 过流 %.3fA > %.2fA —— 已切断 (V=%.2f, step=%d) !!\r\n",
				c, i_limit, v_now, i);
			return;
		}

		if(read_angle()) enc_fail++; else enc_fail = 0;
		if(enc_fail > 20){
			GPIO_ResetBits(GPIOB, GPIO_Pin_0); pwm_off();
			printf("!! 编码器连续 %d 次失败 —— 判定失控，已切断 !!\r\n", enc_fail);
			return;
		}

		if((i % 25) == 0)
			printf("  t=%5dms V=%.2f elec=%.2f ang=%8.3f I=%.3fA\r\n",
				i*STEP_MS, v_now, elec, enc_angle, c);

		dly_ms(STEP_MS);
	}

	GPIO_ResetBits(GPIOB, GPIO_Pin_0); pwm_off();
	printf("\r\n完成: 角度 %.3f -> %.3f (变化 %.3f rad)\r\n",
		start_angle, enc_angle, enc_angle - start_angle);
	printf("编码器: ok=%lu fail=%lu abort=%lu recover=%lu\r\n",
		(unsigned long)enc_stat.ok,(unsigned long)enc_stat.fail,
		(unsigned long)enc_stat.abort,(unsigned long)enc_stat.recover);
}

/* 简单旋转：用 Z 那种"两相加压"的方式，缓慢在 6 个状态间切换（六步换相）
 * 目的：区分"电流尖峰来自 SVPWM 换相"还是"来自电机/机械" */
static void cmd_sixstep(void)
{
	int i, st;
	float v = 1.0f;
	uint16_t d;
	float start;

	printf("\r\n===== 六步换相测试 (两相加压) =====\r\n");
	printf("V=%.2fV, 每状态 400ms, 共 12 步\r\n", v);
	if(read_angle()){ printf("!! 编码器失败\r\n"); return; }
	start = enc_angle;

	d = (uint16_t)((v / V_SUPPLY) * PWM_ARR);
	printf("duty=%u\r\n", (unsigned)d);

	GPIO_SetBits(GPIOB, GPIO_Pin_0);
	for(i = 0; i < 12; i++)
	{
		st = i % 6;
		switch(st){
			case 0: TIM1->CCR1=d; TIM1->CCR2=0; TIM1->CCR3=0; break;
			case 1: TIM1->CCR1=d; TIM1->CCR2=d; TIM1->CCR3=0; break;
			case 2: TIM1->CCR1=0; TIM1->CCR2=d; TIM1->CCR3=0; break;
			case 3: TIM1->CCR1=0; TIM1->CCR2=d; TIM1->CCR3=d; break;
			case 4: TIM1->CCR1=0; TIM1->CCR2=0; TIM1->CCR3=d; break;
			default:TIM1->CCR1=d; TIM1->CCR2=0; TIM1->CCR3=d; break;
		}
		dly_ms(300);
		read_angle();
		printf("  st=%d CCR=%u/%u/%u I=%.3fA ang=%.3f\r\n",
			st,(unsigned)TIM1->CCR1,(unsigned)TIM1->CCR2,(unsigned)TIM1->CCR3,
			cur_max(), enc_angle);
		if(cur_max() > i_limit){
			printf("  !! 过流 %.3fA !!\r\n", cur_max());
			break;
		}
	}
	pwm_off(); GPIO_ResetBits(GPIOB, GPIO_Pin_0);
	printf("完成: 角度 %.3f -> %.3f\r\n", start, enc_angle);
}

/* 固定角度正弦输出测试：验证 set_phase_voltage 本身是否有问题 */
static void cmd_fixed(void)
{
	float a;
	printf("\r\n===== 固定角度正弦测试 =====\r\n");
	GPIO_SetBits(GPIOB, GPIO_Pin_0);
	for(a = 0.0f; a < 6.30f; a += 0.78539816f)   /* 每 45 度 */
	{
		set_phase_voltage(1.0f, a);
		dly_ms(400);
		printf("  ang=%.2f CCR=%u/%u/%u I=%.3fA\r\n", a,
			(unsigned)TIM1->CCR1,(unsigned)TIM1->CCR2,(unsigned)TIM1->CCR3, cur_max());
		if(cur_max() > i_limit){ printf("  !! 过流 !!\r\n"); break; }
	}
	pwm_off(); GPIO_ResetBits(GPIOB, GPIO_Pin_0);
	printf("完成\r\n");
}

/* 关保护旋转：直接看电流曲线，判断是尖峰还是持续大电流 */
static void cmd_spin_noport(void)
{
	float elec = 0.0f, v_now = 0.5f;
	int i;
	printf("\r\n===== 无保护旋转（仅诊断！）V=0.5V F=0.5 =====\r\n");
	printf("!! 只跑 20 步，请盯电流表 !!\r\n");
	if(read_angle()){ printf("编码器失败\r\n"); return; }
	GPIO_SetBits(GPIOB, GPIO_Pin_0);
	for(i = 0; i < 20; i++)
	{
		elec += 0.5f * 0.02f;
		if(elec >= 6.28318531f) elec -= 6.28318531f;
		set_phase_voltage(v_now, elec);
		dly_ms(50);
		printf("  i=%2d elec=%.3f CCR=%u/%u/%u I=%.3fA\r\n", i, elec,
			(unsigned)TIM1->CCR1,(unsigned)TIM1->CCR2,(unsigned)TIM1->CCR3, cur_max());
		if(cur_max() > 6.0f){ printf("  !! 超过 6A，停止 !!\r\n"); break; }
	}
	pwm_off(); GPIO_ResetBits(GPIOB, GPIO_Pin_0);
	printf("完成\r\n");
}

static void cmd_zero(void)
{
	float v;
	GPIO_SetBits(GPIOB, GPIO_Pin_0);
	printf("零电流测试: 固定磁场\r\n");
	for(v = 0.5f; v <= v_limit; v += 0.5f)
	{
		uint16_t d = (uint16_t)((v / V_SUPPLY) * PWM_ARR);
		TIM1->CCR1 = d; TIM1->CCR2 = d; TIM1->CCR3 = 0;
		dly_ms(600);
		printf("  V=%.1fV I=%.3fA\r\n", v, cur_max());
		if(cur_max() > i_limit) break;
	}
	pwm_off(); GPIO_ResetBits(GPIOB, GPIO_Pin_0);
	printf("完成\r\n");
}

static void cmd_selftest(void)
{
	printf("\r\n===== 安全自检（不动电机）=====\r\n");
	printf("1) EN 失能: %s\r\n", GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_0)?"!! 仍使能":"OK");
	printf("2) PWM 全零: CCR=%u/%u/%u %s\r\n",
		(unsigned)TIM1->CCR1,(unsigned)TIM1->CCR2,(unsigned)TIM1->CCR3,
		(TIM1->CCR1||TIM1->CCR2||TIM1->CCR3)?"!! 非零":"OK");
	printf("3) TIM1 MOE: %s\r\n", (TIM1->BDTR&0x8000)?"OK":"!! 关闭");
	printf("4) 编码器: ");
	if(read_angle()) printf("!! 读失败\r\n");
	else printf("OK angle=%.4f raw=%u\r\n", enc_angle, (unsigned)enc_raw);
	printf("5) 电流: %.3fA (零漂 %s)\r\n", cur_max(),
		(cur_max()>0.2f)?"!! 偏大":"OK");
	printf("6) Vlim=%.2fV(->%.2fA) Ilim=%.2fA\r\n", v_limit, v_limit/6.5f, i_limit);
	if(v_limit > 4.0f) printf("   !! 警告: 电压上限偏高\r\n");
	printf("=============================\r\n");
}

static void cmd_status(void)
{
	printf("EN=%d\r\n", GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_0));
	printf("Vlim=%.2f(->%.2fA) Ilim=%.2f field=%.2f\r\n",
		v_limit, v_limit/6.5f, i_limit, field_speed);
	printf("angle=%.4f raw=%u\r\n", enc_angle, (unsigned)enc_raw);
	{ const float *z = cur_zero();
	  printf("I=%.3fA adc=%u/%u/%u zero(EN%d)=%.0f/%.0f/%.0f\r\n",
		cur_max(),(unsigned)adc_buf[0],(unsigned)adc_buf[1],(unsigned)adc_buf[2],
		GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_0)?1:0, z[0],z[1],z[2]); }
	printf("CCR=%u/%u/%u\r\n",(unsigned)TIM1->CCR1,(unsigned)TIM1->CCR2,(unsigned)TIM1->CCR3);
	printf("enc ok=%lu fail=%lu abort=%lu recover=%lu\r\n",
		(unsigned long)enc_stat.ok,(unsigned long)enc_stat.fail,
		(unsigned long)enc_stat.abort,(unsigned long)enc_stat.recover);
}

int main(void)
{
	RCC_Configuration();
	clk_hz = SystemCoreClock;

	{
		GPIO_InitTypeDef gi;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
		gi.GPIO_Pin = GPIO_Pin_13; gi.GPIO_Mode = GPIO_Mode_Out_PP;
		gi.GPIO_Speed = GPIO_Speed_10MHz; GPIO_Init(GPIOC, &gi);
		gi.GPIO_Pin = GPIO_Pin_0; gi.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOB, &gi);
		GPIO_ResetBits(GPIOB, GPIO_Pin_0);
	}
	uart_init(115200);
	dly_ms(200);

	printf("\r\n=== SPIN TEST (阶段3) ===\r\n");
	printf("SysClk=%lu\r\n", (unsigned long)clk_hz);

	/* 编码器 */
	encoder_i2c_init(100000);
	dly_ms(10);
	if(read_angle()) printf("!! 编码器初始读失败\r\n");
	else printf("编码器 OK angle=%.4f raw=%u\r\n", enc_angle,(unsigned)enc_raw);

	/* PWM（先初始化，输出保持 0）*/
	pwm_setup();
	pwm_off();

	/* 电流零点：需 EN 使能后 CSA 才工作 */
	GPIO_SetBits(GPIOB, GPIO_Pin_0);
	dly_ms(300);
	adc_setup();
	dly_ms(100);
	adc_cal_both();              /* 两套零点 */
	pwm_off();

	printf("电流零点 EN开 %.0f/%.0f/%.0f  EN关 %.0f/%.0f/%.0f\r\n",
		adc_zero_on[0],adc_zero_on[1],adc_zero_on[2],
		adc_zero_off[0],adc_zero_off[1],adc_zero_off[2]);
	printf("当前 I=%.3fA\r\n", cur_max());
	printf("Vlim=%.2fV(->%.2fA) Ilim=%.2fA\r\n", v_limit, v_limit/6.5f, i_limit);
	printf("\r\n命令: X=自检 S=软启动旋转 D=急停 Z=零电流 A I V<x> L<x> F<x> ?\r\n");
	printf("请先发 X\r\n\r\n");

	for(;;)
	{
		cmd_poll();                    /* ★ 轮询接收命令 */

		if(cmd_ready())
		{
			switch(cmd_buf[0])
			{
				case '?': cmd_status(); break;
				case 'X': cmd_selftest(); break;
				case 'U':   /* 诊断串口寄存器 */
					printf("USART2 SR=0x%04lX DR=0x%04lX BRR=%lu CR1=0x%04lX\r\n",
						(unsigned long)USART2->SR,(unsigned long)USART2->DR,
						(unsigned long)USART2->BRR,(unsigned long)USART2->CR1);
					printf("cmd_ready=%d cmd_len=%u\r\n", cmd_ready_flag, (unsigned)cmd_len);
					break;
				case 'S': cmd_spin(); break;
				case 'D': GPIO_ResetBits(GPIOB,GPIO_Pin_0); pwm_off();
						  printf("!! 急停 !!\r\n"); break;
				case 'Z': cmd_zero(); break;
				case 'G': cmd_sixstep(); break;
				case 'H': cmd_fixed(); break;
				case 'N': cmd_spin_noport(); break;
				case 'A': if(read_angle()) printf("读失败\r\n");
						  else printf("angle=%.4f raw=%u\r\n",enc_angle,(unsigned)enc_raw); break;
				case 'I': printf("I=%.3fA\r\n", cur_max()); break;
				case 'V': v_limit = atof((const char *)(cmd_buf+1));
						  if(v_limit>13.0f) v_limit=13.0f;
						  printf("Vlim=%.2f(->%.2fA)\r\n",v_limit,v_limit/6.5f);
						  if(v_limit>6.0f) printf("!! 警告: >6V 电流超 0.9A\r\n"); break;
				case 'L': i_limit = atof((const char *)(cmd_buf+1));
						  printf("Ilim=%.2fA\r\n",i_limit); break;
				case 'F': field_speed = atof((const char *)(cmd_buf+1));
						  if(field_speed>10.0f) field_speed=10.0f;
						  if(field_speed<0.3f)  field_speed=0.3f;
						  printf("field=%.2f rad/s\r\n",field_speed); break;
				default: break;
			}
			cmd_clear();
		}
	}
}












