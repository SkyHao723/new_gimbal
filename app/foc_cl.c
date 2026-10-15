/* ============================================================================
 * foc_cl.c —— 真正的闭环 FOC（基于已验证的组件）
 *
 * 已验证可用的部件：
 *   - TIM1 三路 PWM (PA8/9/10)          ✅
 *   - AS5600 编码器（绝对位置）          ✅
 *   - ADC 电流采样 + 过流保护            ✅
 *   - 三相正弦换相（固定角度测试正常）    ✅
 *   - 六步换相能转、电流 0.15A           ✅
 *
 * ★ 关键思路：AS5600 是【绝对位置】编码器，所以不需要传统对齐！
 *   传统对齐要"开环转一圈再看方向"，既慢又危险。
 *   绝对编码器可以这样做：
 *     1. 给一个固定电角度，把转子锁在已知位置
 *     2. 读编码器 -> 得到 机械角 与 电角度 的关系
 *     3. 之后就能直接算：elec = (enc_mech - offset) * pole_pairs
 *   这就是闭环 FOC 的换相角，完全不需要试探。
 *
 * 安全：
 *   1. 电压上限默认 2.0V（约 0.31A）
 *   2. 过流 1.5A 立即切断
 *   3. 标定只在低电压短暂进行
 *   4. 闭环速度限制可调
 *   5. 急停 D
 *
 * 命令：
 *   ?        状态
 *   X        安全自检（不动电机）
 *   C        标定（锁转子 -> 读编码器 -> 算偏移）★ 先做这个
 *   R        闭环旋转（速度控制）
 *   D        急停
 *   A        读编码器角度
 *   I        读电流
 *   V<x>     电压上限   L<x> 过流阈值   S<x> 目标速度 rad/s
 * ==========================================================================*/
#include "stm32f10x.h"
#include "encoder.h"
#include <stdio.h>
#include <stdlib.h>

extern void RCC_Configuration(void);

/* ---------------- 串口（轮询，不开中断） ---------------- */
#define RX_MAX 64
static char     cmd_buf[RX_MAX];
static uint16_t cmd_len = 0;
static volatile uint8_t cmd_ready_flag = 0;

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
	USART2->CR1 = 0x200C;          /* 轮询，不开 RXNEIE */
	USART2->CR2 = 0; USART2->CR3 = 0;
}

/* 每圈调用：收满一行就置标志 */
static void cmd_poll(void)
{
	while(USART2->SR & 0x20)
	{
		uint8_t ch = (uint8_t)(USART2->DR & 0xFF);
		if(ch == 0x0A || ch == 0x0D){
			if(cmd_len > 0){ cmd_buf[cmd_len] = 0; cmd_ready_flag = 1; }
		} else if(cmd_len < RX_MAX-1) cmd_buf[cmd_len++] = (char)ch;
		else cmd_len = 0;
	}
}
static int  cmd_ready(void){ return cmd_ready_flag ? 1 : 0; }
static void cmd_clear(void){ cmd_ready_flag = 0; cmd_len = 0; }

static uint32_t clk_hz = 72000000;
static void dly_ms(uint32_t ms){ volatile uint32_t n=(clk_hz/4000)*ms; while(n--) __asm volatile("nop"); }

/* ---------------- 正弦 ---------------- */
static float my_sin(float x)
{
	const float TWOPI = 6.28318531f;
	int sign = 1;
	while(x >= TWOPI) x -= TWOPI;
	while(x < 0.0f)   x += TWOPI;
	if(x > 3.14159265f){ x -= 3.14159265f; sign = -1; }
	if(x > 1.57079633f) x = 3.14159265f - x;
	{ float x2 = x*x;
	  return sign * x * (1.0f - x2/6.0f * (1.0f - x2/20.0f * (1.0f - x2/42.0f))); }
}

/* ---------------- PWM ---------------- */
#define PWM_ARR 1440
static void pwm_setup(void)
{
	GPIO_InitTypeDef gi; TIM_TimeBaseInitTypeDef tb; TIM_OCInitTypeDef oc;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_TIM1, ENABLE);
	gi.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;
	gi.GPIO_Mode = GPIO_Mode_AF_PP; gi.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gi);
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
static float A_per_cnt = 0.00269f;

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
static void adc_cal_into(float *dst)
{
	uint32_t i; float s0=0,s1=0,s2=0;
	for(i=0;i<256;i++){ s0+=adc_buf[0]; s1+=adc_buf[1]; s2+=adc_buf[2]; dly_ms(1); }
	dst[0]=s0/256.0f; dst[1]=s1/256.0f; dst[2]=s2/256.0f;
}
static void adc_cal_both(void)
{
	GPIO_SetBits(GPIOB,GPIO_Pin_0);  dly_ms(300); adc_cal_into(adc_zero_on);
	GPIO_ResetBits(GPIOB,GPIO_Pin_0); dly_ms(300); adc_cal_into(adc_zero_off);
}
static float cur_max(void)
{
	float a,b,c,m;
	const float *z = GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_0) ? adc_zero_on : adc_zero_off;
	a=((float)adc_buf[0]-z[0])*A_per_cnt; if(a<0)a=-a;
	b=((float)adc_buf[1]-z[1])*A_per_cnt; if(b<0)b=-b;
	c=((float)adc_buf[2]-z[2])*A_per_cnt; if(c<0)c=-c;
	m=a; if(b>m)m=b; if(c>m)m=c;
	return m;
}

/* ---------------- 状态 ---------------- */
static float v_limit   = 2.0f;
static float i_limit   = 1.5f;
static const float V_SUPPLY = 24.0f;
static int   pole_pairs = 7;

static float enc_mech = 0.0f;      /* 编码器机械角（连续展开，rad） */
static uint16_t enc_raw = 0;

/* 标定结果 */
static float elec_offset = 0.0f;   /* 电角度偏移 */
static int   sensor_dir = 1;       /* 编码器与磁场的方向关系 */
static int   calibrated = 0;

static int read_angle(void)
{
	uint16_t raw;
	if(encoder_read_checked(&raw, 20, 2)) return 1;
	{
		float a = encoder_raw_to_rad(raw);
		float d = a - enc_mech;
		if(d >  3.14159265f) d -= 6.28318531f;
		if(d < -3.14159265f) d += 6.28318531f;
		enc_mech += d;
		enc_raw = raw;
	}
	return 0;
}

/* 三相正弦换相（已验证） */
static void set_phase_voltage(float Uq, float angle_el)
{
	float va, vb, vc, half = (float)PWM_ARR * 0.5f;
	int32_t d1,d2,d3;
	if(Uq >  v_limit) Uq =  v_limit;
	if(Uq < -v_limit) Uq = -v_limit;
	va = Uq * my_sin(angle_el);
	vb = Uq * my_sin(angle_el - 2.09439510f);
	vc = Uq * my_sin(angle_el + 2.09439510f);
	d1 = (int32_t)(half + (va / V_SUPPLY) * half);
	d2 = (int32_t)(half + (vb / V_SUPPLY) * half);
	d3 = (int32_t)(half + (vc / V_SUPPLY) * half);
	if(d1<0)d1=0; if(d1>PWM_ARR)d1=PWM_ARR;
	if(d2<0)d2=0; if(d2>PWM_ARR)d2=PWM_ARR;
	if(d3<0)d3=0; if(d3>PWM_ARR)d3=PWM_ARR;
	TIM1->CCR1=(uint16_t)d1;
	TIM1->CCR2=(uint16_t)d2;
	TIM1->CCR3=(uint16_t)d3;
}

/* ============================================================================
 * 标定：绝对编码器专用
 *   1. 让转子停在已知电角度（施加固定电角度 + 低电压锁住）
 *   2. 读编码器机械角
 *   3. 算出偏移：elec_offset = enc_mech*pole_pairs - 施加的电角度
 * ==========================================================================*/
static void cmd_calibrate(void)
{
	int i;
	float lock_elec = 0.0f;

	printf("\r\n===== 标定（绝对编码器）=====\r\n");
	printf("步骤1: 低电压锁住转子 (V=%.2fV)\r\n", v_limit);

	if(read_angle()){ printf("!! 编码器读失败，中止\r\n"); return; }

	GPIO_SetBits(GPIOB, GPIO_Pin_0);

	/* 施加固定电角度，锁住转子。用正弦换相（已验证） */
	for(i = 0; i < 60; i++)        /* 60*20ms = 1.2s */
	{
		set_phase_voltage(v_limit, lock_elec);
		dly_ms(20);
		if(cur_max() > i_limit){
			GPIO_ResetBits(GPIOB,GPIO_Pin_0); pwm_off();
			printf("!! 过流 %.3fA，中止\r\n", cur_max()); return;
		}
	}

	/* 读锁住后的编码器角度（多读几次取平均） */
	{
		float sum = 0; int n = 0;
		for(i = 0; i < 10; i++){
			if(read_angle() == 0){ sum += enc_mech; n++; }
			dly_ms(20);
		}
		if(n == 0){
			GPIO_ResetBits(GPIOB,GPIO_Pin_0); pwm_off();
			printf("!! 编码器读失败\r\n"); return;
		}
		enc_mech = sum / (float)n;
		printf("步骤2: 锁住后编码器机械角 = %.4f rad (raw=%u)\r\n",
			enc_mech, (unsigned)enc_raw);
	}

	pwm_off();
	GPIO_ResetBits(GPIOB, GPIO_Pin_0);
	dly_ms(300);

	/* 计算偏移：使 elec = (enc_mech - enc0)*pp*dir + lock_elec
	 * 即 enc0 是"机械角零点" */
	elec_offset = enc_mech * (float)pole_pairs - lock_elec;
	sensor_dir = 1;
	calibrated = 1;

	printf("步骤3: 标定完成\r\n");
	printf("  电角度偏移 = %.4f\r\n", elec_offset);
	printf("  极对数 = %d  方向 = %d\r\n", pole_pairs, sensor_dir);
	printf("  验证公式: elec = enc_mech*%d - %.4f\r\n", pole_pairs, elec_offset);
	printf("  现在可以发 R 进入闭环\r\n");
}

/* 由编码器算电角度 */
static float elec_from_encoder(void)
{
	float e = (enc_mech * (float)pole_pairs - elec_offset) * (float)sensor_dir;
	/* 归一化到 0..2π */
	while(e >= 6.28318531f) e -= 6.28318531f;
	while(e < 0.0f)         e += 6.28318531f;
	return e;
}

/* ============================================================================
 * 闭环旋转：速度 PI -> Uq -> 正弦换相（换相角来自编码器）
 * ==========================================================================*/
static float target_speed = 2.0f;   /* rad/s 机械 */

static void cmd_closedloop(void)
{
	float v_now = 0.3f;            /* 软启动电压 */
	float integ = 0.0f;
	float prev_mech, dt, vel;
	uint32_t t0, t1;
	int i, enc_fail = 0;
	int N = 800;                   /* 800 * 10ms = 8s */

	if(!calibrated){ printf("!! 请先发 C 标定\r\n"); return; }
	if(read_angle()){ printf("!! 编码器失败\r\n"); return; }

	printf("\r\n===== 闭环 FOC =====\r\n");
	printf("目标速度 %.2f rad/s  电压上限 %.2fV  过流 %.2fA\r\n",
		target_speed, v_limit, i_limit);
	printf("换相角来自编码器（绝对位置）\r\n\r\n");

	GPIO_SetBits(GPIOB, GPIO_Pin_0);
	prev_mech = enc_mech;
	t0 = SysTick->VAL;

	for(i = 0; i < N; i++)
	{
		float elec, err, uq, c;

		/* --- 读编码器 --- */
		if(read_angle()){ enc_fail++; } else enc_fail = 0;

		/* ★ 起动阶段：先开环小步推进电角度，把转子"带起来"
		 * 原因：闭环下磁场角来自编码器，转子不动 -> 电角度不变 -> 磁场静止
		 *       -> 没有起动力矩（原地锁死）。
		 * 前 200 步用开环推进，之后切回闭环。 */
		if(i < 200)
		{
			static float ol_elec = 0.0f;
			ol_elec += 0.02f;                  /* 每步 0.02 rad 电角度 */
			if(ol_elec >= 6.28318531f) ol_elec -= 6.28318531f;
			set_phase_voltage(v_now, ol_elec);
			if((i % 20) == 0)
				printf("  [开环] i=%3d elec=%.3f uq=%.3f I=%.3fA mech=%.4f\r\n",
					i, ol_elec, v_now, cur_max(), enc_mech);
			if(cur_max() > i_limit){
				GPIO_ResetBits(GPIOB,GPIO_Pin_0); pwm_off();
				printf("!! 过流 %.3fA 切断\r\n", cur_max()); return;
			}
			dly_ms(10);
			continue;
		}
		if(enc_fail > 10){
			GPIO_ResetBits(GPIOB,GPIO_Pin_0); pwm_off();
			printf("!! 编码器连续失败，切断\r\n"); return;
		}

		/* --- 测速 --- */
		t1 = SysTick->VAL;
		{
			uint32_t dtk = (t0 >= t1) ? (t0 - t1) : (0xFFFFFFu - t1 + t0);
			dt = (float)dtk / 9.0f * 1e-6f;
		}
		t0 = t1;
		if(dt < 1e-5f) dt = 1e-3f;
		vel = (enc_mech - prev_mech) / dt;
		prev_mech = enc_mech;

		/* --- PI 速度环 --- */
		err = target_speed - vel;
		integ += err * dt;
		if(integ >  2.0f) integ =  2.0f;      /* antiwindup */
		if(integ < -2.0f) integ = -2.0f;
		uq = 0.5f * err + 1.0f * integ;       /* P=0.5 I=1.0，保守 */
		if(uq >  v_now) uq =  v_now;
		if(uq < -v_now) uq = -v_now;

		/* --- 用编码器算出的电角度换相 --- */
		elec = elec_from_encoder();
		set_phase_voltage(uq, elec);

		/* --- 软启动：电压上限慢慢放开 --- */
		if(v_now < v_limit) v_now += 0.3f * dt * 10.0f;

		/* --- 过流保护 --- */
		c = cur_max();
		if(c > i_limit){
			GPIO_ResetBits(GPIOB,GPIO_Pin_0); pwm_off();
			printf("!! 过流 %.3fA 切断\r\n", c); return;
		}

		if(i < 15 || (i % 20) == 0)
			printf("  i=%4d mech=%8.4f raw=%4u elec=%.3f vel=%7.3f uq=%6.3f I=%.3fA err=%d\r\n",
				i, enc_mech, (unsigned)enc_raw, elec, vel, uq, c, enc_fail);

		dly_ms(10);
	}
	pwm_off();
	GPIO_ResetBits(GPIOB, GPIO_Pin_0);
	printf("闭环完成: 最终角度 %.3f, 编码器 ok=%lu fail=%lu\r\n",
		enc_mech,(unsigned long)enc_stat.ok,(unsigned long)enc_stat.fail);
}

static void cmd_selftest(void)
{
	printf("\r\n===== 安全自检 =====\r\n");
	printf("1) EN 失能: %s\r\n", GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_0)?"!! 仍使能":"OK");
	printf("2) PWM 全零: %u/%u/%u %s\r\n",
		(unsigned)TIM1->CCR1,(unsigned)TIM1->CCR2,(unsigned)TIM1->CCR3,
		(TIM1->CCR1||TIM1->CCR2||TIM1->CCR3)?"!! 非零":"OK");
	printf("3) TIM1 MOE: %s\r\n", (TIM1->BDTR&0x8000)?"OK":"!! 关闭");
	printf("4) 编码器: ");
	if(read_angle()) printf("!! 失败\r\n");
	else printf("OK mech=%.4f raw=%u\r\n", enc_mech, (unsigned)enc_raw);
	printf("5) 电流: %.3fA %s\r\n", cur_max(), (cur_max()>0.2f)?"!! 偏大":"OK");
	printf("6) 已标定: %s", calibrated ? "是" : "否（请发 C）");
	if(calibrated) printf("  offset=%.4f", elec_offset);
	printf("\r\n7) Vlim=%.2f(->%.2fA) Ilim=%.2fA pp=%d\r\n",
		v_limit, v_limit/6.5f, i_limit, pole_pairs);
	printf("=====================\r\n");
}

static void cmd_status(void)
{
	printf("EN=%d cal=%d\r\n", GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_0), calibrated);
	printf("Vlim=%.2f(->%.2fA) Ilim=%.2f target=%.2f pp=%d\r\n",
		v_limit, v_limit/6.5f, i_limit, target_speed, pole_pairs);
	printf("enc mech=%.4f raw=%u\r\n", enc_mech, (unsigned)enc_raw);
	if(calibrated) printf("elec=%.4f offset=%.4f dir=%d\r\n",
		elec_from_encoder(), elec_offset, sensor_dir);
	printf("I=%.3fA adc=%u/%u/%u\r\n",
		cur_max(),(unsigned)adc_buf[0],(unsigned)adc_buf[1],(unsigned)adc_buf[2]);
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

	printf("\r\n=== 闭环 FOC (foc_cl) ===\r\n");
	printf("SysClk=%lu\r\n", (unsigned long)clk_hz);

	encoder_i2c_init(100000);
	dly_ms(10);
	if(read_angle()) printf("!! 编码器初始读失败\r\n");
	else printf("编码器 OK mech=%.4f raw=%u\r\n", enc_mech,(unsigned)enc_raw);

	pwm_setup();
	pwm_off();

	GPIO_SetBits(GPIOB,GPIO_Pin_0);
	dly_ms(300);
	adc_setup();
	dly_ms(100);
	adc_cal_both();
	pwm_off();

	printf("电流零点 EN开 %.0f/%.0f/%.0f  EN关 %.0f/%.0f/%.0f\r\n",
		adc_zero_on[0],adc_zero_on[1],adc_zero_on[2],
		adc_zero_off[0],adc_zero_off[1],adc_zero_off[2]);
	printf("Vlim=%.2fV(->%.2fA) Ilim=%.2fA pp=%d\r\n",
		v_limit, v_limit/6.5f, i_limit, pole_pairs);
	printf("\r\n命令: X=自检 C=标定 R=闭环旋转 D=急停 A I\r\n");
	printf("       V<x>=限压 L<x>=过流 S<x>=目标速度\r\n");
	printf("★ 请先发 X 自检，再发 C 标定\r\n\r\n");

	for(;;)
	{
		cmd_poll();
		if(cmd_ready())
		{
			switch(cmd_buf[0])
			{
				case '?': cmd_status(); break;
				case 'X': cmd_selftest(); break;
				case 'C': cmd_calibrate(); break;
				case 'R': cmd_closedloop(); break;
				case 'D': GPIO_ResetBits(GPIOB,GPIO_Pin_0); pwm_off();
						  printf("!! 急停 !!\r\n"); break;
				case 'A': if(read_angle()) printf("读失败\r\n");
						  else printf("mech=%.4f raw=%u\r\n",enc_mech,(unsigned)enc_raw); break;
				case 'I': printf("I=%.3fA\r\n", cur_max()); break;
				case 'V': v_limit = atof((const char *)(cmd_buf+1));
						  if(v_limit>6.0f) v_limit=6.0f;
						  printf("Vlim=%.2f(->%.2fA)\r\n",v_limit,v_limit/6.5f); break;
				case 'L': i_limit = atof((const char *)(cmd_buf+1));
						  printf("Ilim=%.2fA\r\n",i_limit); break;
				case 'S': target_speed = atof((const char *)(cmd_buf+1));
						  printf("target=%.2f rad/s\r\n",target_speed); break;
				case 'P': pole_pairs = atoi((const char *)(cmd_buf+1));
						  if(pole_pairs<1) pole_pairs=1;
						  calibrated = 0;
						  printf("pole_pairs=%d (需重新标定)\r\n",pole_pairs); break;
				default: break;
			}
			cmd_clear();
		}
	}
}



