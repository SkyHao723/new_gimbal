/* ============================================================================
 * cal_curr.c —— 电流采样外部标定固件
 *
 * 目的：用外部电流表（功率计 / 万用表）标定 A_per_cnt 系数
 *
 * 原理：电机锁住不转时是纯电阻负载
 *         真实电流 = 施加电压 / 电阻
 *       固件读到的电流应当与之相等。两者之比就是系数校正量。
 *
 * 安全：全程固定磁场锁住转子（不旋转），电流单调可预测
 *       过流保护可关（标定时需要，但要盯紧电流表）
 *
 * 命令：
 *   ?        状态
 *   K<x>     在电角度 0 施加 <x> 伏，锁住转子，持续 60 秒（★ 标定主命令）
 *            例：K2  → 施加 2V，锁 60 秒，期间读外部电流表
 *   J<系数>  设定 A_per_cnt（例：J0.000269）
 *   O        过流保护开/关（标定时可关）
 *   D        急停
 *   A        读编码器
 *   C        自动扫描 1V/2V/3V/4V/5V 各 8 秒
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

/* ★ 多次采样做平均：解决"ADC 异步于 PWM 导致的剧烈抖动"
 * 每次读 32 个点取平均，同时报告波动范围（判断采样是否稳定） */
#define AVG_N 32
static volatile uint16_t adc_buf[3];
static float adc_zero_on[3]={2048,2048,2048}, adc_zero_off[3]={2048,2048,2048};
static float A_per_cnt=0.00269f;

/* 采 AVG_N 次取平均，返回均值；同时输出最小/最大用于判断波动 */
/* 均值结果也存起来，供 cur_max() 使用（否则 cur_max 还在用瞬时值，读数会抖） */
static volatile uint16_t adc_mean[3] = {2048,2048,2048};

static void adc_avg(uint16_t *o0, uint16_t *o1, uint16_t *o2)
{
	uint32_t i; uint32_t s0=0,s1=0,s2=0;
	for(i=0;i<AVG_N;i++){
		s0+=adc_buf[0]; s1+=adc_buf[1]; s2+=adc_buf[2];
	}
	adc_mean[0]=(uint16_t)(s0/AVG_N);
	adc_mean[1]=(uint16_t)(s1/AVG_N);
	adc_mean[2]=(uint16_t)(s2/AVG_N);
	*o0=adc_mean[0]; *o1=adc_mean[1]; *o2=adc_mean[2];
}

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
	ai.ADC_Mode=ADC_Mode_Independent;
	ai.ADC_ScanConvMode=ENABLE;
	ai.ADC_ContinuousConvMode=ENABLE;                     /* F103 的 TIM1_CC4 不能触发 ADC，
	                                                       * 先用连续转换 + 多次平均 */
	ai.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;
	ai.ADC_DataAlign=ADC_DataAlign_Right;
	ai.ADC_NbrOfChannel=3;
	ADC_Init(ADC1,&ai);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_1,2,ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_9,3,ADC_SampleTime_55Cycles5);
	ADC_DMACmd(ADC1,ENABLE); ADC_Cmd(ADC1,ENABLE);
	ADC_ResetCalibration(ADC1); while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1); while(ADC_GetCalibrationStatus(ADC1));

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
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

/* 返回三相电流（不取绝对值），用于看符号 */
static void cur3(float *ia, float *ib, float *ic)
{
	const float *z = GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_0)?adc_zero_on:adc_zero_off;
	*ia=((float)adc_mean[0]-z[0])*A_per_cnt;
	*ib=((float)adc_mean[1]-z[1])*A_per_cnt;
	*ic=((float)adc_mean[2]-z[2])*A_per_cnt;
}
static float cur_max(void)
{
	float a,b,c,m; cur3(&a,&b,&c);
	if(a<0)a=-a; if(b<0)b=-b; if(c<0)c=-c;
	m=a; if(b>m)m=b; if(c>m)m=c;
	return m;
}

/* 在电角度 ang 施加 v 伏（三相正弦换相，锁住转子） */
static void apply_field(float v, float ang)
{
	float va,vb,vc,half=(float)PWM_ARR*0.5f;
	int32_t d1,d2,d3;
	const float V_SUPPLY=24.0f;
	va=v*my_sin(ang);
	vb=v*my_sin(ang-2.09439510f);
	vc=v*my_sin(ang+2.09439510f);
	d1=(int32_t)(half+(va/V_SUPPLY)*half);
	d2=(int32_t)(half+(vb/V_SUPPLY)*half);
	d3=(int32_t)(half+(vc/V_SUPPLY)*half);
	if(d1<0)d1=0; if(d1>PWM_ARR)d1=PWM_ARR;
	if(d2<0)d2=0; if(d2>PWM_ARR)d2=PWM_ARR;
	if(d3<0)d3=0; if(d3>PWM_ARR)d3=PWM_ARR;
	TIM1->CCR1=(uint16_t)d1; TIM1->CCR2=(uint16_t)d2; TIM1->CCR3=(uint16_t)d3;
}

static uint8_t prot_on = 0;
static float   i_limit = 3.0f;
static float   v_limit = 6.0f;

/* ============================================================================
 * ★ 标定主命令：K<x> —— 在电角度 0 施加 x 伏，锁住 60 秒
 * 电角度 0 时三相电压为 (0, -0.866v, +0.866v)，即 B、C 两相之间加 1.5v
 * 这时候线电压 = 1.5 * x，电流 = 1.5x / R_line
 * ==========================================================================*/
static void cmd_lock(float v)
{
	int i;
	uint16_t m0,m1,m2;

	if(v > v_limit) v = v_limit;

	printf("\r\n=================== 电流标定（平均值采样）===================\r\n");
	printf("施加电压: %.2f V（电角度 0，定子磁场固定，转子锁死）\r\n", v);
	printf("线电压约 %.2f V\r\n", v*1.5f);
	printf("\r\n★ 每次打印的是【32 次采样的平均值】，已消除 PWM 异步采样抖动\r\n");
	printf("★ 读你的功率计，和下面的固件读数对比\r\n");
	printf("★ 60 秒后自动停止，随时发 D 急停\r\n\r\n");
	printf("  时间   B通道均值  C通道均值   偏差(相对EN开零点)   固件电流\r\n");

	GPIO_SetBits(GPIOB,GPIO_Pin_0);
	dly_ms(500);

	for(i=0;i<600;i++)              /* 60 秒 */
	{
		float ia,ib,ic;
		apply_field(v, 0.0f);
		dly_ms(100);
		adc_avg(&m0,&m1,&m2);
		if((i % 10) == 0)
		{
			cur3(&ia,&ib,&ic);
			printf("  %3ds   %4u      %4u     B:%+5d  C:%+5d      %.3f A\r\n",
				i/10, (unsigned)m1, (unsigned)m2,
				(int)m1-(int)adc_zero_on[1], (int)m2-(int)adc_zero_on[2],
				cur_max());
		}
		if(prot_on && cur_max() > i_limit){
			GPIO_ResetBits(GPIOB,GPIO_Pin_0); pwm_off();
			printf("!! 过流 %.3fA 切断 !!\r\n", cur_max()); return;
		}
		cmd_poll();
		if(cmd_ready()){
			if(cmd_buf[0]=='D'){ cmd_clear(); break; }
			cmd_clear();
		}
	}
	pwm_off(); GPIO_ResetBits(GPIOB,GPIO_Pin_0);
	printf("\r\n完成。请把【功率计读数】和【上面的均值】告诉我\r\n");
	printf("新系数 = 0.002690 x (真实电流 / 固件电流)\r\n");
}
/* ============================================================================
 * ★★ 直流标定（最可靠的方法）
 *
 * 之前的失败原因：PWM 开着时，ADC 平均值要么全落在"导通期"、
 * 要么全落在"关断期"，所以读数在 0.005A 和 0.18A 之间跳。
 *
 * 解法：标定时【完全关掉 PWM】，用固定占空比产生纯直流：
 *     CCR1 = 50% + k   (A 相接正)
 *     CCR2 = 50% - k   (B 相接负)
 *     CCR3 = 0         (C 相悬空)
 * 没有开关动作 → 没有纹波 → 读数必然稳定。
 * 此时电流 = k/1440*24V*2 / R，且恒定不变。
 * ==========================================================================*/
static void cmd_dc(float v)
{
	int i;
	/* 让 A-B 之间产生 v 伏：d = v/(2*24)*ARR */
	int32_t d = (int32_t)((v / (2.0f * 24.0f)) * (float)PWM_ARR);
	int32_t hi = PWM_ARR/2 + d;
	int32_t lo = PWM_ARR/2 - d;
	uint16_t m0,m1,m2;

	if(hi > PWM_ARR) hi = PWM_ARR;
	if(lo < 0) lo = 0;

	printf("\r\n============ 直流标定（PWM 静止，无纹波）============\r\n");
	printf("A-B 之间线电压 = %.2f V，C 相悬空\r\n", v);
	printf("CCR1=%ld CCR2=%ld CCR3=0\r\n", (long)hi, (long)lo);
	printf("\r\n★ 读功率计，和下面的 B 通道偏差对比\r\n");
	printf("★ 60 秒后自动停，随时发 D\r\n\r\n");
	printf("  时间  B通道  C通道   B偏差   C偏差   固件电流\r\n");

	GPIO_SetBits(GPIOB,GPIO_Pin_0);
	TIM1->CCR1=(uint16_t)hi;
	TIM1->CCR2=(uint16_t)lo;
	TIM1->CCR3=0;
	dly_ms(1000);            /* 等电流稳定 */

	for(i=0;i<600;i++)
	{
		float ia,ib,ic;
		dly_ms(100);
		adc_avg(&m0,&m1,&m2);
		if((i % 10) == 0)
		{
			cur3(&ia,&ib,&ic);
			printf("  %3ds  %4u   %4u   %+5d   %+5d    %.3f A\r\n",
				i/10, (unsigned)m1,(unsigned)m2,
				(int)m1-(int)adc_zero_on[1], (int)m2-(int)adc_zero_on[2],
				cur_max());
		}
		if(prot_on && cur_max() > i_limit){
			pwm_off(); GPIO_ResetBits(GPIOB,GPIO_Pin_0);
			printf("!! 过流 %.3fA !!\r\n",cur_max()); return;
		}
		cmd_poll();
		if(cmd_ready()){
			if(cmd_buf[0]=='D'){ cmd_clear(); break; }
			cmd_clear();
		}
	}
	pwm_off(); GPIO_ResetBits(GPIOB,GPIO_Pin_0);
	printf("\r\n完成。理论电流 = %.2fV / R，请对照功率计\r\n", v);
}

/* 自动扫描：1..5V 各 8 秒 */
static void cmd_scan(void)
{
	float v;
	printf("\r\n========== 自动扫描（每档 8 秒）==========\r\n");
	printf("请记录【每一档】功率计的电流读数\r\n\r\n");
	GPIO_SetBits(GPIOB,GPIO_Pin_0);
	for(v=1.0f; v<=5.05f; v+=1.0f)
	{
		int i;
		printf("--- %.1fV ---\r\n", v);
		for(i=0;i<80;i++){
			apply_field(v,0.0f);
			dly_ms(100);
			if((i%20)==0)
				printf("   %.1fV  固件=%.3fA  ADC=%u/%u/%u\r\n",
					v, cur_max(), (unsigned)adc_buf[0],
					(unsigned)adc_buf[1],(unsigned)adc_buf[2]);
			if(prot_on && cur_max()>i_limit){
				GPIO_ResetBits(GPIOB,GPIO_Pin_0); pwm_off();
				printf("!! 过流 !!\r\n"); return;
			}
		}
	}
	pwm_off(); GPIO_ResetBits(GPIOB,GPIO_Pin_0);
	printf("扫描完成\r\n");
}

static void cmd_status(void)
{
	float ia,ib,ic;
	cur3(&ia,&ib,&ic);
	printf("EN=%d prot=%d A_per_cnt=%.6f\r\n",
		GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_0),prot_on,A_per_cnt);
	printf("I = %.3f / %.3f / %.3f A   (max %.3f)\r\n",ia,ib,ic,cur_max());
	printf("ADC = %u/%u/%u   zero(EN%d)=%.0f/%.0f/%.0f\r\n",
		(unsigned)adc_buf[0],(unsigned)adc_buf[1],(unsigned)adc_buf[2],
		GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_0)?1:0,
		GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_0)?adc_zero_on[0]:adc_zero_off[0],
		GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_0)?adc_zero_on[1]:adc_zero_off[1],
		GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_0)?adc_zero_on[2]:adc_zero_off[2]);
	printf("CCR=%u/%u/%u  Vlim=%.1f Ilim=%.1f\r\n",
		(unsigned)TIM1->CCR1,(unsigned)TIM1->CCR2,(unsigned)TIM1->CCR3,v_limit,i_limit);
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

	printf("\r\n=== 电流标定固件 ===\r\n");
	printf("SysClk=%lu\r\n",(unsigned long)clk_hz);

	encoder_i2c_init(100000);
	pwm_setup(); pwm_off();
	GPIO_SetBits(GPIOB,GPIO_Pin_0);
	dly_ms(300);
	adc_setup(); dly_ms(100);
	adc_cal_both();
	pwm_off();
	GPIO_ResetBits(GPIOB,GPIO_Pin_0);

	printf("电流零点 EN开 %.0f/%.0f/%.0f  EN关 %.0f/%.0f/%.0f\r\n",
		adc_zero_on[0],adc_zero_on[1],adc_zero_on[2],
		adc_zero_off[0],adc_zero_off[1],adc_zero_off[2]);
	printf("当前 A_per_cnt=%.6f  (假设 0.3V/A 增益)\r\n", A_per_cnt);
	printf("\r\n命令:\r\n");
	printf("  B<x>   ★ 直流标定：A-B 间加 x 伏，PWM 静止无纹波（如 B2）\r\n");
	printf("  K<x>   三相正弦锁转子（PWM 有纹波，已不推荐）\r\n");
	printf("  C      自动扫描 1/2/3/4/5V\r\n");
	printf("  J<x>   设定系数（如 J0.000269）\r\n");
	printf("  O      过流保护开关（标定时可关）\r\n");
	printf("  D      急停    ?      状态    A  读编码器\r\n");
	printf("\r\n★ 先测电机两相电阻（万用表），再发 K2 开始标定\r\n\r\n");

	for(;;)
	{
		cmd_poll();
		if(cmd_ready())
		{
			switch(cmd_buf[0])
			{
				case '?': cmd_status(); break;
				case 'K': { float v=atof((const char *)(cmd_buf+1));
							if(v<=0) v=2.0f; cmd_lock(v); } break;
				case 'C': cmd_scan(); break;
				case 'B': { float v=atof((const char *)(cmd_buf+1)); if(v<=0) v=2.0f; cmd_dc(v); } break;
				case 'J': A_per_cnt=atof((const char *)(cmd_buf+1));
						  printf("A_per_cnt=%.6f\r\n",A_per_cnt); break;
				case 'O': prot_on = prot_on?0:1;
						  printf("过流保护 %s\r\n",prot_on?"ON":"OFF"); break;
				case 'D': GPIO_ResetBits(GPIOB,GPIO_Pin_0); pwm_off();
						  printf("!! 急停 !!\r\n"); break;
				case 'A': { uint16_t r; if(encoder_read_checked(&r,20,2)==0)
							  printf("raw=%u\r\n",(unsigned)r); else printf("读失败\r\n"); } break;
				default: break;
			}
			cmd_clear();
		}
	}
}






