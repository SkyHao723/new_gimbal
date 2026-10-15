/* ============================================================================
 * foc_idle_test.c —— 阶段 2：FOC 库集成 + 安全空转验证
 *
 * 目的：让 SimpleFOC 库跑起来，但【电机不转、不使能】，验证：
 *   1. 库与自写代码共存不会卡死（之前诊断固件卡在 b.n 自循环）
 *   2. shaft_angle / shaft_velocity 不再出现 NaN
 *   3. 编码器在库环境下仍然可靠（复用 encoder.c 的统计）
 *   4. 看门狗能正常喂（为后续闭环兜底）
 *
 * ★ 安全设计
 *   - 电机 EN 始终保持【失能】，本固件绝不使能
 *   - voltage.q / voltage.d 显式清零
 *     （FOCMotor.c 里是全局未初始化变量，loopFOC() 会直接把它写进 PWM！）
 *   - PWM 输出强制为 0，并持续校验
 *   - 目标恒为 0，不调用 alignSensor()
 *   - 独立看门狗 IWDG 兜底，卡死自动复位
 *
 * 引脚：PWM PA8/9/10 | EN PB0 | 串口 PA2/PA3 | I2C PB6/PB7
 *
 * 命令：
 *   ?        状态（角度/速度/循环速率/NaN 计数/编码器统计）
 *   A        读编码器角度（原始）
 *   Z        清零统计
 *   N        重置 NaN 计数
 *   M        打印电机参数
 *   S        打印 ASCII 折线图（角度随时间）
 * ==========================================================================*/
#include "MyProject.h"
#include "encoder.h"
#include "foc_utils.h"
#include "pid.h"
#include "lowpass_filter.h"
#include "BLDCMotor.h"

extern void RCC_Configuration(void);

/* ---------------- 自包含串口 USART2 (PA2/PA3) ---------------- */
#define RX_BUF_LEN 96
static volatile unsigned char rx_buf[RX_BUF_LEN];
static volatile unsigned short rx_sta = 0;

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
	USART2->BRR = (SystemCoreClock/2 + baud/2) / baud;
	USART2->CR1 = 0x200C;
	ni.NVIC_IRQChannel = USART2_IRQn;
	ni.NVIC_IRQChannelPreemptionPriority = 3;
	ni.NVIC_IRQChannelSubPriority = 3;
	ni.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&ni);
	USART2->CR1 |= 0x0020;
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
				else { rx_buf[rx_sta & 0x3FFF] = r; rx_sta++;
				       if(rx_sta > RX_BUF_LEN-1) rx_sta = 0; }
			}
		}
	}
}

/* ============================================================================
 * 编码器桥接 —— 把 SimpleFOC 库的 getAngle()/getVelocity() 接到 encoder.c
 * 这样库不直接碰 I2C，用的是阶段 1 验证过的健壮驱动。
 * ==========================================================================*/
/* 原 MagneticSensor.c 里定义的变量，库仍然引用 */
long  cpr = 4096;
float full_rotation_offset = 0.0f;
long  angle_data_prev = 0;
float angle_prev = 0.0f;
unsigned long velocity_calc_timestamp = 0;

static uint16_t enc_last_raw   = 0;
static float    enc_last_angle = 0.0f;
static uint32_t loop_ms        = 0;        /* 主循环毫秒计数 */

float getAngle(void)
{
	uint16_t raw;
	/* 读一次；失败则用上次的值（绝不返回 0 冒充有效角度） */
	if(encoder_read_checked(&raw, 20, 2) == 0)
	{
		float a = encoder_raw_to_rad(raw);
		/* 展开到连续角度，避免 2π 跳变 */
		float d = a - enc_last_angle;
		if(d >  3.14159265f) d -= 6.28318531f;
		if(d < -3.14159265f) d += 6.28318531f;
		enc_last_angle += d;
		enc_last_raw = raw;
		angle_prev = enc_last_angle;
	}
	return enc_last_angle;
}

float getVelocity(void)
{
	/* 用两次采样的角度差 / 时间差估算（简化版，够用于空转验证） */
	static float    prev_angle = 0.0f;
	static uint32_t prev_ms    = 0;
	float dt = (float)(loop_ms - prev_ms) * 1e-3f;
	float da = enc_last_angle - prev_angle;
	prev_angle = enc_last_angle;
	prev_ms    = loop_ms;
	if(dt <= 0.0001f) return 0.0f;
	return da / dt;
}

/* ============================================================================
 * 看门狗
 * ==========================================================================*/
static void iwdg_init(uint32_t ms)
{
	/* LSI ≈ 40kHz；0xFFFF/40 约 1638ms 为最大单级。
	 * 取 prescaler=64 -> 625Hz，reload = ms*0.625 */
	uint32_t rl = (ms * 5) / 8;
	if(rl > 0xFFF) rl = 0xFFF;
	if(rl < 1) rl = 1;
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_64);
	IWDG_SetReload((uint16_t)rl);
	IWDG_ReloadCounter();
	IWDG_Enable();
}
#define IWDG_FEED()  IWDG_ReloadCounter()

/* ============================================================================
 * 主程序
 * ==========================================================================*/
#define PWM_ARR 1440
static uint32_t clk_hz = 72000000;

static void gpio_init(void)
{
	GPIO_InitTypeDef gi;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|
	                       RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	gi.GPIO_Pin = GPIO_Pin_13; gi.GPIO_Mode = GPIO_Mode_Out_PP;
	gi.GPIO_Speed = GPIO_Speed_10MHz; GPIO_Init(GPIOC, &gi);
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);
	gi.GPIO_Pin = GPIO_Pin_0; gi.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &gi);
	GPIO_ResetBits(GPIOB, GPIO_Pin_0);        /* ★ EN 保持失能 */
}

/* 计数统计 */
static uint32_t nan_cnt = 0, loop_cnt = 0, loop_min = 0xFFFFFFFF, loop_max = 0;
static uint32_t loop_prev_ms = 0, loop_hz = 0;
static uint32_t run_sec = 0;

static void cmd_status(void)
{
	float a = shaft_angle, v = shaft_velocity;
	printf("---- FOC IDLE STATUS ----\r\n");
	printf("EN=%s (应为 OFF)\r\n", GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_0)?"ON":"OFF");
	printf("shaft_angle=%.4f rad  shaft_velocity=%.4f rad/s\r\n", a, v);
	printf("electrical_angle=%.4f  zero_elec=%.4f  dir=%ld  pp=%d\r\n",
		electrical_angle, zero_electric_angle, sensor_direction, pole_pairs);
	printf("voltage.q=%.4f  voltage.d=%.4f\r\n", voltage.q, voltage.d);
	printf("CCR %u/%u/%u  (ARR=%u)  <- 应全为 0\r\n",
		(unsigned)TIM1->CCR1,(unsigned)TIM1->CCR2,(unsigned)TIM1->CCR3,(unsigned)TIM1->ARR);
	printf("loop: %lu Hz  avg_period=%lu us  min=%lu max=%lu\r\n",
		(unsigned long)loop_hz,
		loop_cnt? (unsigned long)(1000000UL/loop_hz) : 0UL,
		(unsigned long)((loop_min==0xFFFFFFFF)?0:loop_min),
		(unsigned long)loop_max);
	printf("NaN count=%lu   run=%lu s\r\n", (unsigned long)nan_cnt,(unsigned long)run_sec);
	printf("encoder: ok=%lu fail=%lu abort=%lu recover=%lu speed=%luHz\r\n",
		(unsigned long)enc_stat.ok,(unsigned long)enc_stat.fail,
		(unsigned long)enc_stat.abort,(unsigned long)enc_stat.recover,
		(unsigned long)enc_stat.speed_hz);
	{
		uint32_t tot = enc_stat.ok + enc_stat.fail;
		if(tot) printf("encoder error rate = %.4f%%\r\n",
			(double)enc_stat.fail*100.0/(double)tot);
	}
	if(nan_cnt == 0 && GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_0)==0)
		printf(">>> HEALTHY\r\n");
	else
		printf(">>> 注意: 有 NaN 或 EN 非 OFF\r\n");
}

#define IS_NAN(x)  ((x) != (x))

int main(void)
{
	RCC_Configuration();
	clk_hz = SystemCoreClock;

	gpio_init();
	uart_init(115200);
	delay_ms(200);

	printf("\r\n=== FOC IDLE TEST (阶段2: 库集成, 电机不转) ===\r\n");
	printf("SysClk=%lu Hz\r\n", (unsigned long)clk_hz);
	printf("!!! 电机 EN 保持失能，本固件不会让电机转 !!!\r\n\r\n");

	/* --- 编码器（用阶段 1 的健壮驱动）--- */
	enc_stat.speed_hz = 100000;
	encoder_i2c_init(100000);
	delay_ms(10);
	{
		uint16_t raw;
		if(encoder_read_checked(&raw, 20, 3) == 0){
			enc_last_angle = encoder_raw_to_rad(raw);
			printf("编码器 OK, raw=%u angle=%.4f\r\n",(unsigned)raw, enc_last_angle);
		} else {
			printf("!! 编码器读取失败，检查接线\r\n");
		}
	}

	/* --- PWM（TIM1 -> PA8/9/10）--- */
	TIM2_PWM_Init();                 /* 函数名沿用工程，内部是 TIM1 */
	GPIO_ResetBits(GPIOB, GPIO_Pin_0);   /* 再次确保失能 */

	/* --- 电机参数 --- */
	voltage_power_supply = 24.0f;
	pole_pairs           = 7;
	voltage_limit        = 5.0f;      /* 安全值，对应约 0.77A */
	velocity_limit       = 10.0f;
	voltage_sensor_align = 2.5f;
	torque_controller    = Type_voltage;
	controller           = Type_velocity;
	shaft_angle_sp       = 0.0f;
	shaft_velocity_sp    = 0.0f;

	/* ★★★ 关键安全步骤 ★★★
	 * FOCMotor.c 里 voltage 是全局未初始化变量。
	 * loopFOC() 会无条件执行 setPhaseVoltage(voltage.q, voltage.d, ...)，
	 * 直接把未初始化的值写进 PWM！必须先清零。 */
	voltage.q = 0.0f;
	voltage.d = 0.0f;
	current.q = 0.0f;
	current.d = 0.0f;
	shaft_angle = 0.0f;
	shaft_velocity = 0.0f;
	sensor_direction = 1;
	zero_electric_angle = 0.0f;
	sensor_offset = 0.0f;

	/* PID 初始化（用库的保守默认值） */
	PID_init();

	/* 强制 PWM 为 0 */
	TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = 0;

	printf("库已集成: pwm=%s  encoder=%s\r\n",
		(TIM1->BDTR & 0x8000) ? "MOE=1" : "MOE=0", "encoder.c");
	printf("参数: Vbus=%.1f Vlim=%.1f pp=%d\r\n",
		voltage_power_supply, voltage_limit, pole_pairs);
	printf("voltage.q=%f voltage.d=%f (已清零)\r\n", voltage.q, voltage.d);

	/* 切换到 SysTick 无限计数模式（库的 PID/LPF 需要它作为时基）
	 * ★ 之后不能再调用 delay_ms()，会破坏时基 */
	systick_CountMode();

	iwdg_init(4000);      /* 4 秒看门狗 */

	printf("\r\n开始空转循环（目标恒 0）...\r\n");
	printf("命令: ? A Z N M S\r\n\r\n");

	loop_prev_ms = 0;
	for(;;)
	{
		uint32_t t0, t1, dt;

		IWDG_FEED();

		/* 循环速率统计 */
		t0 = SysTick->VAL;

		/* --- 读取传感器 --- */
		getAngle();                    /* 更新 enc_last_angle */

		/* --- 目标恒 0，绝不使能 --- */
		move(0.0f);
		loopFOC();

		/* ★ 强制 PWM 输出为 0：本阶段电机绝不能出力 */
		TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = 0;

		/* --- NaN 检测 --- */
		if(IS_NAN(shaft_angle) || IS_NAN(shaft_velocity)) nan_cnt++;
		if(IS_NAN(electrical_angle)) nan_cnt++;

		loop_cnt++;

		/* 周期统计（用 SysTick 差值；9MHz 下 1us ≈ 9 counts） */
		t1 = SysTick->VAL;
		dt = (t0 >= t1) ? (t0 - t1) : (0xFFFFFF - t1 + t0);
		{
			uint32_t us = dt / 9;
			if(us && us < loop_min) loop_min = us;
			if(us > loop_max) loop_max = us;
		}

		/* 每秒统计一次 */
		{
			static uint32_t acc = 0;
			acc++;
			if(acc >= 1000){
				acc = 0;
				run_sec++;
				loop_hz = loop_cnt;
				loop_cnt = 0;
				/* 每秒打印一次健康行，便于长时间观察 */
				printf("[%lu s] angle=%.4f vel=%.4f Hz=%lu NaN=%lu ok=%lu fail=%lu\r\n",
					(unsigned long)run_sec, shaft_angle, shaft_velocity,
					(unsigned long)loop_hz,(unsigned long)nan_cnt,
					(unsigned long)enc_stat.ok,(unsigned long)enc_stat.fail);
			}
		}

		/* --- 串口命令 --- */
		if(rx_sta & 0x8000)
		{
			switch(rx_buf[0])
			{
				case '?': cmd_status(); break;
				case 'A': { uint16_t raw; int e = encoder_read_checked(&raw,20,3);
					printf("raw=%u angle=%.4f %s\r\n",(unsigned)raw,
						encoder_raw_to_rad(raw), e?"FAIL":"OK"); } break;
				case 'Z': encoder_reset_stats(); nan_cnt=0; loop_min=0xFFFFFFFF; loop_max=0;
					printf("统计已清零\r\n"); break;
				case 'N': nan_cnt = 0; printf("NaN 计数已清零\r\n"); break;
				case 'M':
					printf("Vbus=%.1f Vlim=%.1f Valign=%.1f vlim=%.1f pp=%d\r\n",
						voltage_power_supply, voltage_limit, voltage_sensor_align,
						velocity_limit, pole_pairs);
					printf("controller=%d torque=%d dir=%ld\r\n",
						(int)controller,(int)torque_controller,sensor_direction);
					break;
				case 'S': {   /* ASCII 折线图 */
					int i;
					printf("角度折线（每 100ms 一点，共 40 点）:\r\n");
					for(i=0;i<40;i++){
						float a; uint32_t w;
						getAngle(); a = enc_last_angle;
						w = (uint32_t)((a + 7.0f) * 3.0f);
						if(w > 50) w = 50;
						printf("%6.3f |", a);
						while(w--) printf("#");
						printf("\r\n");
						for(w=0;w<300000;w++){ __asm volatile("nop"); }
						IWDG_FEED();
					}
					} break;
				default: break;
			}
			rx_sta = 0;
		}
	}
}
