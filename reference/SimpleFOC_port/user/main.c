
/* ============================================================================
 * SimpleFOC 移植 —— STM32F103C8T6(蓝板) + SimpleFOC Mini(DRV8316C) 正式固件
 *
 * 原始代码：loop222 @郑州  SimpleMotor 开发板
 *   教程：https://blog.csdn.net/loop222/article/details/120471390
 *
 * ============================ 安全性设计 ==================================
 * 1. 上电默认【失能】：EN 拉低，并且初始阶段先失能
 * 2. 限压优先：voltage_limit 默认 5V（约 0.8A），有明确注释说明依据
 * 3. 上电自检：先报告参数，让使用者核对电压/电流上限
 * 4. 分阶段启动：使能 -> 延时 -> 对齐，每步都可观察
 * 5. 随时可停：串口 'D' 立即失能；'S' 立即把目标清零并失能
 * 6. 关键参数可在线调（V/G/L），不用反复烧录
 *
 * ============================ 引脚分配 ====================================
 *   PWM1/2/3  PA8/PA9/PA10   TIM1_CH1/2/3
 *   EN        PB0            高电平使能
 *   串口      PA2/PA3        USART2
 *   编码器    PB6/PB7        I2C1 (AS5600)
 *   SWD       PA13/PA14
 *   LED       PC13
 * ==========================================================================*/
#include "stm32f10x.h"
#include <stdlib.h>
#include "MyProject.h"

/************************************************
 * 电机参数（按实测标定，改动前务必先读注释）
 * ---------------------------------------------------------------
 * 厂家参数: 24V / 额定 1.4A / 堵转 4.9A / 相电阻 5.4Ω / 功率 33W
 * 实测验证: 两相串联约 6.5Ω  ->  I = V / 6.5
 *     1.0V -> 0.14A     2.0V -> 0.31A     3.0V -> 0.46A
 *     5.0V -> 0.77A    10.0V -> 1.54A(超额定!)   15V -> 2.3A
 * 结论: voltage_limit 取 5V 比较安全（约 0.8A，额定 1.4A 的 57%）
 ************************************************/
#define SUPPLY_VOLTAGE_V     24.0f    /* 驱动电源电压 */
#define VOLTAGE_LIMIT_V       5.0f    /* 最大相电压 -> 直接决定电流 */
#define ALIGN_VOLTAGE_V       2.5f    /* 对齐电压，建议 0.5 * VOLTAGE_LIMIT_V */
#define VELOCITY_LIMIT_RS    20.0f    /* rad/s */
#define POLE_PAIRS            7       /* 极对数，按电机手册 */

/******************************************************************************/
#define LED_blink    GPIOC->ODR^=(1<<13)
/******************************************************************************/
float target;
/******************************************************************************/
void commander_run(void);
void print_status(void);
void print_safety(void);
/******************************************************************************/
void GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;         //PC13 LED
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOC,GPIO_Pin_13);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;          //PB0 = EN
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_0);                  //★ 上电默认失能
}
/******************************************************************************/
/* 时钟配置：原工程 SystemInit() 不配 PLL，实际只跑 HSI 8MHz。
 * 8MHz 会让 PWM 变成 2.8kHz、I2C 时序也跟着变，所以这里显式配到 72MHz。
 * 若 HSE(8MHz 晶振) 起振失败，退回 HSI/2*16 = 64MHz。 */
void RCC_Configuration(void)
{
	ErrorStatus hse_ok;
	uint32_t t;

	RCC_DeInit();
	RCC_HSEConfig(RCC_HSE_ON);
	hse_ok = RCC_WaitForHSEStartUp();

	FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
	FLASH_SetLatency(FLASH_Latency_2);
	RCC_HCLKConfig(RCC_SYSCLK_Div1);
	RCC_PCLK2Config(RCC_HCLK_Div1);
	RCC_PCLK1Config(RCC_HCLK_Div2);

	if(hse_ok == SUCCESS)
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);    /* 8*9 = 72MHz */
	else
		RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_16);   /* 4*16 = 64MHz */

	RCC_PLLCmd(ENABLE);
	t = 0;
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET && ++t < 0x200000);
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	while(RCC_GetSYSCLKSource() != 0x08);
	SystemCoreClockUpdate();
}
/******************************************************************************/
int main(void)
{
	unsigned int count_i=0;
	
	RCC_Configuration();       /* ★ 必须先配时钟：PWM/I2C/串口都依赖它 */
	GPIO_Config();
	uart_init(115200);
	
	printf("\r\n==============================\r\n");
	printf(" SimpleFOC / STM32F103 + DRV8316C\r\n");
	printf("==============================\r\n");
	printf("[1/6] GPIO ok, SysClk=%lu Hz\r\n", (unsigned long)SystemCoreClock);
	
#if M1_AS5600
	I2C_Init_();
	printf("[2/6] I2C1 (AS5600) on PB6/PB7\r\n");
#elif M1_TLE5012B
	SPI2_Init_();
	printf("[2/6] SPI2 (TLE5012B)\r\n");
#endif
	
	TIM2_PWM_Init();           /* 内部实际是 TIM1 -> PA8/PA9/PA10 */
	printf("[3/6] PWM: PA8/PA9/PA10 @ TIM1, 25kHz\r\n");
	
	TIM4_1ms_Init();
	printf("[4/6] TIM4 1ms tick\r\n");
	
	delay_ms(500);
	MagneticSensor_Init();
	printf("[5/6] encoder init\r\n");
	
	/* ---- 电机参数：全部先设好，再使能 ---- */
	voltage_power_supply = SUPPLY_VOLTAGE_V;
	pole_pairs           = POLE_PAIRS;
	voltage_limit        = VOLTAGE_LIMIT_V;
	velocity_limit       = VELOCITY_LIMIT_RS;
	voltage_sensor_align = ALIGN_VOLTAGE_V;
	if(voltage_sensor_align > voltage_limit) voltage_sensor_align = voltage_limit;
	torque_controller    = Type_voltage;
	controller           = Type_velocity;
	target               = 0;
	
	printf("[6/6] params set\r\n");
	print_safety();
	
	/* ---- 分阶段启动 ---- */
	printf("\r\n>>> 开始电机初始化（会失能->使能->对齐）\r\n");
	printf(">>> 对齐阶段电机会自己转约一圈，属正常现象，注意别夹手\r\n");
	printf(">>> 任何时候发 D 可立即失能\r\n\r\n");
	delay_ms(1500);                 /* 给使用者反应时间 */
	
	Motor_init();                   /* 使能 EN */
	Motor_initFOC();                /* 对齐 + 计算零点 */
	PID_init();
	printf("\r\n>>> Motor ready. 发 ? 看状态，发 T<x> 给目标\r\n");
	
	systick_CountMode();            /* 之后不能再调用 delay_ms/us */
	
	while(1)
	{
		count_i++;
		
		if(time1_cntr>=200)  /* 0.2s */
		{
			time1_cntr=0;
			LED_blink;
		}
		if(time2_cntr>=1000)
		{
			time2_cntr=0;
			count_i=0;
		}
		move(target);
		shaft_angle = shaftAngle();
		loopFOC();
		commander_run();
	}
}
/******************************************************************************/
void print_safety(void)
{
	printf("\r\n---------------- 安全参数 ----------------\r\n");
	printf(" 电源电压   : %.1f V\r\n", voltage_power_supply);
	printf(" 电压上限   : %.1f V   (-> 约 %.2f A)\r\n",
		voltage_limit, voltage_limit / 6.5f);
	printf(" 对齐电压   : %.1f V\r\n", voltage_sensor_align);
	printf(" 速度上限   : %.1f rad/s\r\n", velocity_limit);
	printf(" 极对数     : %d\r\n", pole_pairs);
	printf(" 电机额定   : 1.4 A / 堵转 4.9 A\r\n");
	printf("------------------------------------------\r\n");
	printf(" 危险: 电压上限超过 10V 时电流会超过额定值!\r\n");
	printf(" 急停: 随时发 D 立即失能\r\n\r\n");
}
/******************************************************************************/
void commander_run(void)
{
	if((USART_RX_STA&0x8000)!=0)
	{
		switch(USART_RX_BUF[0])
		{
			case 'H':
				printf("Hello World!\r\n");
				break;
			case 'T':   /* T6.28 速度模式给 rad/s，位置模式给 rad */
				target=atof((const char *)(USART_RX_BUF+1));
				printf("RX=%.4f\r\n", target);
				break;
			case 'S':   /* 切速度模式并清零目标 */
				controller=Type_velocity;
				target=0;
				printf("Mode: VELOCITY, target=0\r\n");
				break;
			case 'P':   /* 切位置模式，以当前位置为目标 */
				controller=Type_angle;
				MagneticSensor_Init();
				shaft_angle = shaftAngle();
				target = shaft_angle;
				printf("Mode: ANGLE\r\n");
				break;
			case '?':
				print_status();
				break;
			/* ---------- 安全相关 ---------- */
			case 'D':   /* ★ 急停：立即失能 */
				M1_Disable;
				target=0;
				printf("!! EMERGENCY STOP: EN OFF, target=0 !!\r\n");
				break;
			case 'E':   /* 使能 */
				M1_Enable;
				printf("EN ON (注意电机会立刻可以出力)\r\n");
				break;
			case 'V':   /* V<x> 在线改电压上限 */
				voltage_limit=atof((const char *)(USART_RX_BUF+1));
				if(voltage_limit > 13.0f){
					voltage_limit = 13.0f;
					printf("!! 已夹到 13V (Vbus/1.732) !!\r\n");
				}
				printf("voltage_limit=%.2fV -> 约 %.2fA\r\n",
					voltage_limit, voltage_limit/6.5f);
				if(voltage_limit > 10.0f)
					printf("!! 警告: 超过 10V 电流将超过电机额定 1.4A !!\r\n");
				break;
			case 'G':   /* G<x> 在线改对齐电压 */
				voltage_sensor_align=atof((const char *)(USART_RX_BUF+1));
				if(voltage_sensor_align > voltage_limit) voltage_sensor_align = voltage_limit;
				printf("align voltage=%.2fV\r\n", voltage_sensor_align);
				break;
			case 'L':   /* L<x> 在线改速度上限 */
				velocity_limit=atof((const char *)(USART_RX_BUF+1));
				printf("velocity_limit=%.2f rad/s\r\n", velocity_limit);
				break;
			case 'X':   /* 打印安全参数 */
				print_safety();
				break;
		}
		USART_RX_STA=0;
	}
}
/******************************************************************************/
void print_status(void)
{
	const char *m;
	switch(controller)
	{
		case Type_angle:             m = "ANGLE";        break;
		case Type_velocity:          m = "VELOCITY";     break;
		case Type_torque:            m = "TORQUE";       break;
		case Type_angle_openloop:    m = "ANGLE_OPEN";   break;
		case Type_velocity_openloop: m = "VEL_OPEN";     break;
		default:                     m = "?";            break;
	}
	printf("mode=%s angle=%.4f vel=%.4f target=%.4f\r\n",
		m, shaft_angle, shaft_velocity, target);
	printf("EN=%s  Vlim=%.2fV(约%.2fA)  dir=%ld  zero=%.3f\r\n",
		GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_0) ? "ON" : "OFF",
		voltage_limit, voltage_limit/6.5f,
		sensor_direction, zero_electric_angle);
}
/******************************************************************************/
