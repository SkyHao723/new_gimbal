#ifndef HOST_CONFIG_H
#define HOST_CONFIG_H

/* 上位机板（G431 作三块驱动板的 CAN 主机）引脚定义。
   板子状态：已打样、部分焊接，固件按下面的定义写，焊好后逐项验证。

   PA9  / PA10 : USART1  <-> CH343P <-> USB（和电脑通信，115200 8N1）
   PB8  / PB9  : FDCAN1  <-> TJA1042（CAN 主机，500 kbit/s，与驱动板同一套参数）
   PB6  / PB7  : I2C1    <-> ICM-42670-P（六轴 IMU，地址 0x68 或 0x69）
   PA5         : 工作指示灯
   PA2  / PA3  : USART2  <-> ESP32-S3（功能暂缓，只初始化不通信）
*/

/* ---- 指示灯 ---------------------------------------------------------- */
#define HOST_LED_PORT            GPIOA
#define HOST_LED_PIN             GPIO_PIN_5
/* 驱动板上 PA5 是低电平点亮；上位机板不确定，先按低电平点亮写。
   焊好后如果发现常亮/常灭反了，把这里改成 0 即可。 */
#define HOST_LED_ACTIVE_LOW      1

/* ---- 与电脑的串口（CH343P） ------------------------------------------ */
#define HOST_CONSOLE_BAUD        115200u

/* ---- IMU（ICM-42670-P） ---------------------------------------------- */
#define HOST_I2C_INSTANCE        I2C1
#define HOST_I2C_TIMING_100K     0xF0413335u  /* 170 MHz PCLK1 -> 100 kHz */
#define HOST_I2C_TIMING_50K      0xF0813335u
#define HOST_I2C_TIMING_10K      0xF0429A9Au
#define HOST_IMU_ADDR_PRIMARY    0x68u
#define HOST_IMU_ADDR_SECONDARY  0x69u
#define HOST_IMU_WHO_AM_I_REG    0x75u
#define HOST_IMU_WHO_AM_I_VALUE  0x67u
#define HOST_IMU_BANK_SEL_REG    0x76u
#define HOST_IMU_ADDR_SCAN_FIRST 0x08u
#define HOST_IMU_ADDR_SCAN_LAST  0x77u

/* ---- CAN（TJA1042） -------------------------------------------------- */
/* 驱动板的 TJA1042 STB 由 PB5 控制（低=正常）。上位机板没在引脚表里提到
   PB5，如果它的 STB 是直接接地的，保持 0；如果实测收发器不发，把这里改 1
   由 PB5 拉低使能。 */
#define HOST_CAN_DRIVE_STB_ON_PB5  0
#define HOST_CAN_NODE               0u     /* 0 = 主机/广播，不占用 1..7 */
#define HOST_CAN_BROADCAST_ID       0x200u
#define HOST_CAN_DRIVER_CMD_BASE    0x201u /* node1..node7 -> 0x201..0x207 */
#define HOST_CAN_DRIVER_REPLY_BASE  0x281u /* node1..node7 -> 0x281..0x287 */

/* ---- 与 ESP32-S3 的串口（暂缓） -------------------------------------- */
#define HOST_ESP_BAUD               115200u

/* ---- 硬件排查用：I2C 引脚推挽驱动测试 -------------------------------- */
/* 1 = 开机后把 PB6/PB7 当普通推挽输出，5 s 低 / 5 s 高循环翻转，方便用
 *     万用表直接量 IMU 的 SDA/SCL 引脚，判断"MCU 到芯片这段线到底通不通"。
 *     测出来如果 IMU 侧电压跟着翻，线就是通的；不跟就是断线/虚焊/走线错。
 * 0 = 正常固件（自检后进入 I2C 模式）。默认 0，排查时临时改 1。 */
#define HOST_I2C_PIN_DRIVE_TEST     1

#endif
