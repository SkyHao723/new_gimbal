/**
 * ============================================================================
 * STM32F103C8T6(蓝板) + SimpleFOC Mini(DRV8316C) + AS5600 —— 官方 SimpleFOC 闭环
 *
 * 库：官方 askuric/Simple FOC 2.4.0（未改动算法，仅打了一个 core 兼容补丁）
 * 参考：官方示例
 *   examples/hardware_specific_examples/Bluepill_examples/magnetic_sensor/bluepill_position_control/
 *   examples/hardware_specific_examples/SimpleFOCMini/angle_control/
 *
 * 引脚
 *   PWM1/2/3 : PA8 / PA9 / PA10     TIM1_CH1/2/3   -> 驱动板 IN1/IN2/IN3
 *   EN       : PB0                                 -> 驱动板 EN
 *   串口     : PA2(TX) / PA3(RX)    USART2         (寄存器直驱 MyUart)
 *   编码器   : PB6(SCL) / PB7(SDA)  I2C1, AS5600
 *   SWD      : PA13 / PA14
 *
 * ★ 安全设计（实测电机 3.8Ω、额定 1.4A、24V 系统）
 *   - voltage_limit 默认 1.5V（约 0.4A）
 *   - 上电先对齐，然后【失能】，要发 E 才使能
 *   - 命令可随时改限压 / 急停
 *
 * 串口命令（115200）
 *   E        使能        D        失能(急停)
 *   T<n>     目标角度     V<n>     电压上限(V)
 *   L<n>     速度上限     R        读编码器角度
 *   ?        状态
 * ============================================================================
 */
#include <SimpleFOC.h>
#include "MyUart.h"

MyUart uart;

/* ---------------- 引脚 ---------------- */
#define PIN_PWM1   PA8
#define PIN_PWM2   PA9
#define PIN_PWM3   PA10
#define PIN_EN     PB0

/* ---------------- 参数 ---------------- */
#define POLE_PAIRS      7
#define SUPPLY_VOLTAGE  24.0f
#define V_LIMIT_INIT    1.5f      /* ★ 保守起步，约 0.4A */
#define ALIGN_VOLTAGE   1.0f
#define VEL_LIMIT_INIT  20.0f

/* ---------------- 对象 ---------------- */
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_PWM1, PIN_PWM2, PIN_PWM3, PIN_EN);

float target_angle = 0.0f;

/* ---------------- 状态输出 ---------------- */
static void print_status()
{
    uart.puts("EN=");      uart.puts(motor.enabled ? "ON" : "OFF");
    uart.puts("  angle="); uart.putf3(motor.shaft_angle);
    uart.puts("  vel=");   uart.putf3(motor.shaft_velocity);
    uart.puts("  tgt=");   uart.putf3(target_angle);
    uart.puts("  Vlim=");  uart.putf1(motor.voltage_limit);
    uart.puts("  Vbus=");  uart.putf1(SUPPLY_VOLTAGE);
    uart.puts("  dir=");   uart.puti((int)motor.sensor_direction);
    uart.puts("  zero=");  uart.putf3(motor.zero_electric_angle);
    uart.nl();
}

/* ---------------- 命令解析 ---------------- */
static char    cmd[32];
static uint8_t cmd_i = 0;

static float parse_float(const char *s)
{
    float v = 0.0f; int sign = 1; bool dec = false; float f = 0.1f;
    if (*s == '-') { sign = -1; s++; }
    while (*s) {
        if (*s == '.') { dec = true; s++; continue; }
        if (*s < '0' || *s > '9') break;
        if (!dec) v = v * 10.0f + (*s - '0');
        else { v += (*s - '0') * f; f *= 0.1f; }
        s++;
    }
    return sign * v;
}

static void run_command(char c, const char *arg)
{
    switch (c) {
    case 'E':
        motor.enable();
        uart.puts("EN ON\r\n");
        break;
    case 'D':
        motor.disable();
        uart.puts("EN OFF (急停)\r\n");
        break;
    case 'T':
        target_angle = parse_float(arg);
        uart.puts("target="); uart.putf3(target_angle); uart.nl();
        break;
    case 'V': {
        float v = parse_float(arg);
        if (v <= 0.0f) v = V_LIMIT_INIT;
        if (v > 6.0f)  { v = 6.0f; uart.puts("!! 已夹到 6V\r\n"); }
        motor.voltage_limit = v;
        uart.puts("Vlim="); uart.putf1(v);
        uart.puts("V (约 "); uart.putf1(v / 3.8f); uart.puts("A)\r\n");
        if (v > 3.0f) uart.puts("!! 警告: 超过 3V 时电流可能超过额定 1.4A\r\n");
        break;
    }
    case 'L':
        motor.velocity_limit = parse_float(arg);
        uart.puts("Vellim="); uart.putf1(motor.velocity_limit); uart.nl();
        break;
    case 'R':
        sensor.update();
        uart.puts("sensor angle="); uart.putf3(sensor.getAngle());
        uart.puts("  vel="); uart.putf3(sensor.getVelocity());
        uart.puts("  mech="); uart.putf3(sensor.getMechanicalAngle());
        uart.nl();
        break;
    case '?':
        print_status();
        break;
    default:
        uart.puts("命令: E D T<n> V<n> L<n> R ?\r\n");
        break;
    }
}

/* ---------------- 收命令 ---------------- */
static void poll_command()
{
    while (uart.available()) {
        char c = (char)(USART2->DR & 0xFF);
        if (c == '\r' || c == '\n') {
            if (cmd_i > 0) {
                cmd[cmd_i] = 0;
                run_command(cmd[0], cmd + 1);
                cmd_i = 0;
            }
        } else if (cmd_i < sizeof(cmd) - 1) {
            cmd[cmd_i++] = c;
        }
    }
}

/* ============================================================================ */
void setup()
{
    pinMode(PC13, OUTPUT);

    uart.begin(115200);
    delay(100);

    uart.puts("\r\n=============================================\r\n");
    uart.puts(" SimpleFOC 官方库 / STM32F103C8T6 + DRV8316C\r\n");
    uart.puts("=============================================\r\n");
    uart.puts("PWM : PA8 / PA9 / PA10\r\n");
    uart.puts("EN  : PB0\r\n");
    uart.puts("I2C : PB6(SCL) / PB7(SDA)  AS5600\r\n");
    uart.puts("SysClk="); uart.puti((int32_t)SystemCoreClock); uart.nl();
    uart.puts("---------------------------------------------\r\n");

    /* 1. 编码器 */
    uart.puts("[1/5] 编码器 AS5600 @0x36 ... ");
    sensor.init();
    sensor.update();
    motor.linkSensor(&sensor);
    uart.puts("angle="); uart.putf3(sensor.getAngle()); uart.nl();

    /* 2. 驱动 */
    uart.puts("[2/5] 驱动 PWM/EN ... ");
    driver.voltage_power_supply = SUPPLY_VOLTAGE;
    driver.voltage_limit = SUPPLY_VOLTAGE;
    driver.init();
    motor.linkDriver(&driver);
    uart.puts("OK\r\n");

    /* 3. 控制模式与参数 */
    uart.puts("[3/5] 参数设置 ... ");
    motor.foc_modulation    = FOCModulationType::SpaceVectorPWM;
    motor.controller        = MotionControlType::angle;
    motor.torque_controller = TorqueControlType::voltage;

    motor.voltage_limit     = V_LIMIT_INIT;
    motor.velocity_limit    = VEL_LIMIT_INIT;
    motor.voltage_sensor_align = ALIGN_VOLTAGE;

    motor.PID_velocity.P = 0.2f;
    motor.PID_velocity.I = 20.0f;
    motor.PID_velocity.D = 0.0f;
    motor.PID_velocity.output_ramp = 1000.0f;
    motor.LPF_velocity.Tf = 0.01f;
    motor.P_angle.P = 20.0f;
    uart.puts("OK\r\n");

    /* 4. 摘要 */
    uart.puts("---------------------------------------------\r\n");
    uart.puts("[4/5] 安全参数\r\n");
    uart.puts("      Vbus      = "); uart.putf1(SUPPLY_VOLTAGE); uart.puts(" V\r\n");
    uart.puts("      电压上限  = "); uart.putf1(motor.voltage_limit);
    uart.puts(" V  (约 ");  uart.putf1(motor.voltage_limit / 3.8f); uart.puts(" A)\r\n");
    uart.puts("      对齐电压  = "); uart.putf1(motor.voltage_sensor_align); uart.puts(" V\r\n");
    uart.puts("      速度上限  = "); uart.putf1(motor.velocity_limit); uart.puts(" rad/s\r\n");
    uart.puts("      极对数    = "); uart.puti(POLE_PAIRS); uart.nl();
    uart.puts("---------------------------------------------\r\n");

    /* 5. 电机初始化（会做对齐，电机可能轻微转动） */
    uart.puts("[5/5] motor.init() ... ");
    motor.init();
    uart.puts("OK\r\n");

    uart.puts("开始对齐 (initFOC) —— 电机会轻微转动，属正常\r\n");
    delay(1200);

    motor.initFOC();

    uart.puts("---------------------------------------------\r\n");
    uart.puts("对齐结果: dir=");
    uart.puts(motor.sensor_direction == Direction::CW ? "CW" : "CCW");
    uart.puts("  zero_electric_angle=");
    uart.putf3(motor.zero_electric_angle);
    uart.nl();

    /* ★ 对齐后失能，等用户确认 */
    motor.disable();
    uart.puts("电机已【失能】。确认安全后发 E 使能\r\n");
    uart.puts("命令: E=使能 D=急停 T<角度> V<限压> L<限速> R ?\r\n");
    uart.puts("建议先用 V1.5 限压，逐步验证\r\n");
    uart.puts("=============================================\r\n");
}

/* ============================================================================ */
void loop()
{
    static uint32_t last = 0;

    motor.loopFOC();
    motor.move(target_angle);

    poll_command();

    /* LED 心跳 */
    uint32_t now = millis();
    if (now - last > 500) {
        last = now;
        digitalWrite(PC13, (now / 500) % 2 ? LOW : HIGH);
    }
}
