/**
 * ============================================================================
 * STM32F103C8T6 + SimpleFOC Mini(DRV8316C) + AS5600 —— 闭环位置控制
 *
 * 库：官方 SimpleFOC 2.4.0
 * 参考官方示例：
 *   examples/hardware_specific_examples/Bluepill_examples/magnetic_sensor/
 *   examples/hardware_specific_examples/SimpleFOCMini/angle_control/
 *
 * ---------------------------------------------------------------------------
 * ★ 串口用 Serial2（不是 Serial1）
 *
 *   Arduino STM32 core 的 cores/arduino/Serial.cpp 里：
 *       Uart Serial1(USART1);   <- Serial1 是 USART1 (PA9/PA10，已被 PWM 占用)
 *       Uart Serial2(USART2);   <- Serial2 才是 USART2 (PA2/PA3)
 *   之前用 Serial1 想接 USART2，实际接的是 USART1，导致命令丢一半。
 *
 *   platformio.ini 里必须定义（否则 Serial2 不存在）：
 *       -D HAVE_HWSERIAL2
 *       -D PIN_SERIAL2_TX=PA2
 *       -D PIN_SERIAL2_RX=PA3
 * ---------------------------------------------------------------------------
 *
 * 引脚
 *   PWM1/2/3 : PA8 / PA9 / PA10   -> 驱动板 IN1/IN2/IN3
 *   EN       : PB0                -> 驱动板 EN
 *   串口     : PA2(TX) / PA3(RX)  USART2 = Serial2, 115200
 *   编码器   : PB6(SCL) / PB7(SDA)  I2C1, AS5600
 *   SWD      : PA13 / PA14
 *   LED      : PC13
 *
 * 串口命令
 *   E       使能               D       失能 / 急停
 *   T<n>    目标角度 (rad)      V<n>    电压上限 (V)
 *   L<n>    速度上限 (rad/s)    R       读编码器
 *   ?       状态                H       帮助
 * ============================================================================
 */
#include <SimpleFOC.h>

/* ---------------- 引脚 ---------------- */
#define PIN_PWM1   PA8
#define PIN_PWM2   PA9
#define PIN_PWM3   PA10
#define PIN_EN     PB0

/* ---------------- 参数 ---------------- */
#define POLE_PAIRS      7
#define SUPPLY_VOLTAGE  24.0f
#define V_LIMIT_INIT    3.0f      /* 3.0V 约 0.79A；1.5V 力矩不足推不动 */
#define ALIGN_VOLTAGE   3.0f      /* ★ 对齐必须 >= 能推动转子的电压。
                                     SimpleFOC 的对齐要让转子转一整圈来测方向，
                                     1.5V 推不动 -> alignSensor() 失败 ->
                                     dir=UNKNOWN / zero_electric_angle=NOT_SET ->
                                     FOC 不知道电角度零点 -> 电机完全不动。 */
#define VEL_LIMIT_INIT  20.0f

/* ---------------- 对象 ---------------- */
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_PWM1, PIN_PWM2, PIN_PWM3, PIN_EN);

static float target_angle = 0.0f;

/* ======================= 命令 ======================= */
static char    cmd[40];
static uint8_t cmd_len = 0;

static float parse_float(const char *s)
{
    float v = 0.0f; int sign = 1; bool dec = false; float f = 0.1f;
    if (*s == '-') { sign = -1; s++; }
    while (*s) {
        if (*s == '.') { dec = true; s++; continue; }
        if (*s < '0' || *s > '9') break;
        if (!dec) v = v * 10.0f + (float)(*s - '0');
        else { v += (float)(*s - '0') * f; f *= 0.1f; }
        s++;
    }
    return sign * v;
}

static void print_status(void)
{
    Serial2.print(F("EN="));      Serial2.print(motor.enabled ? "ON" : "OFF");
    Serial2.print(F("  angle=")); Serial2.print(motor.shaft_angle, 4);
    Serial2.print(F("  vel="));   Serial2.print(motor.shaft_velocity, 4);
    Serial2.print(F("  tgt="));   Serial2.print(target_angle, 4);
    Serial2.print(F("  Vlim="));  Serial2.print(motor.voltage_limit, 2);
    Serial2.print(F("  Vbus="));  Serial2.print(SUPPLY_VOLTAGE, 1);
    Serial2.print(F("  dir="));   Serial2.print((int)motor.sensor_direction);
    Serial2.print(F("  zero="));  Serial2.println(motor.zero_electric_angle, 4);
}

static void run_command(char c, const char *arg)
{
    switch (c) {
    case 'E':
        motor.enable();
        Serial2.println(F("EN ON"));
        break;
    case 'D':
        motor.disable();
        Serial2.println(F("EN OFF (stop)"));
        break;
    case 'T':
        target_angle = parse_float(arg);
        Serial2.print(F("target=")); Serial2.println(target_angle, 4);
        break;
    case 'V': {
        float v = parse_float(arg);
        if (v <= 0.0f) v = V_LIMIT_INIT;
        if (v > 6.0f)  { v = 6.0f; Serial2.println(F("!! clamped to 6V")); }
        motor.voltage_limit = v;
        Serial2.print(F("Vlim=")); Serial2.print(v, 2);
        Serial2.print(F("V  (~")); Serial2.print(v / 3.8f, 2);
        Serial2.println(F("A)"));
        if (v > 3.0f)
            Serial2.println(F("!! warning: above 3V current may exceed rated 1.4A"));
        break;
    }
    case 'L':
        motor.velocity_limit = parse_float(arg);
        Serial2.print(F("Vellim=")); Serial2.println(motor.velocity_limit, 2);
        break;
    case 'R':
        sensor.update();
        Serial2.print(F("sensor angle=")); Serial2.print(sensor.getAngle(), 4);
        Serial2.print(F("  vel="));        Serial2.print(sensor.getVelocity(), 4);
        Serial2.print(F("  mech="));       Serial2.println(sensor.getMechanicalAngle(), 4);
        break;
    case '?':
        print_status();
        break;
    case 'H':
        Serial2.println(F("E D T<n> V<n> L<n> R ?"));
        break;
    default:
        Serial2.println(F("? for help"));
        break;
    }
}

static void poll_command(void)
{
    while (Serial2.available() > 0) {
        char c = (char)Serial2.read();
        if (c == '\r' || c == '\n') {
            if (cmd_len > 0) {
                cmd[cmd_len] = 0;
                run_command(cmd[0], cmd + 1);
                cmd_len = 0;
            }
        } else if (cmd_len < sizeof(cmd) - 1) {
            cmd[cmd_len++] = c;
        } else {
            cmd_len = 0;      /* 太长，丢弃 */
        }
    }
}

/* ======================= setup ======================= */
void setup()
{
    pinMode(PC13, OUTPUT);

    Serial2.begin(115200);
    delay(100);

    Serial2.println();
    Serial2.println(F("================================================"));
    Serial2.println(F(" SimpleFOC 2.4.0 / STM32F103 + DRV8316C + AS5600"));
    Serial2.println(F("================================================"));
    Serial2.println(F("PWM PA8/PA9/PA10   EN PB0   I2C PB6/PB7"));
    Serial2.print(F("SysClk=")); Serial2.println(SystemCoreClock);
    Serial2.println(F("UART: Serial2 = USART2 (PA2/PA3)"));
    Serial2.println(F("------------------------------------------------"));

    sensor.init();
    motor.linkSensor(&sensor);
    sensor.update();
    Serial2.print(F("[1] sensor AS5600 @0x36  angle="));
    Serial2.println(sensor.getAngle(), 4);

    driver.voltage_power_supply = SUPPLY_VOLTAGE;
    driver.voltage_limit = SUPPLY_VOLTAGE;
    driver.init();
    motor.linkDriver(&driver);
    Serial2.println(F("[2] driver OK"));

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

    Serial2.print(F("[3] Vlim=")); Serial2.print(motor.voltage_limit, 2);
    Serial2.print(F("V (~"));      Serial2.print(motor.voltage_limit / 3.8f, 2);
    Serial2.print(F("A)  Vellim=")); Serial2.println(motor.velocity_limit, 1);

    motor.init();
    Serial2.println(F("[4] motor.init OK"));

    Serial2.println(F("aligning (initFOC)..."));
    delay(1200);
    motor.initFOC();

    Serial2.print(F("aligned: dir="));
    Serial2.println(motor.sensor_direction == Direction::CW ? "CW" : "CCW");
    Serial2.print(F("         zero_electric_angle="));
    Serial2.println(motor.zero_electric_angle, 4);

    /* ★ 对齐后必须失能，等用户确认 */
    motor.disable();
    Serial2.println(F("motor DISABLED - send E to enable"));
    Serial2.println(F("commands: E D T<n> V<n> L<n> R ?"));
    Serial2.println(F("================================================"));
}

/* ======================= loop ======================= */
void loop()
{
    motor.loopFOC();
    motor.move(target_angle);

    poll_command();

    digitalWrite(PC13, (millis() / 400) % 2);
}
