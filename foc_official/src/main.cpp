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
 *   E       使能并保持当前位置 D       失能 / 急停
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
#define POLE_PAIRS      11       /* 对齐实测估算约 11.28；官方 SimpleFOC Mini 例程也是 11 */
#define SUPPLY_VOLTAGE  24.0f
#define SKIP_SENSOR_ALIGNMENT    1
#define FIXED_SENSOR_ZERO        0.92f    /* 最近一次实机校准：0.9219 rad */
#define V_LIMIT_INIT    3.0f      /* 3.0V 约 0.79A；1.5V 力矩不足推不动 */
#define ALIGN_VOLTAGE   3.0f      /* ★ 对齐必须 >= 能推动转子的电压。
                                     SimpleFOC 的对齐要让转子转一整圈来测方向，
                                     1.5V 推不动 -> alignSensor() 失败 ->
                                     dir=UNKNOWN / zero_electric_angle=NOT_SET ->
                                     FOC 不知道电角度零点 -> 电机完全不动。 */
#define VEL_LIMIT_INIT  20.0f
#define VELOCITY_LPF_TF 0.05f     /* 先只增强速度反馈滤波，压低 AS5600 差分噪声 */

/* 上电一次性自检动作：开环正向旋转一整圈后自动失能。 */
#define STARTUP_ROTATE_ONCE       1
#define STARTUP_OPENLOOP_VEL      4.0f     /* 开环起动速度 */
#define TARGET_SLEW_VEL            4.0f     /* 普通位置命令的速度斜坡 */
#define STARTUP_ROTATE_TOLERANCE  0.03f    /* rad */
#define STARTUP_ROTATE_TIMEOUT_MS 10000UL

/* ---------------- 对象 ---------------- */
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_PWM1, PIN_PWM2, PIN_PWM3, PIN_EN);

static float target_angle = 0.0f;
static float control_target_angle = 0.0f;
static uint32_t control_target_ts = 0;
static bool startup_rotate_active = false;
static float startup_rotate_target = 0.0f;
static float startup_saved_velocity_limit = VEL_LIMIT_INIT;
static MotionControlType startup_saved_controller = MotionControlType::angle;
static float startup_saved_pid_p = 0.2f;
static float startup_saved_pid_i = 5.0f;
static uint32_t startup_rotate_started_ms = 0;

static void restore_after_startup_rotation()
{
    motor.controller = startup_saved_controller;
    motor.velocity_limit = startup_saved_velocity_limit;
    motor.PID_velocity.P = startup_saved_pid_p;
    motor.PID_velocity.I = startup_saved_pid_i;
}

static float slew_control_target(float requested)
{
    const uint32_t now = micros();
    float dt = (now - control_target_ts) * 1e-6f;
    control_target_ts = now;
    if (dt <= 0.0f || dt > 0.05f) dt = 0.001f;

    const float max_step = TARGET_SLEW_VEL * dt;
    const float error = requested - control_target_angle;
    if (fabsf(error) <= max_step)
        control_target_angle = requested;
    else
        control_target_angle += (error > 0.0f) ? max_step : -max_step;
    return control_target_angle;
}

static void cancel_startup_rotation(const __FlashStringHelper *message)
{
    if (!startup_rotate_active) return;
    startup_rotate_active = false;
    restore_after_startup_rotation();
    sensor.update();
    target_angle = motor.shaftAngle();
    control_target_angle = target_angle;
    Serial2.println(message);
}

static void start_startup_rotation()
{
    sensor.update();
    /* 目标必须使用 motor.shaft_angle 坐标：它包含 sensor_direction 和
       sensor_offset。直接拿 sensor.getAngle() 会引入零点偏差，表现为
       上电只在零点附近来回摆动，而不是完整旋转一圈。 */
    motor.shaft_angle = motor.shaftAngle();
    const float start_angle = motor.shaft_angle;
    startup_rotate_target = start_angle + 6.28318530718f;
    target_angle = startup_rotate_target;
    startup_saved_controller = motor.controller;
    startup_saved_velocity_limit = motor.velocity_limit;
    startup_saved_pid_p = motor.PID_velocity.P;
    startup_saved_pid_i = motor.PID_velocity.I;
    /* 整圈期间使用开环速度控制，避开静止位置环的起动死点；
       完成后恢复正常位置闭环参数。 */
    motor.controller = MotionControlType::velocity_openloop;
    motor.velocity_limit = VEL_LIMIT_INIT;
    motor.PID_velocity.P = 0.2f;
    motor.PID_velocity.I = 5.0f;
    startup_rotate_started_ms = millis();
    startup_rotate_active = true;
    motor.enable();
    Serial2.print(F("startup rotate +360deg, target="));
    Serial2.println(startup_rotate_target, 4);
}

static void service_startup_rotation()
{
    if (!startup_rotate_active) return;

    sensor.update();
    const float measured_angle = motor.shaftAngle();
    const float position_error = startup_rotate_target - measured_angle;
    if (position_error <= STARTUP_ROTATE_TOLERANCE) {
        startup_rotate_active = false;
        target_angle = measured_angle;
        control_target_angle = target_angle;
        motor.disable();
        restore_after_startup_rotation();
        Serial2.println(F("startup rotate DONE (+360deg), motor DISABLED"));
        return;
    }

    if (millis() - startup_rotate_started_ms >= STARTUP_ROTATE_TIMEOUT_MS) {
        startup_rotate_active = false;
        target_angle = measured_angle;
        control_target_angle = target_angle;
        motor.disable();
        restore_after_startup_rotation();
        Serial2.println(F("startup rotate TIMEOUT, motor DISABLED"));
    }
}

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
    Serial2.print(F("  pp="));    Serial2.print(motor.pp_check_result ? "OK" : "FAIL");
    Serial2.print(F("  zero="));  Serial2.println(motor.zero_electric_angle, 4);
}

static void run_command(char c, const char *arg)
{
    switch (c) {
    case 'E':
        /* 从失能进入使能时，先锁定当前位置，避免残留目标造成
           位置阶跃、瞬时大电流和明显嗡鸣。用户随后再发送 T<n> 移动。 */
        if (!motor.enabled) {
            sensor.update();
            target_angle = motor.shaftAngle();
            control_target_angle = target_angle;
            motor.enable();
            Serial2.print(F("EN ON  target=")); Serial2.println(target_angle, 4);
        } else {
            /* enable() 会重置 PID；重复 E 不应重新触发这个过程。 */
            /* 保留 `EN ON` 前缀，兼容前端的状态解析。 */
            Serial2.println(F("EN ON (already enabled)"));
        }
        break;
    case 'D':
        cancel_startup_rotation(F("startup rotate ABORTED"));
        motor.disable();
        Serial2.println(F("EN OFF (stop)"));
        break;
    case 'T':
        cancel_startup_rotation(F("startup rotate ABORTED by target command"));
        target_angle = parse_float(arg);
        Serial2.print(F("target=")); Serial2.println(target_angle, 4);
        break;
    case 'V': {
        float v = parse_float(arg);
        if (v <= 0.0f) v = V_LIMIT_INIT;
        if (v > 5.0f)  { v = 5.0f; Serial2.println(F("!! clamped to 5V")); }
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

    /* 与工作区官方 angle_control.ino 一致：较小的速度环增益可避免
       AS5600 微分速度噪声被放大成静止时的高频抖动/嗡鸣。 */
    motor.PID_velocity.P = 0.2f;
    motor.PID_velocity.I = 5.0f;
    motor.PID_velocity.D = 0.0f;
    motor.PID_velocity.output_ramp = 1000.0f;
    motor.LPF_velocity.Tf = VELOCITY_LPF_TF;
    motor.P_angle.P = 20.0f;

    Serial2.print(F("[3] Vlim=")); Serial2.print(motor.voltage_limit, 2);
    Serial2.print(F("V (~"));      Serial2.print(motor.voltage_limit / 3.8f, 2);
    Serial2.print(F("A)  Vellim=")); Serial2.println(motor.velocity_limit, 1);

    motor.init();
    Serial2.println(F("[4] motor.init OK"));

    Serial2.println(F("aligning (initFOC)..."));
#if SKIP_SENSOR_ALIGNMENT
    /* motor.init() 为兼容官方流程会短暂使能驱动；固定 AS5600 校准时
       不需要输出对齐电压，立即失能可消除这段无意义的保持噪声。 */
    motor.disable();
    delay(50);
#else
    delay(1200);
#endif
#if SKIP_SENSOR_ALIGNMENT
    /* AS5600 提供绝对机械角；方向和电角度偏置固定后跳过每次上电的
       正反扫角校准，消除初始化阶段的右/左摆动和噪声。 */
    motor.sensor_direction = Direction::CW;
    motor.zero_electric_angle = FIXED_SENSOR_ZERO;
    motor.pp_check_result = true;
#endif
    motor.initFOC();

    /* 上电后默认执行一次开环 +360° 自检；完成后自动失能。 */
    target_angle = motor.shaft_angle;
    control_target_angle = target_angle;
    control_target_ts = micros();

    Serial2.print(F("aligned: dir="));
    Serial2.println(motor.sensor_direction == Direction::CW ? "CW" : "CCW");
    Serial2.print(F("         zero_electric_angle="));
    Serial2.println(motor.zero_electric_angle, 4);

#if STARTUP_ROTATE_ONCE
    start_startup_rotation();
#else
    motor.disable();
    Serial2.println(F("motor DISABLED - send E to enable"));
#endif
    Serial2.println(F("commands: E D T<n> V<n> L<n> R ?"));
    Serial2.println(F("================================================"));
}

/* ======================= loop ======================= */
void loop()
{
    motor.loopFOC();
    if (startup_rotate_active)
        motor.move(STARTUP_OPENLOOP_VEL);
    else
        motor.move(slew_control_target(target_angle));
    service_startup_rotation();

    poll_command();

    digitalWrite(PC13, (millis() / 400) % 2);
}
