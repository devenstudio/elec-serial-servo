/*
 * TT 电机霍尔编码器闭环控制
 * 功能: 每秒切换目标转速, 10 秒循环
 * 硬件: LGT8F328P + TT 电机(13线霍尔编码器, 1:48减速) + L298N/TB6612
 *
 * 电机参数:
 *   减速比: 1:48
 *   编码器线数: 13
 *   输出轴单相脉冲: 13 * 48 = 624
 *   CHANGE中断(2倍频): 624 * 2 = 1248
 *   满速 300rpm = 5圈/秒 * 1248 = 6240 脉冲/秒
 *
 * 转速定义: 编码器脉冲数/秒
 * 正数 = 正转, 负数 = 反转, 0 = 停止
 */

#include <Arduino.h>

// ==================== 引脚定义 ====================
#define ENCODER_A       2      // 霍尔 A 相 (外部中断)
#define ENCODER_B       3      // 霍尔 B 相
#define MOTOR_PWM       9      // 电机 PWM
#define MOTOR_DIR_A     7      // 方向 A
#define MOTOR_DIR_B     8      // 方向 B

// ==================== 编码器参数 ====================
// 13线 * 48减速比 = 624 (单相)
// CHANGE中断 = 2倍频 -> 624 * 2 = 1248
#define PULSES_PER_OUTPUT_REV  1248

// ==================== 测速参数 ====================
#define SPEED_WINDOW_MS   50   // 测速窗口 50ms
#define PID_INTERVAL      20   // PID 计算周期 20ms

// ==================== PID 参数 ====================
// 转速量级 ~6000, 输出限幅 ±255
float Kp = 0.05;
float Ki = 0.3;
float Kd = 0.0;
float integral = 0;
float lastError = 0;
unsigned long lastPidTime = 0;

// ==================== 目标转速表 ====================
// 每秒切换一次, 10 秒循环
// 单位: 脉冲/秒 (满速 6240)
const int TARGET_SPEEDS[10] = {
    1500,    // 0s: 正转低速 (~72rpm)
    3000,    // 1s: 正转中速 (~144rpm)
    6000,    // 2s: 正转高速 (~288rpm)
    0,       // 3s: 停止
    -1500,   // 4s: 反转低速
    -3000,   // 5s: 反转中速
    -6000,   // 6s: 反转高速
    0,       // 7s: 停止
    4500,    // 8s: 正转中高速
    -4500    // 9s: 反转中高速
};

const unsigned long STEP_INTERVAL = 1000;
const int STEP_COUNT = 10;

// ==================== 全局变量 ====================
volatile long encoderCount = 0;
long lastEncoderCount = 0;
float currentSpeed = 0;
int targetSpeed = 0;

unsigned long lastSpeedTime = 0;
unsigned long lastStepTime = 0;
int currentStep = 0;

// ==================== 编码器中断 ====================
void encoderISR() {
    int a = digitalRead(ENCODER_A);
    int b = digitalRead(ENCODER_B);
    if (a == b) encoderCount--;
    else encoderCount++;
}

// ==================== 电机控制 ====================
void setMotor(int pwm, int dir) {
    pwm = constrain(pwm, 0, 255);
    if (dir > 0) {
        digitalWrite(MOTOR_DIR_A, HIGH);
        digitalWrite(MOTOR_DIR_B, LOW);
        analogWrite(MOTOR_PWM, pwm);
    } else if (dir < 0) {
        digitalWrite(MOTOR_DIR_A, LOW);
        digitalWrite(MOTOR_DIR_B, HIGH);
        analogWrite(MOTOR_PWM, pwm);
    } else {
        digitalWrite(MOTOR_DIR_A, LOW);
        digitalWrite(MOTOR_DIR_B, LOW);
        analogWrite(MOTOR_PWM, 0);
    }
}

// ==================== 测速 ====================
void updateSpeed() {
    unsigned long now = millis();
    if (now - lastSpeedTime >= SPEED_WINDOW_MS) {
        noInterrupts();
        long count = encoderCount;
        interrupts();

        long delta = count - lastEncoderCount;
        lastEncoderCount = count;

        float dt = (now - lastSpeedTime) / 1000.0;
        currentSpeed = delta / dt;

        lastSpeedTime = now;
    }
}

// ==================== PID ====================
float computePid(float setpoint, float measured) {
    unsigned long now = millis();
    float dt = (now - lastPidTime) / 1000.0;
    if (dt < 0.001) dt = 0.001;
    lastPidTime = now;

    float error = setpoint - measured;
    integral += error * dt;
    if (integral > 2000) integral = 2000;
    if (integral < -2000) integral = -2000;

    float derivative = (error - lastError) / dt;
    lastError = error;

    float output = Kp * error + Ki * integral + Kd * derivative;
    if (output > 255) output = 255;
    if (output < -255) output = -255;
    return output;
}

// ==================== setup ====================
void setup() {
    Serial.begin(115200);

    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);

    pinMode(MOTOR_PWM, OUTPUT);
    pinMode(MOTOR_DIR_A, OUTPUT);
    pinMode(MOTOR_DIR_B, OUTPUT);

    setMotor(0, 0);

    lastSpeedTime = millis();
    lastPidTime = millis();
    lastStepTime = millis();

    Serial.println(F("TT Motor Encoder Closed-Loop"));
    Serial.println(F("Step | Target | Current | PWM | Dir"));
}

// ==================== loop ====================
void loop() {
    unsigned long now = millis();

    // ---- 1. 每秒切换目标转速 ----
    if (now - lastStepTime >= STEP_INTERVAL) {
        lastStepTime = now;
        currentStep = (currentStep + 1) % STEP_COUNT;
        targetSpeed = TARGET_SPEEDS[currentStep];

        // 切换目标时重置积分, 避免残留
        integral = 0;

        Serial.print(F("--- Step "));
        Serial.print(currentStep);
        Serial.print(F(" Target: "));
        Serial.println(targetSpeed);
    }

    // ---- 2. 测速 ----
    updateSpeed();

    // ---- 3. PID 闭环 ----
    if (now - lastPidTime >= PID_INTERVAL) {
        float output = computePid(targetSpeed, currentSpeed);

        int dir = 0;
        int pwm = 0;
        if (output > 3) { dir = 1; pwm = (int)output; }
        else if (output < -3) { dir = -1; pwm = (int)(-output); }
        else { dir = 0; pwm = 0; }

        setMotor(pwm, dir);

        // ---- 4. 调试输出 (每 200ms) ----
        static unsigned long lastPrint = 0;
        if (now - lastPrint >= 200) {
            lastPrint = now;
            Serial.print(F("Step "));
            Serial.print(currentStep);
            Serial.print(F(" | T: "));
            Serial.print(targetSpeed);
            Serial.print(F(" | C: "));
            Serial.print(currentSpeed, 0);
            Serial.print(F(" | PWM: "));
            Serial.print(pwm);
            Serial.print(F(" Dir: "));
            Serial.println(dir);
        }
    }
}