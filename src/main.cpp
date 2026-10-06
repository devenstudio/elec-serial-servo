/*
 * TT 电机霍尔编码器闭环控制 - 串口调试版
 *
 * 修正:
 *   1. 滤波系数从 0.7/0.3 改成 0.3/0.7, 减少滞后
 *   2. 去掉 PID 输出到死区的强制映射
 *   3. 死区补偿只在启动时施加
 *   4. Ki 从 0.08 加大到 0.15
 */

#include <Arduino.h>

// ==================== 引脚定义 ====================
#define ENCODER_A       2
#define ENCODER_B       3
#define MOTOR_IN1       7
#define MOTOR_IN2       8

// ==================== 编码器参数 ====================
#define PULSES_PER_OUTPUT_REV  624

// ==================== 测速参数 ====================
#define SPEED_WINDOW_MS   50
#define PID_INTERVAL      50

// ==================== PID 参数 ====================
float Kp = 0.04;
float Ki = 0.15;
float Kd = 0.0;
float integral = 0;
float lastError = 0;
unsigned long lastPidTime = 0;

// ==================== 死区补偿 ====================
#define MOTOR_DEADZONE  130

// ==================== 方向锁定阈值 ====================
#define DIR_LOCK_THRESHOLD  500

// ==================== 全局变量 ====================
volatile long encoderCount = 0;
long lastEncoderCount = 0;
float currentSpeed = 0;
int targetSpeed = 0;
unsigned long lastSpeedTime = 0;

String inputString = "";
bool stringComplete = false;

// ==================== 编码器中断 ====================
void encoderISR() {
    int b = digitalRead(ENCODER_B);
    if (b == LOW) encoderCount++;
    else encoderCount--;
}

// ==================== 电机控制 ====================
void setMotor(int pwm, int dir) {
    pwm = constrain(pwm, 0, 255);
    if (dir > 0) {
        analogWrite(MOTOR_IN1, pwm);
        digitalWrite(MOTOR_IN2, LOW);
    } else if (dir < 0) {
        digitalWrite(MOTOR_IN1, LOW);
        analogWrite(MOTOR_IN2, pwm);
    } else {
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, LOW);
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
        float rawSpeed = delta / dt;
        // 滤波系数: 新值权重 0.7, 旧值权重 0.3
        currentSpeed = 0.3 * currentSpeed + 0.7 * rawSpeed;
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
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);

    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    setMotor(0, 0);

    lastSpeedTime = millis();
    lastPidTime = millis();

    Serial.println(F("TT Motor PID Tuning"));
    Serial.println(F("Input target speed: 1500 / -1500 / 0"));
    inputString.reserve(16);
}

// ==================== loop ====================
void loop() {
    unsigned long now = millis();

    // ---- 串口读取 ----
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '"') continue;
        if (c == '\n' || c == '\r') {
            if (inputString.length() > 0) stringComplete = true;
        } else {
            inputString += c;
        }
    }

    if (stringComplete) {
        int val = inputString.toInt();
        targetSpeed = val;
        integral = 0;
        lastError = 0;
        inputString = "";
        stringComplete = false;
        Serial.print(F(">>> New Target: "));
        Serial.println(targetSpeed);
    }

    // ---- 测速 ----
    updateSpeed();

    // ---- PID ----
    if (now - lastPidTime >= PID_INTERVAL) {
        if (targetSpeed == 0) {
            integral = 0;
            lastError = 0;
        }

        float output = computePid(targetSpeed, currentSpeed);

        // 死区补偿: 只在电机几乎停住时施加
        if (targetSpeed != 0 && abs(currentSpeed) < 100) {
            if (output > 0 && output < MOTOR_DEADZONE) output = MOTOR_DEADZONE;
            if (output < 0 && output > -MOTOR_DEADZONE) output = -MOTOR_DEADZONE;
        }

        int dir = 0;
        int pwm = 0;

        if (targetSpeed > DIR_LOCK_THRESHOLD) {
            dir = 1;
            pwm = (int)abs(output);
        } else if (targetSpeed < -DIR_LOCK_THRESHOLD) {
            dir = -1;
            pwm = (int)abs(output);
        } else {
            if (output > 3) { dir = 1; pwm = (int)output; }
            else if (output < -3) { dir = -1; pwm = (int)(-output); }
            else { dir = 0; pwm = 0; }
        }

        if (targetSpeed == 0 && abs(currentSpeed) < 50) {
            setMotor(0, 0);
            pwm = 0;
            dir = 0;
        } else {
            setMotor(pwm, dir);
        }

        // ---- 调试输出 (每 200ms) ----
        static unsigned long lastPrint = 0;
        if (now - lastPrint >= 200) {
            lastPrint = now;
            Serial.print(F("T: "));
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