#include "PID.h"
#include <Arduino.h>

PID::PID(float kp, float ki, float kd, float sp) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
    setpoint = sp;

    error = 0.0;
    prev_error = 0.0;
    integral = 0.0;
    derivative = 0.0;
    output = 0.0;

}

void PID::setTunings(float kp, float ki, float kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
}

void PID::setSetpoint(float sp) {
    setpoint = sp;
}

float PID::getSetpoint() {
    return setpoint;
}

float PID::compute(float input, float dt) {

    if (dt <= 0.0) dt = 0.001; // Tránh chia cho 0

    // Tính toán sai số
    error = setpoint - input;

    // Tính tích phân
    integral += error * dt;

    // Tính đạo hàm
    derivative = (error - prev_error) / dt;

    // Tính tín hiệu điều khiển
    output = Kp * error + Ki * integral + Kd * derivative;

    // Lưu trạng thái hiện tại cho lần lặp sau
    prev_error = error;
    return output;
}

void PID::reset() {
    integral = 0.0;
    derivative = 0.0;
    prev_error = 0.0;
}

float PID::constrainOutput(float min_val, float max_val) {
    if (output > max_val) output = max_val;
    if (output < min_val) output = min_val;
    return output;
}

float PID::getOutput() {
    return output;
}
