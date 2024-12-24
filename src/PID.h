#ifndef PID_H
#define PID_H

class PID {
private:
    float Kp;       // Hệ số P (Proportional)
    float Ki;       // Hệ số I (Integral)
    float Kd;       // Hệ số D (Derivative)

    float setpoint; // Giá trị mục tiêu

    float error;        // Sai số hiện tại
    float prev_error;   // Sai số trước đó
    float integral;     // Tích phân sai số
    float derivative;   // Đạo hàm sai số

    float output;       // Tín hiệu điều khiển cuối cùng

    unsigned long last_time; // Thời điểm tính toán cuối cùng (ms)

public:
    // Constructor
    PID(float kp, float ki, float kd, float sp = 0.0);

    // Cập nhật hệ số PID
    void setTunings(float kp, float ki, float kd);

    // Cập nhật giá trị mục tiêu
    void setSetpoint(float sp);

    // Đọc giá trị mục tiêu hiện tại
    float getSetpoint();

    // Hàm tính toán PID
    float compute(float input, float dt);

    // Đặt lại giá trị tích phân và đạo hàm
    void reset();

    // Giới hạn giá trị đầu ra
    float constrainOutput(float min_val, float max_val);

    // Đọc giá trị đầu ra hiện tại
    float getOutput();
};

#endif