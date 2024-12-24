#ifndef MPU6050HELPER_H
#define MPU6050HELPER_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

class MPU6050Helper {
private:
    Adafruit_MPU6050 mpu;

public:
    // Khởi tạo và cấu hình
    bool begin();

    // Đọc dữ liệu từ cảm biến
    void getAllMPU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz, float &temp);
};

#endif
