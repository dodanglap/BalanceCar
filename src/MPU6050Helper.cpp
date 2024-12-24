#include "MPU6050Helper.h"

bool MPU6050Helper::begin() {
    if (!mpu.begin()) {
        return false; // Không khởi tạo được MPU6050
    }

    // Cấu hình bộ lọc và dải đo mặc định
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    return true;
}

void MPU6050Helper::getAllMPU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz, float &temp) {
    sensors_event_t accel, gyro, tempEvent;
    mpu.getEvent(&accel, &gyro, &tempEvent);

    ax = accel.acceleration.x;
    ay = accel.acceleration.y;
    az = accel.acceleration.z;

    gx = gyro.gyro.x;
    gy = gyro.gyro.y;
    gz = gyro.gyro.z;

    temp = tempEvent.temperature;
}
