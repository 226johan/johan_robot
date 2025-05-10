#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include "MPU6050_light.h"

#define I2C_SDA_PIN 12
#define I2C_SCL_PIN 13

class IMU
{
public:
    IMU() = default;
    void IMUInit();
    void IMUUpdate();

private:
    MPU6050 mpu_{Wire};
    unsigned long timer_ = 0; // 计时器
    byte status_;             // IMU状态
};

#endif // IMU_H