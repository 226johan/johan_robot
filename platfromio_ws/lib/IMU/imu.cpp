#include "imu.h"

void IMU::IMUInit()
{
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    status_ = mpu_.begin(); // 启动MPU6050并获取状态
    Serial.print(F("MPU6050 status: "));
    Serial.print(status_);
    while (status_ != 0)
    {
    }
    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    mpu_.calcOffsets(); // 计算偏移量
    Serial.println("Done!\n");
}

void IMU::IMUUpdate()
{
    mpu_.update();
    if (millis() - timer_ > 10)
    {
        Serial.print("X: ");
        Serial.print(mpu_.getAngleX());
        Serial.print("\tY: ");
        Serial.print(mpu_.getAngleY());
        Serial.print("\tZ: ");
        Serial.println(mpu_.getAngleZ());
        timer_ = millis();
    }
}