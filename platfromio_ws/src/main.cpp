#include <Arduino.h>
#include <Wire.h>

#include "MPU6050_light.h"
#include <Esp32McpwmMotor.h>
#include "imu.h"
#include "Config.h"
#include <Esp32PcntEncoder.h>
#include <PidController.h>
#include<Kinematics.h>

Esp32PcntEncoder encoders[4];    // 编码器
IMU imu;                         // imu
Esp32McpwmMotor motor;           // 电机
PidController pid_controller[4]; // 四个电机的pid控制器
Kinematics kinematics;           // 运动学模型

int64_t last_ticks[4];    // 上一次读取的计数器数值
int32_t delta_tick[4];    // 两次读取之间的计数器差值
int64_t last_update_time; // 上一次更新时间
float current_speeds[4];  // 四个电机的速度

float target_linear_x_speed = 100.0; // 目标x线速度 ms/s
float target_linear_y_speed = 0.0;  // 目标y线速度 ms/s
float target_angular_speed = 1.f;   // 目标角速度 rad/s
float out_speed[4];                 // 电机输出速度

void motor_speed_control()
{
    uint64_t dt = millis() - last_update_time;
    for (int i = 0; i < 4; i++)
    {
        // 计算编码器差值
        delta_tick[i] = encoders[i].getTicks() - last_ticks[i];

        // 计算电机速度  m/s->mm/s
        current_speeds[i] = (float)(delta_tick[i] * 0.1051566) / (float)dt * 1000.0;

        // 更新上一次计数数值
        last_ticks[i] = encoders[i].getTicks();

        motor.updateMotorSpeed(i, pid_controller[i].update(current_speeds[i]));
    }

    last_update_time = millis(); // 更新上一次更新时间

    Serial.printf("speeds: 1=%fm/s ,2=%fm/s ,3=%fm/s ,4=%fm/s  \n",
                  current_speeds[0], current_speeds[1], current_speeds[2], current_speeds[3]);
}

void setup()
{
    Serial.begin(115200);
    // imu.IMUInit();

    // 设置电机
    motor.attachMotor(MOTOR_ID0, MOTOR_ID0_GPIO_IN1, MOTOR_ID0_GPIO_IN2);
    motor.attachMotor(MOTOR_ID1, MOTOR_ID1_GPIO_IN1, MOTOR_ID1_GPIO_IN2);
    motor.attachMotor(MOTOR_ID2, MOTOR_ID2_GPIO_IN1, MOTOR_ID2_GPIO_IN2);
    motor.attachMotor(MOTOR_ID3, MOTOR_ID3_GPIO_IN1, MOTOR_ID3_GPIO_IN2);
    // 设置编码器
    encoders[0].init(ENCODER_PCNT_UINT0, ENCODER_PCNT_UINT0_PINA, ENCODER_PCNT_UINT0_PINB);
    encoders[1].init(ENCODER_PCNT_UINT1, ENCODER_PCNT_UINT1_PINA, ENCODER_PCNT_UINT1_PINB);
    encoders[2].init(ENCODER_PCNT_UINT2, ENCODER_PCNT_UINT2_PINA, ENCODER_PCNT_UINT2_PINB);
    encoders[3].init(ENCODER_PCNT_UINT3, ENCODER_PCNT_UINT3_PINA, ENCODER_PCNT_UINT3_PINB);

    // 设置pid控制器
    for (int i = 0; i < 4; i++)
    {
        pid_controller[i].update_pid(0.625, 0.125, 0.0);
        pid_controller[i].out_limit(-100, 100);
    }

    kinematics.set_wheel_distance(216.f,177.f);
    kinematics.set_motor_params(0,0.1051566);
    kinematics.set_motor_params(1,0.1051566);
    kinematics.set_motor_params(2,0.1051566);
    kinematics.set_motor_params(3,0.1051566);

    kinematics.kinematic_inverse(target_linear_x_speed,target_linear_y_speed,target_angular_speed,out_speed[0],out_speed[1],out_speed[2],out_speed[3]);

    // 设置电机速度
    for (int i = 0; i < 4; i++)
    {
        // mm/s
        pid_controller[i].update_target(out_speed[i]);
    }
}

void loop()
{

    delay(10);
    motor_speed_control();
}