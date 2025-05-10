#ifndef KINEMATICS_H
#define KINEMATICS_H
#include <Arduino.h>

typedef struct
{
    float per_pulse_distance;  /* 单个脉冲对应轮子前景距离 */
    int16_t motor_speed;       /* 电机速度 mm/s */
    int64_t last_encoder_tick; /* 上一次脉冲编码器读数 */
} motor_params_t;

/* 运动学解算类 */
class Kinematics
{
public:
    Kinematics() = default;
    ~Kinematics() = default;
    /* 设置电机参数，编号id和每个脉冲对应的轮子前进距离 */
    void set_motor_params(uint16_t id, float per_pulse_distance);
    /* 设置轮子间距 */
    void set_wheel_distance(float wheel_distance_a,float wheel_distance_b);
    /* 运动学逆解算，输入平移速度和转动速度，输出各个轮子速度 */
    void kinematic_inverse(float linear_x_speed, float linear_y_speed, float angular_speed,
                           float &out_wheel1_speed, float &out_wheel2_speed, float &out_wheel3_speed, float &out_wheel4_speed);
    /* 运动学正解算，输入各个轮子速度，输出平移速度和转动速度 */
    void kinematic_forward(float wheel1_speed, float wheel2_speed, float wheel3_speed, float wheel4_speed,
                           float &linear_x_speed, float &linear_y_speed, float &angular_speed);
    /* 电机速度和编码器读数更新 */
    void update_motor_speed(uint16_t current_time, int32_t motor_tick1, int32_t motor_tick2, int32_t motor_tick3, int32_t motor_tick4);
    /* 获取电机速度 */
    int64_t get_motor_speed(uint8_t id);

private:
    motor_params_t motor_params_[4]; /* 电机参数 */
    uint64_t last_update_time_;      /* 上一次更新时间 ms */
    float wheel_distance_a_and_b_;           /* 轮子间距 */
    float wheel_distance_a_;
    float wheel_distance_b_;
};

#endif // KINEMATICS_H