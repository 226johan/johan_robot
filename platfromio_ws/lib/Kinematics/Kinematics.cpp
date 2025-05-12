#include "Kinematics.h"

/* 设置电机参数，编号id和每个脉冲对应的轮子前进距离 */
void Kinematics::set_motor_params(uint16_t id, float per_pulse_distance)
{
    motor_params_[id].per_pulse_distance = per_pulse_distance;
}

/* 设置轮子间距 */
void Kinematics::set_wheel_distance(float wheel_distance_a, float wheel_distance_b)
{
    wheel_distance_a_ = wheel_distance_a;
    wheel_distance_b_ = wheel_distance_b;
    wheel_distance_a_and_b_ = (wheel_distance_a_ + wheel_distance_b_) / 2;
}

/* 运动学逆解算，输入平移速度和转动速度，输出各个轮子速度 */
void Kinematics::kinematic_inverse(float linear_x_speed, float linear_y_speed, float angular_speed,
                                   float &out_wheel1_speed, float &out_wheel2_speed, float &out_wheel3_speed, float &out_wheel4_speed)
{
    out_wheel1_speed = linear_x_speed - linear_y_speed - (angular_speed * wheel_distance_a_and_b_);
    out_wheel2_speed = linear_x_speed + linear_y_speed + (angular_speed * wheel_distance_a_and_b_);
    out_wheel3_speed = linear_x_speed + linear_y_speed - (angular_speed * wheel_distance_a_and_b_);
    out_wheel4_speed = linear_x_speed - linear_y_speed + (angular_speed * wheel_distance_a_and_b_);
}

/* 运动学正解算，输入各个轮子速度，输出平移速度和转动速度 */
void Kinematics::kinematic_forward(float wheel1_speed, float wheel2_speed, float wheel3_speed, float wheel4_speed,
                                   float &linear_x_speed, float &linear_y_speed, float &angular_speed)
{
    linear_x_speed = (wheel1_speed + wheel2_speed + wheel3_speed + wheel4_speed) / 4.0f;
    linear_y_speed = (-wheel1_speed + wheel2_speed + wheel3_speed - wheel4_speed) / 4.0f;
    angular_speed = (-wheel1_speed + wheel2_speed - wheel3_speed + wheel4_speed) / (wheel_distance_a_and_b_ * 4.0f);
}

/* 电机速度和编码器读数更新 */
void Kinematics::update_motor_speed(uint16_t current_time, int32_t motor_tick1, int32_t motor_tick2, int32_t motor_tick3, int32_t motor_tick4)
{
    uint32_t dt = current_time - last_update_time_;
    last_update_time_ = current_time;

    int32_t dtick1 = motor_tick1 - motor_params_[0].last_encoder_tick;
    int32_t dtick2 = motor_tick2 - motor_params_[1].last_encoder_tick;
    int32_t dtick3 = motor_tick3 - motor_params_[2].last_encoder_tick;
    int32_t dtick4 = motor_tick4 - motor_params_[3].last_encoder_tick;
    motor_params_[0].last_encoder_tick = motor_tick1;
    motor_params_[1].last_encoder_tick = motor_tick2;
    motor_params_[2].last_encoder_tick = motor_tick3;
    motor_params_[3].last_encoder_tick = motor_tick4;

    motor_params_[0].motor_speed = float(dtick1 * motor_params_[0].per_pulse_distance) / dt * 1000;
    motor_params_[1].motor_speed = float(dtick2 * motor_params_[1].per_pulse_distance) / dt * 1000;
    motor_params_[2].motor_speed = float(dtick3 * motor_params_[2].per_pulse_distance) / dt * 1000;
    motor_params_[3].motor_speed = float(dtick4 * motor_params_[3].per_pulse_distance) / dt * 1000;

    update_odom(dt);
}

/* 获取电机速度 */
int64_t Kinematics::get_motor_speed(uint8_t id)
{
    return motor_params_[id].motor_speed;
}

/* 更新里程计数据 */
void Kinematics::update_odom(uint16_t dt)
{
    static float linear_x_speed, linear_y_speed, angular_speed;
    // printf("dt: %d\n", dt);
    float dt_s = (float)(((float)dt / 1000.f) / 1000.f);
    // printf("dt_s: %f\n", dt_s);
    this->kinematic_forward(motor_params_[0].motor_speed,
                            motor_params_[1].motor_speed,
                            motor_params_[2].motor_speed,
                            motor_params_[3].motor_speed,
                            linear_x_speed,
                            linear_y_speed,
                            angular_speed);

    odom_.angular_speed = angular_speed;
    odom_.linear_x_speed = linear_x_speed / 1000; // mm/s -> m/s
    odom_.linear_y_speed = linear_y_speed / 1000; // mm/s -> m/s

    odom_.x += odom_.linear_x_speed * cos(odom_.yaw) * dt_s + odom_.linear_y_speed * sin(odom_.yaw) * dt_s;
    odom_.y += odom_.linear_x_speed * sin(odom_.yaw) * dt_s + odom_.linear_y_speed * cos(odom_.yaw) * dt_s;
    odom_.yaw += odom_.angular_speed * dt_s;
    Kinematics::TransAngleInPI(odom_.yaw, odom_.yaw);

    // Serial.printf("odom(%f,%f)\n", odom_.x, odom_.y);
}

/* 获取里程计数据 */
odom_t &Kinematics::get_odom()
{
    return odom_;
}

/* 用于将角度转换到 -π 到 π 的范围内 */
void Kinematics::TransAngleInPI(float angle, float &out_angle)
{
    // 如果 angle 大于 π，则将 out_angle 减去 2π
    if (angle > PI)
    {
        out_angle -= 2 * PI;
    }
    // 如果 angle 小于 -π，则将 out_angle 加上 2π
    else if (angle < -PI)
    {
        out_angle += 2 * PI;
    }
}