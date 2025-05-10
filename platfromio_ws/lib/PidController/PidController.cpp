#include "PidController.h"
#include "Arduino.h"

PidController::PidController(float kp, float ki, float kd)
{
    reset();                // 初始化pid控制器
    update_pid(kp, ki, kd); // 更新pid系数
}

// 提供当前值返回下次输出值
float PidController::update(float current)
{
    // 计算误差 和 误差变化率
    float error = target_-current;  // 计算误差   (目标-当前)
    derror_ = error_last_-error;  // 计算误差变化率 (上次误差-当前误差)
    error_last_ = error;          // 更新上次误差为当前误差

    // 计算积分想并进行积分限制
    error_sum_ += error;           // 累加误差
    if(error_sum_>intergral_up_){
        error_sum_ = intergral_up_;  // 积分上限
    }
    if(error_sum_<-1*intergral_up_){
        error_sum_ = -1*intergral_up_;  // 积分下限
    }

    // 计算输出值
    float output = kp_*error + ki_*error_sum_ + kd_*derror_;

    //输出控制限幅
    if(output>out_max_){
        output = out_max_;
    }
    if(output<out_min_){
        output = out_min_;
    }

    return output;
}

// 更新目标值
void PidController::update_target(float target)
{
    target_ = target;
}

// 更新pid系数
void PidController::update_pid(double kp, double ki, double kd)
{
    reset();                // 初始化pid控制器
    kp_ = kp;               // 更新比例系数
    ki_ = ki;               // 更新积分系数
    kd_ = kd;               // 更新微分系数
}

// 重置pid控制器
void PidController::reset()
{
    target_ = 0;           // 目标值
    out_min_ = 0.0f;       // 输出下限
    out_max_ = 0.0f;       // 输出上限
    kp_ = 0.0f;            // 比例系数
    ki_ = 0.0f;            // 积分系数
    kd_ = 0.0f;            // 微分系数
    error_sum_ = 0.0f;     // 误差累加和
    derror_ = 0.0f;        // 误差变化率
    error_last_ = 0.0f;    // 上次误差
}

// 设置输出限制
void PidController::out_limit(float out_min, float out_max)
{
    out_min_ = out_min;   // 输出下限
    out_max_ = out_max;   // 输出上限
}
