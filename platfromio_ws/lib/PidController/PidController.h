#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PidController
{
public:
    PidController() = default;
    PidController(float kp, float ki, float kd);
    ~PidController(){};

public:
    float target_;  // 目标值
    float out_min_; // 输出下限
    float out_max_; // 输出上限
    float kp_;      // 比例系数
    float ki_;      // 积分系数
    float kd_;      // 微分系数

    // pid
    float error_sum_;           // 误差累加和
    float derror_;              // 误差变化率
    float error_last_;          // 上次误差
    float error_pre_;           // 上上次误差
    float intergral_up_ = 2500; // 积分上限

public:
    float update(float current);                      // 提供当前值返回下次输出值
    void update_target(float target);                 // 更新目标值
    void update_pid(double kp, double ki, double kd); // 更新pid系数
    void reset();                                     // 重置pid控制器
    void out_limit(float out_min, float out_max);     // 设置输出限制
};

#endif /* PIDCONTROLLER_H */
