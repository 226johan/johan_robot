#include <Arduino.h>
#include <Wire.h>

#include "MPU6050_light.h"
#include <Esp32McpwmMotor.h>
#include "imu.h"
#include "Config.h"
#include <Esp32PcntEncoder.h>
#include <PidController.h>
#include <Kinematics.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include<nav_msgs/msg/odometry.h>
#include<micro_ros_utilities/string_utilities.h>

Esp32PcntEncoder encoders[4];    // 编码器
IMU imu;                         // imu
Esp32McpwmMotor motor;           // 电机
PidController pid_controller[4]; // 四个电机的pid控制器
Kinematics kinematics;           // 运动学模型

int64_t last_ticks[4];    // 上一次读取的计数器数值
int32_t delta_tick[4];    // 两次读取之间的计数器差值
int64_t last_update_time; // 上一次更新时间
float current_speeds[4];  // 四个电机的速度

float target_linear_x_speed = 0.0; // 目标x线速度 ms/s
float target_linear_y_speed = 0.0; // 目标y线速度 ms/s
float target_angular_speed = 0.f;  // 目标角速度 rad/s
float out_speed[4];                // 电机输出速度

float out_wheel_speed[4];

rcl_allocator_t allocator; // 内存分配器，用于动态内存管理
rclc_support_t support;    // 用于存储时钟，内存分配器和上下文，提供支持
rclc_executor_t executor;  // 用于执行节点的执行器
rcl_node_t node;           // 节点

rcl_subscription_t subcriber;      // 订阅者
geometry_msgs__msg__Twist sub_msg; // 订阅消息
rcl_publisher_t odom_publisher;  //发布者
nav_msgs__msg__Odometry odom_msg; //里程计消息
rcl_timer_t timer; //定时器


float out_motor_speed[4];

void callback_publisher(rcl_timer_t *timer,int64_t last_call_time){
    odom_t odom=kinematics.get_odom();
    int64_t stamp=rmw_uros_epoch_millis();
    odom_msg.header.stamp.sec=static_cast<int32_t>(stamp/1000); // s
    odom_msg.header.stamp.nanosec=static_cast<uint32_t>(stamp%1000)*1e6; // ns
    odom_msg.pose.pose.position.x=odom.x;
    odom_msg.pose.pose.position.y=odom.y;
    odom_msg.pose.pose.orientation.w=cos(odom.yaw*0.5);
    odom_msg.pose.pose.orientation.x=0;
    odom_msg.pose.pose.orientation.y=0;
    odom_msg.pose.pose.orientation.z=sin(odom.yaw*0.5);
    odom_msg.twist.twist.angular.z=odom.angular_speed;
    odom_msg.twist.twist.linear.x=odom.linear_x_speed;

    if(rcl_publish(&odom_publisher,&odom_msg,NULL)!=RCL_RET_OK){
        Serial.printf("error: odom publisher failed!\n");
    }
}



void twist_callback(const void *msg)
{
    const geometry_msgs__msg__Twist *twist_msg = (const geometry_msgs__msg__Twist *)msg;
    kinematics.kinematic_inverse(twist_msg->linear.x * 1000, twist_msg->linear.y * 1000, twist_msg->angular.z,
                                 pid_controller[0].target_, pid_controller[1].target_,pid_controller[2].target_, pid_controller[3].target_);
    for (int i = 0; i < 4; i++)
    {
        // pid_controller[i].update_target(out_wheel_speed[i]);

        if (pid_controller[i].target_ == 0)
        {
            out_motor_speed[i] = 0;
        }
        else
        {
            // 使用 pid_controller 控制器对电机速度进行 PID 控制
            out_motor_speed[i] = pid_controller[i].update(pid_controller[i].target_);
        }

        motor.updateMotorSpeed(i, pid_controller[i].update(out_motor_speed[i]));
        Serial.printf("speeds1=%fm/s ,speed2=%fm/s ,speed3=%fm/s ,speed4=%fm/s  \n",
                      out_wheel_speed[0], out_wheel_speed[1], out_wheel_speed[2], out_wheel_speed[3]);
    }
}

void micro_ros_task(void *parameter)
{
    // 设置传输协议，并延时等待完成
    IPAddress agent_ip;
    agent_ip.fromString(AGENT_IP);
    set_microros_wifi_transports(WIFI_NAME, WIFI_PASSWORD, agent_ip, 8888);
    delay(2000);

    // 初始化内存分配器
    allocator = rcl_get_default_allocator();

    // 初始化support
    rclc_support_init(&support, 0, NULL, &allocator);

    // 初始化节点
    rclc_node_init_default(&node, "robot_motion_control", "", &support);

    // 初始化执行器
    unsigned int number_handles = 2;
    rclc_executor_init(&executor, &support.context, number_handles, &allocator);

    odom_msg.header.frame_id=micro_ros_string_utilities_set(odom_msg.header.frame_id,"odom");
    odom_msg.child_frame_id=micro_ros_string_utilities_set(odom_msg.child_frame_id,"base_footprint");
    rclc_publisher_init_best_effort(&odom_publisher,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs,msg,Odometry),
                                    "/odom");
    while(!rmw_uros_epoch_synchronized()){      // 没有同步
        rmw_uros_sync_session(1000);    //尝试同步
        delay(10);
    }
    rclc_timer_init_default(&timer,&support,RCL_MS_TO_NS(50),callback_publisher);
    rclc_executor_add_timer(&executor,&timer);

    rclc_subscription_init_best_effort(&subcriber,
                                       &node,
                                       ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                                       "/cmd_vel");
    rclc_executor_add_subscription(&executor, &subcriber, &sub_msg, twist_callback, ON_NEW_DATA);

    // 循环执行器
    rclc_executor_spin(&executor);
}

void motor_speed_control()
{

    uint64_t dt = millis() - last_update_time;
    kinematics.update_motor_speed(millis(), encoders[0].getTicks(), encoders[1].getTicks(), encoders[2].getTicks(), encoders[3].getTicks());
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

    Serial.printf("speeds1=%fm/s ,speed2=%fm/s ,speed3=%fm/s ,speed4=%fm/s  \n",
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

    kinematics.set_wheel_distance(216.f, 177.f);
    kinematics.set_motor_params(0, 0.1051566);
    kinematics.set_motor_params(1, 0.1051566);
    kinematics.set_motor_params(2, 0.1051566);
    kinematics.set_motor_params(3, 0.1051566);

    kinematics.kinematic_inverse(target_linear_x_speed, target_linear_y_speed, target_angular_speed, out_speed[0], out_speed[1], out_speed[2], out_speed[3]);

    // 设置电机速度
    for (int i = 0; i < 4; i++)
    {
        // mm/s
        pid_controller[i].update_target(out_speed[i]);
    }

    // 创建micro_ros_task
    xTaskCreate(micro_ros_task, "micro_ros", 10240, NULL, 1, NULL);
}

void loop()
{

    delay(10);
    motor_speed_control();
    Serial.printf("x=%f,y=%f,angle=%f\n", kinematics.get_odom().x, kinematics.get_odom().y,
                  kinematics.get_odom().y,
                  kinematics.get_odom().yaw);
}