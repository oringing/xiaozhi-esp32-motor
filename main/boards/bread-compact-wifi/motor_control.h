#ifndef MAIN_BOARDS_BREAD_COMPACT_WIFI_MOTOR_CONTROL_H_
#define MAIN_BOARDS_BREAD_COMPACT_WIFI_MOTOR_CONTROL_H_

#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "mcp_server.h"
#include "driver/gpio.h"
#include "motor/motor.h"
#include "motor/distance_sensor.h"  // 引入距离传感器头文件
#include "application.h"  // 添加Application头文件包含
#include "assets/lang_config.h" // 添加Lang::Sounds头文件包含

// ESP32-S3-DevKitC-1 支持的PWM引脚列表（仅限此开发板）
//44针开发板：GPIO 0-21，33-48（包含边界0、21、33、48）
//4、5、6麦克风占用，7、15、16功放占用，38、39音量加减占用，41、42引脚0.91寸、0.96寸屏幕占用
#define VALID_PWM_PINS \
{ \
    GPIO_NUM_0,  GPIO_NUM_1,  GPIO_NUM_2,  GPIO_NUM_3, \
    GPIO_NUM_8,  GPIO_NUM_9,  GPIO_NUM_10, GPIO_NUM_11, \
    GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_14, \
    GPIO_NUM_17, GPIO_NUM_18, GPIO_NUM_19, \
    GPIO_NUM_21, GPIO_NUM_33, GPIO_NUM_34, GPIO_NUM_35, \
    GPIO_NUM_36, GPIO_NUM_37, \
    GPIO_NUM_40, GPIO_NUM_43, \
    GPIO_NUM_44, GPIO_NUM_45, GPIO_NUM_46, GPIO_NUM_47, \
    GPIO_NUM_48 \
}
// 电机控制GPIO宏定义，从上面列的合法且未被占用的GPIO引脚中选择
//电机1、2，三个引脚在ESP32S3开发板上邻近
#define MOTOR_RIGHT_PWM GPIO_NUM_12  // 右侧PWM引脚
#define MOTOR_RIGHT_FORWARD GPIO_NUM_13 // 右侧电机正转引脚
#define MOTOR_RIGHT_BACKWARD GPIO_NUM_14 // 右侧电机反转引脚


#define MOTOR_LEFT_PWM GPIO_NUM_9  // 左侧PWM引脚
#define MOTOR_LEFT_FORWARD  GPIO_NUM_10  // 左侧电机正转引脚
#define MOTOR_LEFT_BACKWARD GPIO_NUM_11  // 左侧电机反转引脚
class MotorControl {
 public:
    MotorControl();
    ~MotorControl();

    void InitializeTools();
    void SetParameters(int speed, int time);
    void Forward(int speed, int time);
    void Backward(int speed, int time);
    void TurnLeft(int speed, int time);
    void TurnRight(int speed, int time);
    void Stop();
    // 添加转圈方法
    void SpinClockwise(int speed, int time);
    void SpinCounterClockwise(int speed, int time);
    // 新增：按距离前进、后退
    void MoveDistance(float target_meters, int speed);  
    int GetMotorSpeed() const;
    int GetRunTime() const;

 private:
    static void StopMotorTimerCallback(TimerHandle_t timer);

    // 新增编码器成员
    WheelEncoder left_encoder_;   // 左轮编码器实例
    WheelEncoder right_encoder_;  // 右轮编码器实例
    
    TimerHandle_t stop_timer_ = nullptr;
    MotorDriver motor_driver_;
    int motor_speed_ = 0;
    int run_time_ = 2;
};

// 全局电机控制实例声明
extern MotorControl* g_motor_control;

// C接口函数声明
extern "C" {
    void motor_control_init();
    void motor_forward(int speed, int time);
    void motor_backward(int speed, int time);
    void motor_turn_left(int speed, int time);
    void motor_turn_right(int speed, int time);
    void motor_stop();
    int motor_get_speed();
    int motor_get_run_time();
}
#endif  // MAIN_BOARDS_BREAD_COMPACT_WIFI_MOTOR_CONTROL_H_