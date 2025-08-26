#ifndef MOTOR_H_
#define MOTOR_H_

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

// PWM配置参数（电机调速用）
#define MOTOR_LEDC_MODE LEDC_LOW_SPEED_MODE
#define MOTOR_PWM_FREQ 1000     // 电机常用PWM频率：1000Hz
#define MOTOR_PWM_RES LEDC_TIMER_8_BIT  // 8位分辨率（速度范围0-255）

// 方向控制引脚定义
#define MOTOR_DIR_PIN_COUNT 8

// 电机控制类（封装方向控制与PWM调速）
class MotorDriver {
 public:
  // 构造函数：初始化8个方向引脚和2个PWM调速引脚
  MotorDriver(gpio_num_t in1, gpio_num_t in2,  // 1号电机
              gpio_num_t in3, gpio_num_t in4,  // 2号电机
              gpio_num_t in5, gpio_num_t in6,  // 3号电机
              gpio_num_t in7, gpio_num_t in8,  // 4号电机
              gpio_num_t ena, gpio_num_t enb); // PWM调速引脚（ENA/ENB）

  // 初始化电机驱动（配置GPIO方向和PWM）
  void Init();

  // 停止所有电机
  void Stop();

  // 前进（4个电机正向转动）
  void Forward(uint8_t speed);

  // 后退（4个电机反向转动）
  void Backward(uint8_t speed);

  // 左转（差速：左侧电机停，右侧电机转）
  void TurnLeft(uint8_t speed);

  // 右转（差速：右侧电机停，左侧电机转）
  void TurnRight(uint8_t speed);

  // 设置定时自动停止
  void SetAutoStop(uint32_t time_ms);

 private:
  // 设置PWM占空比（内部调用）
  void SetPwmSpeed(ledc_channel_t ch, gpio_num_t pin, uint8_t speed);

  // 初始化方向控制GPIO（配置为输出模式）
  void InitDirGpio();

  // 初始化PWM调速功能（基于LEDC）
  void InitPwm();

  // 自动停止定时器回调（静态方法）
  static void AutoStopCallback(TimerHandle_t timer);

  gpio_num_t in_pins_[MOTOR_DIR_PIN_COUNT];  // 方向控制引脚数组
  gpio_num_t ena_pin_;                       // ENA引脚（PWM调速）
  gpio_num_t enb_pin_;                       // ENB引脚（PWM调速）
  ledc_channel_t ena_ch_ = LEDC_CHANNEL_0;   // ENA对应PWM通道
  ledc_channel_t enb_ch_ = LEDC_CHANNEL_1;   // ENB对应PWM通道
  TimerHandle_t auto_stop_timer_ = nullptr;  // 自动停止定时器
};

#endif  // MOTOR_H_