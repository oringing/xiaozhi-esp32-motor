#ifndef MOTOR_H_
#define MOTOR_H_

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

// PWM configuration parameters (used for motor speed control)
// PWM配置参数（电机调速用）
#define MOTOR_LEDC_MODE LEDC_LOW_SPEED_MODE
#define MOTOR_PWM_RES LEDC_TIMER_8_BIT  // 8位分辨率（速度范围0-255）

// 方向控制引脚数量（右侧2个+左侧2个，共4个）
#define MOTOR_DIR_PIN_COUNT 4

// 电机控制类（封装方向控制与PWM调速）
class MotorDriver {
 public:
  // 构造函数：初始化4个方向引脚和2个PWM调速引脚
  MotorDriver(gpio_num_t MOTOR_RIGHT_FORWARD, gpio_num_t MOTOR_RIGHT_BACKWARD,
              gpio_num_t MOTOR_LEFT_FORWARD, gpio_num_t MOTOR_LEFT_BACKWARD,
              gpio_num_t MOTOR_RIGHT_PWM, gpio_num_t MOTOR_LEFT_PWM); // 右侧/左侧PWM引脚

  // 初始化电机驱动（配置GPIO方向和PWM）
  void Init();

  // 停止所有电机
  void Stop();

  // 前进（两侧电机正向转动）
  void Forward(uint8_t speed);

  // 后退（两侧电机反向转动）
  void Backward(uint8_t speed);

  // 左转（右侧电机转动，左侧电机停止）
  void TurnLeft(uint8_t speed);

  // 右转（左侧电机转动，右侧电机停止）
  void TurnRight(uint8_t speed);

  // 设置定时自动停止
  void SetAutoStop(uint32_t time_ms);

  // 动态修改PWM频率（Hz）
  void SetPwmFrequency(uint32_t freq_hz);

 private:
  // 设置PWM占空比（内部调用）
  void SetPwmSpeed(ledc_channel_t ch, gpio_num_t pin, uint8_t speed);

  // 初始化PWM调速功能（基于LEDC）
  void InitPwm();

  // 初始化方向控制GPIO（配置为输出模式）
  void InitDirGpio();

  // 自动停止定时器回调（静态方法）
  static void AutoStopCallback(TimerHandle_t timer);

  gpio_num_t in_pins_[MOTOR_DIR_PIN_COUNT];  // 方向控制引脚数组[右IN1,右IN2,左IN1,左IN2]
  gpio_num_t right_pwm_pin_;                 // 右侧PWM引脚
  gpio_num_t left_pwm_pin_;                  // 左侧PWM引脚
  ledc_channel_t right_pwm_ch_ = LEDC_CHANNEL_3;  // 右侧PWM通道。新通道3避开 LED 常用资源，避免和led\gpio_led.cc、boards\common\backlight.cc里的冲突
  ledc_channel_t left_pwm_ch_ = LEDC_CHANNEL_4;   // 左侧PWM通道
  TimerHandle_t auto_stop_timer_ = nullptr;  // 自动停止定时器
  uint32_t pwm_freq_ = 1000;                 // PWM频率（默认1000Hz）
};

#endif  // MOTOR_H_