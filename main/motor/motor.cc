
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "motor/motor.h"
#include <cstdint>
#include "esp_log.h"

static const char* TAG = "MotorDriver";

// 构造函数：初始化引脚参数
MotorDriver::MotorDriver(gpio_num_t in1, gpio_num_t in2, gpio_num_t in3,
                         gpio_num_t in4, gpio_num_t in5, gpio_num_t in6,
                         gpio_num_t in7, gpio_num_t in8, gpio_num_t ena,
                         gpio_num_t enb) {
  in_pins_[0] = in1;
  in_pins_[1] = in2;
  in_pins_[2] = in3;
  in_pins_[3] = in4;
  in_pins_[4] = in5;
  in_pins_[5] = in6;
  in_pins_[6] = in7;
  in_pins_[7] = in8;
  ena_pin_ = ena;
  enb_pin_ = enb;
}

// 初始化电机驱动资源
void MotorDriver::Init() 
{
  InitDirGpio();
  InitPwm();

  // 创建自动停止定时器
  auto_stop_timer_ = xTimerCreate(
      "MotorAutoStop",
      pdMS_TO_TICKS(1000),  // 初始周期1秒（可动态调整）
      pdFALSE,              // 单次触发
      this,                 // 传递当前对象指针
      AutoStopCallback);
  if (auto_stop_timer_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create auto-stop timer");
  }
}

// 初始化方向控制GPIO
void MotorDriver::InitDirGpio() 
{
  uint64_t pin_bit_mask = 0;
  for (int i = 0; i < MOTOR_DIR_PIN_COUNT; ++i) {
    pin_bit_mask |= (1ULL << in_pins_[i]);
  }

  gpio_config_t dir_gpio_cfg = 
  {
      .pin_bit_mask = pin_bit_mask,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&dir_gpio_cfg);

  // 初始化为低电平（电机停止）
  for (int i = 0; i < MOTOR_DIR_PIN_COUNT; ++i) {
    gpio_set_level(in_pins_[i], 0);
  }
}

// 初始化PWM调速功能
void MotorDriver::InitPwm() 
{
  // 配置PWM定时器（Timer0）
  ledc_timer_config_t timer_cfg;
  timer_cfg.speed_mode = MOTOR_LEDC_MODE;
  timer_cfg.duty_resolution = MOTOR_PWM_RES;
  timer_cfg.timer_num = LEDC_TIMER_0;
  timer_cfg.freq_hz = MOTOR_PWM_FREQ;
  timer_cfg.clk_cfg = LEDC_AUTO_CLK;
  timer_cfg.deconfigure = false;

  if (ledc_timer_config(&timer_cfg) != ESP_OK) 
  {
    ESP_LOGE(TAG, "Failed to initialize PWM timer");
    return;
  }

  // 配置ENA通道
  ledc_channel_config_t ena_ch_cfg;
  ena_ch_cfg.gpio_num = ena_pin_;
  ena_ch_cfg.speed_mode = MOTOR_LEDC_MODE;
  ena_ch_cfg.channel = ena_ch_;
  ena_ch_cfg.timer_sel = LEDC_TIMER_0;
  ena_ch_cfg.duty = 0;  // 初始占空比0（停止）
  ena_ch_cfg.hpoint = 0;
  ena_ch_cfg.intr_type = LEDC_INTR_DISABLE;
  ledc_channel_config(&ena_ch_cfg);

  // 配置ENB通道
  ledc_channel_config_t enb_ch_cfg;
  enb_ch_cfg.gpio_num = enb_pin_;
  enb_ch_cfg.speed_mode = MOTOR_LEDC_MODE;
  enb_ch_cfg.channel = enb_ch_;
  enb_ch_cfg.timer_sel = LEDC_TIMER_0;
  enb_ch_cfg.duty = 0;  // 初始占空比0（停止）
  enb_ch_cfg.hpoint = 0;
  enb_ch_cfg.intr_type = LEDC_INTR_DISABLE;
  ledc_channel_config(&enb_ch_cfg);
}

// 停止所有电机
void MotorDriver::Stop() 
{
  // 方向引脚置低
  for (int i = 0; i < MOTOR_DIR_PIN_COUNT; ++i) {
    gpio_set_level(in_pins_[i], 0);
  }
  // PWM占空比置0
  SetPwmSpeed(ena_ch_, ena_pin_, 0);
  SetPwmSpeed(enb_ch_, enb_pin_, 0);
  ESP_LOGI(TAG, "All motors stopped");
}

// 前进动作
void MotorDriver::Forward(uint8_t speed) 
{
  // 配置方向引脚（正向转动）
  gpio_set_level(in_pins_[0], 1);
  gpio_set_level(in_pins_[1], 0);  // 电机1正向
  gpio_set_level(in_pins_[2], 1);
  gpio_set_level(in_pins_[3], 0);  // 电机2正向
  gpio_set_level(in_pins_[4], 1);
  gpio_set_level(in_pins_[5], 0);  // 电机3正向
  gpio_set_level(in_pins_[6], 1);
  gpio_set_level(in_pins_[7], 0);  // 电机4正向

  // 设置PWM速度
  SetPwmSpeed(ena_ch_, ena_pin_, speed);
  SetPwmSpeed(enb_ch_, enb_pin_, speed);
  ESP_LOGI(TAG, "Moving forward with speed: %u", speed);
}

// 后退动作
void MotorDriver::Backward(uint8_t speed) 
{
  // 配置方向引脚（反向转动）
  gpio_set_level(in_pins_[0], 0);
  gpio_set_level(in_pins_[1], 1);  // 电机1反向
  gpio_set_level(in_pins_[2], 0);
  gpio_set_level(in_pins_[3], 1);  // 电机2反向
  gpio_set_level(in_pins_[4], 0);
  gpio_set_level(in_pins_[5], 1);  // 电机3反向
  gpio_set_level(in_pins_[6], 0);
  gpio_set_level(in_pins_[7], 1);  // 电机4反向

  SetPwmSpeed(ena_ch_, ena_pin_, speed);
  SetPwmSpeed(enb_ch_, enb_pin_, speed);
  ESP_LOGI(TAG, "Moving backward with speed: %u", speed);
}

// 左转动作
void MotorDriver::TurnLeft(uint8_t speed) 
{
    // 左侧电机（3、4）停止，右侧电机（1、2）正向
      gpio_set_level(in_pins_[0], 1);
      gpio_set_level(in_pins_[1], 0);  // 电机1正向
      gpio_set_level(in_pins_[2], 1);
      gpio_set_level(in_pins_[3], 0);  // 电机2正向
      gpio_set_level(in_pins_[4], 0);
      gpio_set_level(in_pins_[5], 0);  // 电机3停止
      gpio_set_level(in_pins_[6], 0);
      gpio_set_level(in_pins_[7], 0);  // 电机4停止
     
      //逆时针原地打转型转弯
      // gpio_set_level(in_pins_[0], 1);
      // gpio_set_level(in_pins_[1], 0);  // 电机1前进
      // gpio_set_level(in_pins_[2], 1);
      // gpio_set_level(in_pins_[3], 0);  // 电机2前进
      // gpio_set_level(in_pins_[4], 0);
      // gpio_set_level(in_pins_[5], 1);  // 电机3后退
      // gpio_set_level(in_pins_[6], 0);
      // gpio_set_level(in_pins_[7], 1);  // 电机4后退

      SetPwmSpeed(ena_ch_, ena_pin_, 0);     // 左侧PWM关闭
      SetPwmSpeed(enb_ch_, enb_pin_, speed); // 右侧PWM使能
      ESP_LOGI(TAG, "Turning left with speed: %u", speed);
    }

    // 右转动作
  void MotorDriver::TurnRight(uint8_t speed) 
  {
      // 右侧电机（3、4）停止，左侧电机（1、2）正向
      gpio_set_level(in_pins_[0], 0);
      gpio_set_level(in_pins_[1], 0);  // 电机1正向
      gpio_set_level(in_pins_[2], 0);
      gpio_set_level(in_pins_[3], 0);  // 电机2正向
      gpio_set_level(in_pins_[4], 1);
      gpio_set_level(in_pins_[5], 0);  // 电机3停止
      gpio_set_level(in_pins_[6], 1);
      gpio_set_level(in_pins_[7], 0);  // 电机4停止

      //顺时针原地打转型转弯
      // gpio_set_level(in_pins_[0], 0);
      // gpio_set_level(in_pins_[1], 1);  // 电机1正向
      // gpio_set_level(in_pins_[2], 0);
      // gpio_set_level(in_pins_[3], 1);  // 电机2正向
      // gpio_set_level(in_pins_[4], 1);
      // gpio_set_level(in_pins_[5], 0);  // 电机3停止
      // gpio_set_level(in_pins_[6], 1);
      // gpio_set_level(in_pins_[7], 0);  // 电机4停止

  SetPwmSpeed(ena_ch_, ena_pin_, speed);  // 左侧PWM使能
  SetPwmSpeed(enb_ch_, enb_pin_, 0);      // 右侧PWM关闭
  ESP_LOGI(TAG, "Turning right with speed: %u", speed);
}

// 设置PWM占空比
void MotorDriver::SetPwmSpeed(ledc_channel_t ch, gpio_num_t pin,
                              uint8_t speed) {
  ledc_set_duty(MOTOR_LEDC_MODE, ch, speed);
  ledc_update_duty(MOTOR_LEDC_MODE, ch);
  ESP_LOGD(TAG, "PWM pin %d set to speed: %u", pin, speed);
}

// 配置自动停止
void MotorDriver::SetAutoStop(uint32_t time_ms) {
  if (auto_stop_timer_ == nullptr) {
    ESP_LOGE(TAG, "Auto-stop timer not initialized");
    return;
  }

  xTimerStop(auto_stop_timer_, portMAX_DELAY);
  xTimerChangePeriod(auto_stop_timer_, pdMS_TO_TICKS(time_ms), portMAX_DELAY);
  xTimerStart(auto_stop_timer_, portMAX_DELAY);
  ESP_LOGI(TAG, "Auto-stop set to %lu ms", time_ms);
}

// 自动停止回调
void MotorDriver::AutoStopCallback(TimerHandle_t timer) {
  MotorDriver* motor = static_cast<MotorDriver*>(pvTimerGetTimerID(timer));
  if (motor != nullptr) {
    motor->Stop();
  }
}