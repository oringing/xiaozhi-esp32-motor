#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "motor/motor.h"
#include "boards/bread-compact-wifi/motor_control.h"
#include "esp_log.h"
#include "driver/ledc.h"

#include <cinttypes>  // 添加这行头文件
#include <cstdint>

static const char* TAG = "MotorDriver";

// 构造函数：初始化引脚参数
MotorDriver::MotorDriver(gpio_num_t MOTOR_RIGHT_FORWARD, gpio_num_t MOTOR_RIGHT_BACKWARD,
                         gpio_num_t MOTOR_LEFT_FORWARD, gpio_num_t MOTOR_LEFT_BACKWARD,
                         gpio_num_t MOTOR_RIGHT_PWM, gpio_num_t MOTOR_LEFT_PWM) {
    in_pins_[0] = MOTOR_RIGHT_FORWARD;
    in_pins_[1] = MOTOR_RIGHT_BACKWARD;
    in_pins_[2] = MOTOR_LEFT_FORWARD;
    in_pins_[3] = MOTOR_LEFT_BACKWARD;
    right_pwm_pin_ = MOTOR_RIGHT_PWM;
    left_pwm_pin_ = MOTOR_LEFT_PWM;
    pwm_freq_ = 1000;  // 默认PWM频率1000Hz
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
    // 检查通道是否在0~7范围内
    if (right_pwm_ch_ > LEDC_CHANNEL_7 || left_pwm_ch_ > LEDC_CHANNEL_7) {
        ESP_LOGE(TAG, "PWM channel out of range (0-7)");
        return;
    }

    // 使用宏定义的合法PWM引脚列表（根据ESP32芯片和具体开发板调整）
    const gpio_num_t valid_pwm_pins[] = VALID_PWM_PINS;

    bool right_pin_valid = false, left_pin_valid = false;
    for (auto pin : valid_pwm_pins) {
        if (right_pwm_pin_ == pin) right_pin_valid = true;
        if (left_pwm_pin_ == pin) left_pin_valid = true;
    }
    if (!right_pin_valid || !left_pin_valid) {
        ESP_LOGE(TAG, "PWM pin does not support LEDC");
        return;
    }
    
    // 配置PWM定时器（Timer1）
    ledc_timer_config_t timer_cfg;
    timer_cfg.speed_mode = MOTOR_LEDC_MODE;
    timer_cfg.duty_resolution = MOTOR_PWM_RES;
    timer_cfg.timer_num = LEDC_TIMER_2; // 使用Timer2（避免和main\boards\common\backlight.cc里的LEDC_TIMER_0，以及gpio_led.cc宏定义里的LEDC_TIMER_1冲突）
    timer_cfg.freq_hz = pwm_freq_;  // 使用当前PWM频率
    timer_cfg.clk_cfg = LEDC_AUTO_CLK;
    timer_cfg.deconfigure = false;

    if (ledc_timer_config(&timer_cfg) != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to initialize PWM timer");
        return;
    }

    // 配置右侧PWM通道
    ledc_channel_config_t right_ch_cfg = {};
    right_ch_cfg.gpio_num = right_pwm_pin_;
    right_ch_cfg.speed_mode = MOTOR_LEDC_MODE;
    right_ch_cfg.channel = right_pwm_ch_;
    right_ch_cfg.timer_sel = LEDC_TIMER_2;  
    right_ch_cfg.duty = 0;  // 初始占空比0（停止）
    right_ch_cfg.hpoint = 0;
    right_ch_cfg.intr_type = LEDC_INTR_DISABLE;
    right_ch_cfg.flags.output_invert = 0; // 确保不反相输出
    right_ch_cfg.sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD;// 默认模式
    esp_err_t right_err = ledc_channel_config(&right_ch_cfg);
    if (right_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure right PWM channel: %d", right_err);
        return;
    }

    // 配置左侧PWM通道
    ledc_channel_config_t left_ch_cfg = {};
    left_ch_cfg.gpio_num = left_pwm_pin_;
    left_ch_cfg.speed_mode = MOTOR_LEDC_MODE;
    left_ch_cfg.channel = left_pwm_ch_;
    left_ch_cfg.timer_sel = LEDC_TIMER_2;  
    left_ch_cfg.duty = 0;  // 初始占空比0（停止）
    left_ch_cfg.hpoint = 0;
    left_ch_cfg.intr_type = LEDC_INTR_DISABLE;
    left_ch_cfg.flags.output_invert = 0; // 确保不反相输出
    left_ch_cfg.sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD; // 默认模式
    esp_err_t left_err = ledc_channel_config(&left_ch_cfg);
    if (left_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure left PWM channel: %d", left_err);
        return;
    }
    
    ESP_LOGI(TAG, "PWM channels configured successfully. Right pin: %d, Left pin: %d", 
             right_pwm_pin_, left_pwm_pin_);
}

// 停止所有电机
void MotorDriver::Stop() 
{
    // 方向引脚置低
    for (int i = 0; i < MOTOR_DIR_PIN_COUNT; ++i) {
        gpio_set_level(in_pins_[i], 0);
    }
    // PWM占空比置0
    SetPwmSpeed(right_pwm_ch_, right_pwm_pin_, 0);
    SetPwmSpeed(left_pwm_ch_, left_pwm_pin_, 0);
    ESP_LOGI(TAG, "All motors stopped");
}

// 前进动作
void MotorDriver::Forward(uint8_t speed) 
{
    // 右侧电机正向
    gpio_set_level(in_pins_[0], 1);
    gpio_set_level(in_pins_[1], 0);
    // 左侧电机正向
    gpio_set_level(in_pins_[2], 1);
    gpio_set_level(in_pins_[3], 0);

    // 设置PWM速度
    SetPwmSpeed(right_pwm_ch_, right_pwm_pin_, speed);
    SetPwmSpeed(left_pwm_ch_, left_pwm_pin_, speed);
    ESP_LOGI(TAG, "Moving forward with speed: %u", speed);
}

// 后退动作
void MotorDriver::Backward(uint8_t speed) 
{
    // 右侧电机反向
    gpio_set_level(in_pins_[0], 0);
    gpio_set_level(in_pins_[1], 1);
    // 左侧电机反向
    gpio_set_level(in_pins_[2], 0);
    gpio_set_level(in_pins_[3], 1);

    SetPwmSpeed(right_pwm_ch_, right_pwm_pin_, speed);
    SetPwmSpeed(left_pwm_ch_, left_pwm_pin_, speed);
    ESP_LOGI(TAG, "Moving backward with speed: %u", speed);
}

// 左转动作
void MotorDriver::TurnLeft(uint8_t speed) 
{
    // 右侧电机正向转动10
    gpio_set_level(in_pins_[0], 1);
    gpio_set_level(in_pins_[1], 0);
    // 左侧电机停止00，或者反向转动01
    gpio_set_level(in_pins_[2], 0);
    gpio_set_level(in_pins_[3], 0);

    SetPwmSpeed(right_pwm_ch_, right_pwm_pin_, speed);
    SetPwmSpeed(left_pwm_ch_, left_pwm_pin_, 0);// 左侧电机停止，速度为0，
    ESP_LOGI(TAG, "Turning left with speed: %u", speed);
}

// 右转动作
void MotorDriver::TurnRight(uint8_t speed) 
{
    // 右侧电机停止00，或者反向转动01
    gpio_set_level(in_pins_[0], 0);
    gpio_set_level(in_pins_[1], 0);
    // 左侧电机正向转动10
    gpio_set_level(in_pins_[2], 1);
    gpio_set_level(in_pins_[3], 0);

    SetPwmSpeed(right_pwm_ch_, right_pwm_pin_, 0);  // 右侧电机停止，速度为0，
    SetPwmSpeed(left_pwm_ch_, left_pwm_pin_,speed);
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

// 动态修改PWM频率
void MotorDriver::SetPwmFrequency(uint32_t freq_hz) {
    // 验证频率范围（50Hz-10kHz，根据电机特性）
    if (freq_hz < 50 || freq_hz > 10000) {
        ESP_LOGE(TAG, "Invalid PWM frequency: %" PRIu32 " Hz (range 50-10000)", freq_hz);
        return;
    }

    // 更新频率值
    pwm_freq_ = freq_hz;

    // 重新配置定时器（会自动应用到所有通道）
    ledc_timer_config_t timer_cfg;
    timer_cfg.speed_mode = MOTOR_LEDC_MODE;
    timer_cfg.duty_resolution = MOTOR_PWM_RES;
    timer_cfg.timer_num = LEDC_TIMER_2; // 使用Timer2（避免和Timer0、1冲突）
    timer_cfg.freq_hz = pwm_freq_;
    timer_cfg.clk_cfg = LEDC_AUTO_CLK;
    timer_cfg.deconfigure = false;
  
    if (ledc_timer_config(&timer_cfg) == ESP_OK) {
        ESP_LOGI(TAG, "PWM frequency updated to %" PRIu32 " Hz", freq_hz);
    } else {
        ESP_LOGE(TAG, "Failed to update PWM frequency");
    }
}