#ifndef DISTANCE_SENSOR_H_
#define DISTANCE_SENSOR_H_

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_attr.h"
#include "time.h"
#include "sys/time.h"

// 硬件配置宏定义（集中管理）
#define IR_RX_LEFT       GPIO_NUM_17       // 左轮编码器信号引脚
#define IR_RX_RIGHT      GPIO_NUM_8       // 右轮编码器信号引脚
#define WHEEL_DIAMETER_DEFAULT 0.066       // 默认车轮直径（米）,也就是66 mm
#define ENCODER_SLOTS    20                // 编码盘镂空槽和遮挡条数量N=20，每转一圈产生20个脉冲
#define PI               3.14159f          // π常量
#define LOG_PULSE_INTERVAL 10              // 日志打印间隔（每10个脉冲打印一次）
#define NO_SIGNAL_TIMEOUT 5000             // 无信号超时时间（毫秒）

class WheelEncoder {
private:
    // GPIO中断处理函数
    static void IRAM_ATTR IR_ISR(void* arg);
    
    // 静态成员变量用于存储实例指针
    static WheelEncoder* left_instance_;
    static WheelEncoder* right_instance_;

    // 硬件参数
    gpio_num_t rx_pin_;           // 接收引脚

    // 计数与控制参数
    float wheel_diameter_;
    volatile int pulse_count_;    
    SemaphoreHandle_t pulse_sem_; 

    // 时间戳
    TickType_t last_receive_time_;  
    TickType_t transmit_start_time_;  // 起始时间标记
    
    // 日志相关
    int last_logged_count_;        // 上次记录的日志计数
    bool timeout_logged_;          // 是否已记录超时日志

    // 接收任务
    static void receive_task(void* arg);

public:
    // 构造函数
    WheelEncoder(float wheel_diameter = WHEEL_DIAMETER_DEFAULT, 
              gpio_num_t rx_pin = IR_RX_LEFT);

    esp_err_t init();

    // 脉冲与距离操作
    int get_pulse_count();
    void reset_pulse_count();
    float get_distance();
    void set_wheel_diameter(float diameter);

    // 静态方法获取实例
    static WheelEncoder* get_left_instance();
    static WheelEncoder* get_right_instance();
    
    // 获取时间信息
    TickType_t get_last_receive_time();
    TickType_t get_transmit_start_time();
    
    // 日志检查和输出函数
    bool check_and_log(const char* prefix);
    bool check_timeout_and_log(const char* prefix);
    void reset_timeout_logged();  // 重置超时日志标志
    
    // 公共接口函数
    bool check_encoder_status();  // 将此函数移到public部分
};

// 距离获取函数和全局日志检查函数
float get_left_distance();
float get_right_distance();
float get_average_distance();
bool check_and_log_encoders();  // 检查并记录两个编码器的状态

#endif  // DISTANCE_SENSOR_H_