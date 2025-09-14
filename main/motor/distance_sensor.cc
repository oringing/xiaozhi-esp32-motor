#include "distance_sensor.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "time.h"
#include "sys/time.h"

static const char* TAG = "DISTANCE_SENSOR";

// 静态成员变量初始化
WheelEncoder* WheelEncoder::left_instance_ = nullptr;
WheelEncoder* WheelEncoder::right_instance_ = nullptr;

// 构造函数初始化
WheelEncoder::WheelEncoder(float wheel_diameter, gpio_num_t rx_pin)
    : rx_pin_(rx_pin), wheel_diameter_(wheel_diameter),
      pulse_count_(0), last_receive_time_(0), transmit_start_time_(0),
      last_logged_count_(0), timeout_logged_(false) {
    pulse_sem_ = xSemaphoreCreateBinary();
    
    // 根据引脚设置实例
    if (rx_pin == IR_RX_LEFT) {
        left_instance_ = this;
    } else if (rx_pin == IR_RX_RIGHT) {
        right_instance_ = this;
    }
    
    // 初始化起始时间
    transmit_start_time_ = xTaskGetTickCount();
}


// 初始化硬件
esp_err_t WheelEncoder::init() {
    // 1. 配置接收引脚（输入+上拉，下降沿中断）
    gpio_config_t rx_conf = {
        .pin_bit_mask = (1ULL << rx_pin_),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE  // 下降沿触发
    };
    esp_err_t err = gpio_config(&rx_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO %d, error: %d", rx_pin_, err);
        return err;
    }
    
    ESP_LOGI(TAG, "Configured GPIO %d as input with pull-up and falling edge interrupt", rx_pin_);
    int initial_level = gpio_get_level(rx_pin_);
    ESP_LOGI(TAG, "Initial GPIO level: %d", initial_level);
    
    // 安装GPIO中断服务（只在第一次调用时安装）
    static bool isr_service_installed = false;
    if (!isr_service_installed) {
        err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "Failed to install GPIO ISR service, error: %d", err);
            return err;
        }
        isr_service_installed = true;
        ESP_LOGI(TAG, "GPIO ISR service installed");
    }
    
    // 注册编码器信号引脚的中断处理函数
    err = gpio_isr_handler_add(rx_pin_, IR_ISR, this);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR handler for GPIO %d, error: %d", rx_pin_, err);
        return err;
    }
    
    // 验证中断是否注册成功
    ESP_LOGI(TAG, "ISR handler added for GPIO %d", rx_pin_);

    // 2. 创建接收任务
    BaseType_t task_result = xTaskCreate(receive_task, "wheel_receive_task", 4096, this, 4, NULL);
    if (task_result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create wheel receive task, error: %d", task_result);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "编码器初始化完成，引脚: %d", rx_pin_);
    return ESP_OK;
}


// 接收任务：持续运行，等待信号变化
void WheelEncoder::receive_task(void* arg) {
    WheelEncoder* encoder = static_cast<WheelEncoder*>(arg);
    while (1) {
        // 等待脉冲信号
        if (xSemaphoreTake(encoder->pulse_sem_, pdMS_TO_TICKS(1000)) == pdTRUE) {
            // 处理接收到的脉冲信号
            // 日志输出在外部调用中处理
        }
    }
}

// GPIO中断处理函数
void IRAM_ATTR WheelEncoder::IR_ISR(void* arg) {
    WheelEncoder* encoder = static_cast<WheelEncoder*>(arg);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // 添加去抖动处理 - 检查时间间隔
    static uint32_t last_interrupt_time[2] = {0, 0}; // 用于GPIO17和GPIO18
    uint32_t current_time = xTaskGetTickCountFromISR();
    
    int index = (encoder->rx_pin_ == GPIO_NUM_17) ? 0 : 1;
    // 如果距离上次中断时间太短，则忽略（去抖动）
    if ((current_time - last_interrupt_time[index]) > pdMS_TO_TICKS(1)) {
        // 增加计数器
        int old_count = encoder->pulse_count_;
        encoder->pulse_count_ = encoder->pulse_count_ + 1;
        encoder->last_receive_time_ = current_time;
        encoder->timeout_logged_ = false;  // 有新信号时重置超时标志
        last_interrupt_time[index] = current_time;
        
        // 添加调试信息，增加触发次数以更好地观察
        if (encoder->pulse_count_ <= 50) {  // 增加到前50次都打印
            esp_rom_printf("Encoder interrupt: pin=%d, count=%d->%d\n", 
                          encoder->rx_pin_, old_count, encoder->pulse_count_);
        }
    }
    
    xSemaphoreGiveFromISR(encoder->pulse_sem_, &xHigherPriorityTaskWoken);
    
    // 如果需要，请求上下文切换
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// 添加一个新的方法来检查编码器状态
bool WheelEncoder::check_encoder_status() {
    static int last_pin_level = -1;
    int current_pin_level = gpio_get_level(rx_pin_);
    
    if (last_pin_level != current_pin_level) {
        ESP_LOGI(TAG, "GPIO %d level changed to: %d", rx_pin_, current_pin_level);
        last_pin_level = current_pin_level;
        return true;
    }
    return false;
}

// 其他函数实现
int WheelEncoder::get_pulse_count() {
    return pulse_count_;
}

void WheelEncoder::reset_pulse_count() {
    pulse_count_ = 0;
    last_logged_count_ = 0;
    timeout_logged_ = false;
}

float WheelEncoder::get_distance() {
    return (float)pulse_count_ * PI * wheel_diameter_ / ENCODER_SLOTS;
}

void WheelEncoder::set_wheel_diameter(float diameter) {
    wheel_diameter_ = diameter;
}

TickType_t WheelEncoder::get_last_receive_time() {
    return last_receive_time_;
}

TickType_t WheelEncoder::get_transmit_start_time() {
    return transmit_start_time_;
}

WheelEncoder* WheelEncoder::get_left_instance() {
    return left_instance_;
}

WheelEncoder* WheelEncoder::get_right_instance() {
    return right_instance_;
}

// 检查并记录日志函数（每转一圈记录一次）
bool WheelEncoder::check_and_log(const char* prefix) {
    int pulse_count = get_pulse_count();
    if (pulse_count >= last_logged_count_ + LOG_PULSE_INTERVAL) {
        ESP_LOGI(TAG, "%s脉冲: %d 次 | 距离: %.2f m", prefix, pulse_count, get_distance());
        last_logged_count_ = pulse_count - (pulse_count % LOG_PULSE_INTERVAL);
        return true;
    }
    return false;
}

// 检查超时并记录日志（只记录一次）
bool WheelEncoder::check_timeout_and_log(const char* prefix) {
    if (!timeout_logged_ && 
        (xTaskGetTickCount() - get_last_receive_time()) > pdMS_TO_TICKS(NO_SIGNAL_TIMEOUT)) {
        float current_distance = get_distance();  // 获取当前距离
        ESP_LOGI(TAG, "%s无信号，当前计数：%d，距离：%.2f m", prefix, get_pulse_count(), current_distance);
        timeout_logged_ = true;
        return true;
    }
    return false;
}

// 重置超时日志标志
void WheelEncoder::reset_timeout_logged() {
    timeout_logged_ = false;
}

// 获取左右轮编码器距离的辅助函数
float get_left_distance() {
    WheelEncoder* left_encoder = WheelEncoder::get_left_instance();
    if (left_encoder) {
        return left_encoder->get_distance();
    }
    return 0.0f;
}

float get_right_distance() {
    WheelEncoder* right_encoder = WheelEncoder::get_right_instance();
    if (right_encoder) {
        return right_encoder->get_distance();
    }
    return 0.0f;
}

float get_average_distance() {
    return (get_left_distance() + get_right_distance()) / 2.0f;
}

// 检查并记录两个编码器的状态（供外部调用）
bool check_and_log_encoders() {
    bool logged = false;
    WheelEncoder* left_encoder = WheelEncoder::get_left_instance();
    WheelEncoder* right_encoder = WheelEncoder::get_right_instance();
    
    // 检查左轮编码器
    if (left_encoder) {
        logged |= left_encoder->check_and_log("左轮 ");
        logged |= left_encoder->check_timeout_and_log("左轮 ");
    }
    
    // 检查右轮编码器
    if (right_encoder) {
        logged |= right_encoder->check_and_log("右轮 ");
        logged |= right_encoder->check_timeout_and_log("右轮 ");
    }
    
    return logged;
}