#include "motor_control.h"
#include "motor/distance_sensor.h"  // 引入距离传感器头文件

#include <string>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "mcp_server.h"

#define TAG "MotorControl"

// 全局电机控制实例定义
MotorControl* g_motor_control = nullptr;

// 初始化电机驱动引脚映射（使用用户定义的宏）
MotorControl::MotorControl()
    : left_encoder_(WHEEL_DIAMETER_DEFAULT, IR_RX_LEFT),    // 左轮编码器
      right_encoder_(WHEEL_DIAMETER_DEFAULT, IR_RX_RIGHT),  // 右轮编码器
      motor_driver_(MOTOR_RIGHT_FORWARD, MOTOR_RIGHT_BACKWARD,  // 右侧方向引脚
                    MOTOR_LEFT_FORWARD, MOTOR_LEFT_BACKWARD,   // 左侧方向引脚
                    MOTOR_RIGHT_PWM, MOTOR_LEFT_PWM) { // 右侧/左侧PWM引脚
  // 初始化电机驱动
  motor_driver_.Init();
  
  // 初始化编码器前添加调试信息
  ESP_LOGI(TAG, "开始初始化编码器...");
  
  // 初始化编码器
  esp_err_t left_err = left_encoder_.init();
  esp_err_t right_err = right_encoder_.init();
  
  if (left_err != ESP_OK) {
      ESP_LOGE(TAG, "左轮编码器初始化失败: %d", left_err);
  } else {
      ESP_LOGI(TAG, "左轮编码器初始化成功");
  }
  
  if (right_err != ESP_OK) {
      ESP_LOGE(TAG, "右轮编码器初始化失败: %d", right_err);
  } else {
      ESP_LOGI(TAG, "右轮编码器初始化成功");
  }
  
  if (left_err == ESP_OK && right_err == ESP_OK) {
      ESP_LOGI(TAG, "编码器初始化成功");
  }
  
  // 创建停止定时器（单次触发）
  stop_timer_ = xTimerCreate("MotorStopTimer", pdMS_TO_TICKS(1000), pdFALSE,
                             this, StopMotorTimerCallback);
  if (stop_timer_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create stop timer");
  }
}

MotorControl::~MotorControl() {
  if (stop_timer_ != nullptr) {
    xTimerDelete(stop_timer_, portMAX_DELAY);
  }
}

void MotorControl::StopMotorTimerCallback(TimerHandle_t timer) {
  MotorControl* motor_control = static_cast<MotorControl*>(pvTimerGetTimerID(timer));
  if (motor_control != nullptr) {
    ESP_LOGI(TAG, "Timer expired, stopping motor");
    motor_control->motor_driver_.Stop();
  }
}

void MotorControl::InitializeTools() {
  auto& mcp_server = McpServer::GetInstance();
  ESP_LOGI(TAG, "开始注册电机MCP工具...");

  // 前进控制工具
  std::vector<Property> forward_properties = {
    Property("speed", kPropertyTypeInteger, 100),
    Property("time", kPropertyTypeInteger, 2)
  };
  
  mcp_server.AddTool(
      "self.motor.forward", "控制电机前进。speed: 速度(0-255); time: 运行时间(0-100秒)",
      PropertyList(forward_properties),
      [this](const PropertyList& properties) -> ReturnValue {
        int speed = properties["speed"].value<int>();
        int time = properties["time"].value<int>();
        Forward(speed, time);
        return "电机前进，速度：" + std::to_string(speed) + "，时间：" +
               std::to_string(time) + "秒";
      });

  // 后退控制工具
  std::vector<Property> backward_properties = {
    Property("speed", kPropertyTypeInteger, 100),
    Property("time", kPropertyTypeInteger, 2)
  };
  
  mcp_server.AddTool(
      "self.motor.backward", "控制电机后退。speed: 速度(0-255); time: 运行时间(0-100秒)",
      PropertyList(backward_properties),
      [this](const PropertyList& properties) -> ReturnValue {
        int speed = properties["speed"].value<int>();
        int time = properties["time"].value<int>();
        Backward(speed, time);
        return "电机后退，速度：" + std::to_string(speed) + "，时间：" +
               std::to_string(time) + "秒";
      });

  // 左转控制工具
  std::vector<Property> turn_left_properties = {
    Property("speed", kPropertyTypeInteger, 100),
    Property("time", kPropertyTypeInteger, 2)
  };
  
  mcp_server.AddTool(
      "self.motor.turn_left", "控制电机左转。speed: 速度(0-255); time: 运行时间(0-100秒)",
      PropertyList(turn_left_properties),
      [this](const PropertyList& properties) -> ReturnValue {
        int speed = properties["speed"].value<int>();
        int time = properties["time"].value<int>();
        TurnLeft(speed, time);
        return "电机左转，速度：" + std::to_string(speed) + "，时间：" +
               std::to_string(time) + "秒";
      });

  // 右转控制工具
  std::vector<Property> turn_right_properties = {
    Property("speed", kPropertyTypeInteger, 100),
    Property("time", kPropertyTypeInteger, 2)
  };
  
  mcp_server.AddTool(
      "self.motor.turn_right", "控制电机右转。speed: 速度(0-255); time: 运行时间(0-100秒)",
      PropertyList(turn_right_properties),
      [this](const PropertyList& properties) -> ReturnValue {
        int speed = properties["speed"].value<int>();
        int time = properties["time"].value<int>();
        TurnRight(speed, time);
        return "电机右转，速度：" + std::to_string(speed) + "，时间：" +
               std::to_string(time) + "秒";
      });

  // 停止控制工具
  mcp_server.AddTool(
      "self.motor.stop", "立即停止电机运行", PropertyList(),
      [this](const PropertyList& properties) -> ReturnValue {
        Stop();
        return "电机已停止";
      });

  // 获取电机状态工具
  mcp_server.AddTool(
      "self.motor.get_status", "获取当前电机状态", PropertyList(),
      [this](const PropertyList& properties) -> ReturnValue {
        std::string status = "{\"speed\":" + std::to_string(motor_speed_) +
                            ",\"run_time\":" + std::to_string(run_time_) + "}";
        return status;
      });

  // 按距离前进控制工具
  std::vector<Property> move_distance_properties = {
    Property("distance", kPropertyTypeString, "1.0"),
    Property("speed", kPropertyTypeInteger, 100)
  };
  
  mcp_server.AddTool(
      "self.motor.move_distance", 
      "控制电机前进指定距离。distance: 距离(米, 0-5); speed: 速度(0-255)",
      PropertyList(move_distance_properties),
      [this](const PropertyList& properties) -> ReturnValue {
        std::string distance_str = properties["distance"].value<std::string>();
        float distance = std::stof(distance_str);
        int speed = properties["speed"].value<int>();
        MoveDistance(distance, speed);
        std::string result = "电机前进指定距离：" + std::to_string(distance) + "米，速度：" +
               std::to_string(speed);
        return result;
      });

  ESP_LOGI(TAG, "电机MCP工具注册完成");
}

void MotorControl::SetParameters(int speed, int time) {
  motor_speed_ = (speed < 0) ? 0 : ((speed > 255) ? 255 : speed);
  run_time_ = (time < 0) ? 0 : ((time > 100) ? 100 : time);
}

void MotorControl::Forward(int speed, int time) {
  SetParameters(speed, time);
  motor_driver_.Forward(motor_speed_);

  // 启动停止定时器
  if (stop_timer_ != nullptr) {
    xTimerStop(stop_timer_, portMAX_DELAY);
    xTimerChangePeriod(stop_timer_, pdMS_TO_TICKS(run_time_ * 1000), portMAX_DELAY);
    xTimerStart(stop_timer_, portMAX_DELAY);
  }

  ESP_LOGI(TAG, "Forward: speed=%d, run_time=%d", motor_speed_, run_time_);
}

void MotorControl::Backward(int speed, int time) {
  SetParameters(speed, time);
  motor_driver_.Backward(motor_speed_);

  if (stop_timer_ != nullptr) {
    xTimerStop(stop_timer_, portMAX_DELAY);
    xTimerChangePeriod(stop_timer_, pdMS_TO_TICKS(run_time_ * 1000), portMAX_DELAY);
    xTimerStart(stop_timer_, portMAX_DELAY);
  }

  ESP_LOGI(TAG, "Backward: speed=%d, run_time=%d", motor_speed_, run_time_);
}

void MotorControl::TurnLeft(int speed, int time) {
  SetParameters(speed, time);
  motor_driver_.TurnLeft(motor_speed_);

  if (stop_timer_ != nullptr) {
    xTimerStop(stop_timer_, portMAX_DELAY);
    xTimerChangePeriod(stop_timer_, pdMS_TO_TICKS(run_time_ * 1000), portMAX_DELAY);
    xTimerStart(stop_timer_, portMAX_DELAY);
  }

  ESP_LOGI(TAG, "Turn left: speed=%d, run_time=%d", motor_speed_, run_time_);
}

void MotorControl::TurnRight(int speed, int time) {
  SetParameters(speed, time);
  motor_driver_.TurnRight(motor_speed_);

  if (stop_timer_ != nullptr) {
    xTimerStop(stop_timer_, portMAX_DELAY);
    xTimerChangePeriod(stop_timer_, pdMS_TO_TICKS(run_time_ * 1000), portMAX_DELAY);
    xTimerStart(stop_timer_, portMAX_DELAY);
  }

  ESP_LOGI(TAG, "Turn right: speed=%d, run_time=%d", motor_speed_, run_time_);
}

void MotorControl::Stop() {
  motor_driver_.Stop();
  ESP_LOGI(TAG, "Motor stopped manually");
}


// 新增：按指定距离前进
void MotorControl::MoveDistance(float target_meters, int speed) {
  ESP_LOGI(TAG, "开始执行移动距离命令: %.2f米, 速度: %d", target_meters, speed);
  
  // 重置计数
  left_encoder_.reset_pulse_count();
  right_encoder_.reset_pulse_count();
  
  // 添加调试信息
  ESP_LOGI(TAG, "初始左轮脉冲数: %d, 右轮脉冲数: %d", 
           left_encoder_.get_pulse_count(), 
           right_encoder_.get_pulse_count());

  // 设置速度并启动电机
  SetParameters(speed, 0);  // 时间设为0，表示不使用定时器停止
  motor_driver_.Forward(motor_speed_);
  
  // 等待电机启动
  vTaskDelay(pdMS_TO_TICKS(100));
  
  // 再次检查脉冲数
  ESP_LOGI(TAG, "电机启动后左轮脉冲数: %d, 右轮脉冲数: %d", 
           left_encoder_.get_pulse_count(), 
           right_encoder_.get_pulse_count());

  // 循环检测距离是否达标（使用平均值判断）
  float current_distance = 0.0f;
  float previous_distance = 0.0f;
  int loop_count = 0;
  const float distance_threshold = target_meters * 0.95f; // 提前5%开始减速
  
  // 计算预测制动距离（基于当前速度的经验值）
  float braking_distance = (speed / 255.0f) * 0.05f; // 简单线性模型，最大制动距离0.05米
  
  while (current_distance < target_meters) {
    current_distance = get_average_distance();
    
    // 检查是否需要减速
    if (current_distance >= (target_meters - braking_distance)) {
      // 逐渐减速到停止
      int reduced_speed = (int)(motor_speed_ * (target_meters - current_distance) / braking_distance);
      if (reduced_speed < 30) reduced_speed = 30; // 保持最小速度防止电机停转
      motor_driver_.Forward(reduced_speed);
    }
    
    ESP_LOGI(TAG, "当前距离: %.3f米, 目标距离: %.3f米", current_distance, target_meters);
    
    // 添加详细的调试信息
    int left_count = left_encoder_.get_pulse_count();
    int right_count = right_encoder_.get_pulse_count();
    float left_distance = left_encoder_.get_distance();
    float right_distance = right_encoder_.get_distance();
    
    ESP_LOGI(TAG, "详细编码器信息 - 左轮: 脉冲=%d, 距离=%.3f米 | 右轮: 脉冲=%d, 距离=%.3f米", 
             left_count, left_distance, right_count, right_distance);
    
    // 检查编码器状态（通过公共接口函数）
    check_and_log_encoders();
    
    // 检查是否卡住或未移动
    if (loop_count++ > 100) { // 1秒超时
      ESP_LOGW(TAG, "移动超时，可能被卡住");
      break;
    }
    
    // 检查是否停止移动
    if (loop_count > 20 && current_distance == previous_distance) {
      ESP_LOGW(TAG, "检测到机器人停止移动");
      break;
    }
    
    previous_distance = current_distance;
    vTaskDelay(pdMS_TO_TICKS(5));  // 提高检测频率到5ms一次
  }

  // 到达目标距离，停止电机
  motor_driver_.Stop();
  ESP_LOGI(TAG, "已到达目标距离: %.2f米", target_meters);
  
  // 显示最终编码器状态
  ESP_LOGI(TAG, "最终编码器状态 - 左轮脉冲: %d, 右轮脉冲: %d", 
           left_encoder_.get_pulse_count(), 
           right_encoder_.get_pulse_count());
}

int MotorControl::GetMotorSpeed() const { 
  return motor_speed_; 
}

int MotorControl::GetRunTime() const { 
  return run_time_; 
}

// C接口函数实现
extern "C" void motor_control_init() {
  if (g_motor_control == nullptr) {
    g_motor_control = new MotorControl();
    g_motor_control->InitializeTools();
  }
}

extern "C" void motor_forward(int speed, int time) {
  if (g_motor_control != nullptr) {
    g_motor_control->Forward(speed, time);
  }
}

extern "C" void motor_backward(int speed, int time) {
  if (g_motor_control != nullptr) {
    g_motor_control->Backward(speed, time);
  }
}

extern "C" void motor_turn_left(int speed, int time) {
  if (g_motor_control != nullptr) {
    g_motor_control->TurnLeft(speed, time);
  }
}

extern "C" void motor_turn_right(int speed, int time) {
  if (g_motor_control != nullptr) {
    g_motor_control->TurnRight(speed, time);
  }
}

extern "C" void motor_stop() {
  if (g_motor_control != nullptr) {
    g_motor_control->Stop();
  }
}

extern "C" int motor_get_speed() {
  if (g_motor_control != nullptr) {
    return g_motor_control->GetMotorSpeed();
  }
  return 0;
}

extern "C" int motor_get_run_time() {
  if (g_motor_control != nullptr) {
    return g_motor_control->GetRunTime();
  }
  return 0;
}