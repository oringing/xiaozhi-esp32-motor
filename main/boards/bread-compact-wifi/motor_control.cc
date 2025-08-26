#include "motor_control.h"

#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "mcp_server.h"

#define TAG "MotorControl"

// 全局电机控制实例定义
MotorControl* g_motor_control = nullptr;

MotorControl::MotorControl()
    : motor_driver_(GPIO_NUM_9, GPIO_NUM_10,  // 1号电机
                    GPIO_NUM_11, GPIO_NUM_12,  // 2号电机
                    GPIO_NUM_13, GPIO_NUM_14,  // 3号电机
                    GPIO_NUM_8, GPIO_NUM_3,    // 4号电机
                    GPIO_NUM_4, GPIO_NUM_5) {  // ENA/ENB引脚
  // 初始化电机驱动
  motor_driver_.Init();

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
  mcp_server.AddTool(
      "self.motor.forward", "控制电机前进。speed: 速度(0-255); time: 运行时间(0-100秒)",
      PropertyList({Property("speed", kPropertyTypeInteger, 100, 0, 255),
                    Property("time", kPropertyTypeInteger, 2, 0, 100)}),
      [this](const PropertyList& properties) -> ReturnValue {
        int speed = properties["speed"].value<int>();
        int time = properties["time"].value<int>();
        Forward(speed, time);
        return "电机前进，速度：" + std::to_string(speed) + "，时间：" +
               std::to_string(time) + "秒";
      });

  // 后退控制工具
  mcp_server.AddTool(
      "self.motor.backward", "控制电机后退。speed: 速度(0-255); time: 运行时间(0-100秒)",
      PropertyList({Property("speed", kPropertyTypeInteger, 100, 0, 255),
                    Property("time", kPropertyTypeInteger, 2, 0, 100)}),
      [this](const PropertyList& properties) -> ReturnValue {
        int speed = properties["speed"].value<int>();
        int time = properties["time"].value<int>();
        Backward(speed, time);
        return "电机后退，速度：" + std::to_string(speed) + "，时间：" +
               std::to_string(time) + "秒";
      });

  // 左转控制工具
  mcp_server.AddTool(
      "self.motor.turn_left", "控制电机左转。speed: 速度(0-255); time: 运行时间(0-100秒)",
      PropertyList({Property("speed", kPropertyTypeInteger, 100, 0, 255),
                    Property("time", kPropertyTypeInteger, 2, 0, 100)}),
      [this](const PropertyList& properties) -> ReturnValue {
        int speed = properties["speed"].value<int>();
        int time = properties["time"].value<int>();
        TurnLeft(speed, time);
        return "电机左转，速度：" + std::to_string(speed) + "，时间：" +
               std::to_string(time) + "秒";
      });

  // 右转控制工具
  mcp_server.AddTool(
      "self.motor.turn_right", "控制电机右转。speed: 速度(0-255); time: 运行时间(0-100秒)",
      PropertyList({Property("speed", kPropertyTypeInteger, 100, 0, 255),
                    Property("time", kPropertyTypeInteger, 2, 0, 100)}),
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

int MotorControl::GetMotorSpeed() const { return motor_speed_; }

int MotorControl::GetRunTime() const { return run_time_; }

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