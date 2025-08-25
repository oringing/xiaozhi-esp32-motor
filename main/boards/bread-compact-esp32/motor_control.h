#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "mcp_server.h"
#include "driver/gpio.h"
#include "motor/motor.h"

class MotorControl {
private:
    TimerHandle_t stop_timer_ = nullptr;
    MotorDriver motor_driver_;
    int motor_speed_ = 0;
    int run_time_ = 2;

    static void StopMotorTimerCallback(TimerHandle_t timer);

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
    int GetMotorSpeed() const;
    int GetRunTime() const;
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

#endif  // MOTOR_CONTROL_H_