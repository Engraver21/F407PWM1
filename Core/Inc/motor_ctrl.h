#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

#include "main.h"
#include "tim.h" // 确保包含这个，以便使用 htim3 和 htim4
#include <stdbool.h> // 引入bool类型
// 定义常量，方便日后修改参数
#define BASE_FREQ      2000  // 基础频率 2000Hz
#define DUTY_CYCLE     50    // 占空比 50%
#define MAX_MULIT      5     // 最大倍频数

// 定义电机状态结构体
typedef struct {
    bool is_big_motor_running;
    bool is_lit_motor_running;
    uint8_t speed_level; // 1-5 档位
    uint32_t frequency;
} MotorState_t;



// 定义电机对应的定时器句柄，方便后续修改
// 如果 PC6 你配置的是 TIM8，请把这里改成 &htim8
#define BIG_MOTOR_TIM     &htim4    // PD12
#define BIG_MOTOR_CHAN    TIM_CHANNEL_1

#define LITTLE_MOTOR_TIM  &htim3    // PC6
#define LITTLE_MOTOR_CHAN TIM_CHANNEL_1


// 假设：你的定时器预分频 (PSC) 设置让计数器时钟为 1MHz (1us跳一次)
// 如果你的 PSC 是 83 (84MHz主频下)，那么计数频率就是 1MHz
#define TIMER_CLOCK_FREQ  1000000

void PWM_Set_Params(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t freq_hz, uint8_t duty_percent);// 设置 PWM 频率和占空比
void BigMotor_Start(void);// 开启大电机
void BigMotor_Stop(void);// 关闭大电机
void BigMotor_Set(uint32_t freq_hz, uint8_t duty_percent);// 设置大电机参数
void LittleMotor_Start(void);// 开启小电机
void LittleMotor_Stop(void);// 关闭小电机
void LittleMotor_Set(uint32_t freq_hz, uint8_t duty_percent);// 设置小电机参数

void control_motor(uint8_t key);// 按键控制电机函数


#endif