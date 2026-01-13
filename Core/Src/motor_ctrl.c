#include "motor_ctrl.h"
#include "stdio.h"
#include "usart.h"
#include "delay.h"
#include "Ball_screw_contrl.h"

/**
 * @brief  通用的设置 PWM 频率和占空比函数
 * @param  htim: 定时器句柄 (如 &htim3)
 * @param  Channel: 通道 (如 TIM_CHANNEL_1)
 * @param  freq_hz: 目标频率 (Hz)
 * @param  duty_percent: 目标占空比 (0-100)
 */
void PWM_Set_Params(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t freq_hz, uint8_t duty_percent)
{
    if (freq_hz == 0) return; // 防止除以0

    // 1. 计算新的自动重装载值 (ARR) -> 决定频率
    // 公式: ARR = (时钟频率 / 目标频率) - 1
    uint32_t new_period = (TIMER_CLOCK_FREQ / freq_hz) - 1;

    // 2. 计算新的比较值 (CCR) -> 决定占空比
    // 公式: CCR = (ARR + 1) * (占空比 / 100)
    uint32_t new_pulse = (new_period + 1) * duty_percent / 100;

    // 3. 写入寄存器
    __HAL_TIM_SET_AUTORELOAD(htim, new_period);       // 修改频率
    __HAL_TIM_SET_COMPARE(htim, Channel, new_pulse);  // 修改占空比
    
    // 4. 重要：如果定时器已经在运行，让 ARR 更改立即生效需要触发更新事件
    // 注意：这可能会稍微打断一下当前的波形，但能保证频率立即切换
    // htim->Instance->EGR = TIM_EGR_UG; 
}

// ==========================================
//              大电机 (PD12) 控制函数
// ==========================================

// 开启大电机
void BigMotor_Start(void)
{
    HAL_TIM_PWM_Start(BIG_MOTOR_TIM, BIG_MOTOR_CHAN);
}

// 关闭大电机
void BigMotor_Stop(void)
{
    HAL_TIM_PWM_Stop(BIG_MOTOR_TIM, BIG_MOTOR_CHAN);
}

// 设置大电机参数
// 例: BigMotor_Set(1000, 50); // 1000Hz, 50% 占空比
void BigMotor_Set(uint32_t freq_hz, uint8_t duty_percent)
{
    PWM_Set_Params(BIG_MOTOR_TIM, BIG_MOTOR_CHAN, freq_hz, duty_percent);
}

// ==========================================
//              小电机 (PC6) 控制函数
// ==========================================

// 开启小电机
void LittleMotor_Start(void)
{
    HAL_TIM_PWM_Start(LITTLE_MOTOR_TIM, LITTLE_MOTOR_CHAN);
}

// 关闭小电机
void LittleMotor_Stop(void)
{
    HAL_TIM_PWM_Stop(LITTLE_MOTOR_TIM, LITTLE_MOTOR_CHAN);
}

// 设置小电机参数
void LittleMotor_Set(uint32_t freq_hz, uint8_t duty_percent)
{
    PWM_Set_Params(LITTLE_MOTOR_TIM, LITTLE_MOTOR_CHAN, freq_hz, duty_percent);
}


// 初始化状态
static MotorState_t sys_state = {
    .is_big_motor_running = false, // 初始假设大电机是关的
    .is_lit_motor_running = false, // 初始假设小电机是关的
    .speed_level = 1,
    .frequency = 2000
};
// ==========================================
//          按键控制电机函数
// ==========================================
void control_motor(uint8_t key)
{
    switch (key) {
        case 1: // 大电机开关
            if (sys_state.is_big_motor_running) {
                BigMotor_Stop();
                sys_state.is_big_motor_running = false;
            } else {
                BigMotor_Start();
                sys_state.is_big_motor_running = true;
            }
            printf("Big Motor: %s\r\n", sys_state.is_big_motor_running ? "ON" : "OFF");
            break;

        case 2: // 小电机开关
            if (sys_state.is_lit_motor_running) {
                LittleMotor_Stop();
                sys_state.is_lit_motor_running = false;
            } else {
                LittleMotor_Start();
                sys_state.is_lit_motor_running = true;
            }
            printf("Little Motor: %s\r\n", sys_state.is_lit_motor_running ? "ON" : "OFF");
            break;

        case 3: // 调速
        {   
            
            // 1. 先切换到下一个档位 (这样按下按钮立刻变数组)
            sys_state.speed_level = (sys_state.speed_level % MAX_MULIT) + 1;

            // 2. 计算新频率
            uint32_t new_freq = BASE_FREQ * sys_state.speed_level;
            sys_state.frequency = new_freq; // 顺便更新结构体里的频率记录

            // 3. 应用参数
            BigMotor_Set(new_freq, DUTY_CYCLE);
            LittleMotor_Set(new_freq, DUTY_CYCLE);

            // 4. 安全冗余：防止Set函数意外启动了电机
            // 如果电机原本是关的，修改参数后强制确保它还是关的
            if (!sys_state.is_big_motor_running) BigMotor_Stop();
            if (!sys_state.is_lit_motor_running) LittleMotor_Stop();

            // 在变量前面加 (int)
            printf("Speed Level: %d, Freq: %d Hz\r\n", sys_state.speed_level, (int)new_freq);
            break;
        }
        
        default:
            break;
    }
}


// ==========================================
// 3. 核心函数：指定圈数与速度
// ==========================================
/* USER CODE BEGIN 0 */

// 控制 39 电机 (小电机)
// turns: 圈数 (可以是小数，如 2.5 圈)
// freq_hz: 速度 (Hz)
void Motor39_Move(float turns, uint32_t freq_hz)
{
    if (turns < 0) return; // 简单起见，暂不处理反转方向脚，假设只正转

    // 1. 计算总脉冲数
    motor_39.steps_left = (int32_t)(turns * motor_39.PULSES_PER_REV);

    // 2. 设置频率 (直接调用你之前的 PWM_Set_Params 函数)
    // 占空比一般给 50% 即可
    PWM_Set_Params(&htim3, TIM_CHANNEL_1, freq_hz, 50);

    // 3. 使用 _IT (中断模式) 启动 PWM
    // 注意：这里必须用 HAL_TIM_PWM_Start_IT，不能用普通的 Start
    HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
}

// 控制 57 电机 (大电机)
void Motor57_Move(float turns, uint32_t freq_hz)
{
    if (turns < 0) return;

    // 1. 计算总脉冲数
    motor_57.steps_left = (int32_t)(turns * motor_57.PULSES_PER_REV);

    // 2. 设置频率
    PWM_Set_Params(&htim4, TIM_CHANNEL_1, freq_hz, 50);

    // 3. 启动 PWM 中断
    HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1);
}

/* USER CODE END 0 */