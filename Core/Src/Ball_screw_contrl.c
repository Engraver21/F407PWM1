#include <stdbool.h>
#include "motor_ctrl.h"
#include "stdio.h"
#include "usart.h"
#include "Ball_screw_contrl.h"


Ball_Screw_position_t ball_screw_pos;//滚珠丝杠状态变量
Screw_info_t screw_info;//丝杠信息变量
Motor_info_t motor_39;//  电机信息变量
Motor_info_t motor_57;//  电机信息变量

void Ball_Screw_init(void)
{
    // 初始化滚珠丝杠相关的GPIO引脚
    // 上限位和下限位开关已经在 gpio.c 里初始化为输入上拉
    // 如果需要其他初始化代码，可以在这里添加

    ball_screw_pos.upper_limit_sta = HAL_GPIO_ReadPin(Microswitch_Up_GPIO_Port, Microswitch_Up_Pin);
    ball_screw_pos.lower_limit_sta = HAL_GPIO_ReadPin(Microswitch_Down_GPIO_Port, Microswitch_Down_Pin);
    ball_screw_pos.position_sta = 0; // 初始位置状态
    ball_screw_pos.current_posi = 0; // 当前位置信息
    ball_screw_pos.target_posi = 0;  // 目标位置
    ball_screw_pos.move_dir = 0;     // 移动方向

    printf("Ball Screw Initialized. Upper Limit: %d, Lower Limit: %d\r\n", 
           ball_screw_pos.upper_limit_sta, ball_screw_pos.lower_limit_sta);

    screw_info.Pitch = 5; // 螺距，单位毫米
    screw_info.Lead = 10; // 导程，单位毫米
    printf("Screw Info - Pitch: %d, Lead: %d\r\n", screw_info.Pitch, screw_info.Lead);

    motor_39.motor_id = 39; // 电机ID
    motor_39.Degree_similarity = 2; // 相数   
    motor_39.Segmentation = 32;     // 细分
    motor_39.Step_angle = 1.8;      // 步距角，单位度
    motor_39.PULSES_PER_REV = 200*motor_39.Segmentation;  // 转一圈需要的脉冲数
    motor_39.steps_left = 0;        // 剩余步数
    motor_39.direction = 0;         // 停止
    printf("Motor Info - Degree Similarity: %d, Segmentation: %d, Step Angle: %.1f\r\n", 
           motor_39.Degree_similarity, motor_39.Segmentation, motor_39.Step_angle);
    motor_57.motor_id = 57; // 电机ID
    motor_57.Degree_similarity = 2; // 相数   
    motor_57.Segmentation = 64;     // 细分    
    motor_57.Step_angle = 1.8;      // 步距角，单位度
    motor_57.PULSES_PER_REV = 200*motor_57.Segmentation;  // 转一圈需要的脉冲数
    motor_57.steps_left = 0;        // 剩余步数
    motor_57.direction = 0;         // 停止
    printf("Motor Info - Degree Similarity: %d, Segmentation: %d, Step Angle: %.1f\r\n", 
           motor_57.Degree_similarity, motor_57.Segmentation, motor_57.Step_angle);
}


/* 碰到上下的滚珠丝杠停止电机*/

void Ball_screw_contrl(void)
{
    if(HAL_GPIO_ReadPin(Microswitch_Up_GPIO_Port, Microswitch_Up_Pin) == GPIO_PIN_RESET) // 上限位
    {
        BigMotor_Stop();
        LittleMotor_Stop();
        ball_screw_pos.upper_limit_sta = true;
        printf("Reached Upper Limit! Motors Stopped.\r\n");
    }
    if(HAL_GPIO_ReadPin(Microswitch_Down_GPIO_Port, Microswitch_Down_Pin) == GPIO_PIN_RESET) // 下限位
    {
        BigMotor_Stop();
        LittleMotor_Stop();
        ball_screw_pos.lower_limit_sta = true;
        printf("Reached Lower Limit! Motors Stopped.\r\n");
    }
}

bool rest_ball_screw_pos(void)
{
    ball_screw_pos.upper_limit_sta = HAL_GPIO_ReadPin(Microswitch_Up_GPIO_Port, Microswitch_Up_Pin);
    ball_screw_pos.lower_limit_sta = HAL_GPIO_ReadPin(Microswitch_Down_GPIO_Port, Microswitch_Down_Pin);
    if(ball_screw_pos.upper_limit_sta == false && ball_screw_pos.lower_limit_sta == false)
    {
        return true;
    }
    else
    {
        return false;
    }
}