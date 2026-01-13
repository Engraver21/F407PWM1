#ifndef BALL_SCREW_CONTRL_H
#define BALL_SCREW_CONTRL_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>



typedef struct Ball_Screw_position//滚珠丝杠的状态结构体
{
    bool upper_limit_sta; // 上限位状态
    bool lower_limit_sta; // 下限位状态
    uint8_t position_sta;    // 当前位置状态
    uint8_t current_posi;   // 当前位置信息
    uint8_t target_posi;     // 目标位置
    uint8_t move_dir;        // 移动方向
}Ball_Screw_position_t;

typedef struct Screw//丝杠的信息结构体
{
    uint8_t Pitch; // 螺距
    uint8_t Lead;  // 导程
}Screw_info_t;

typedef struct Motor//电机的信息结构体
{
    uint8_t motor_id; // 电机ID
    uint8_t Degree_similarity; // 相数
    uint8_t Segmentation; //细分
    float Step_angle; // 步距角
    uint16_t PULSES_PER_REV; // 转一圈需要的脉冲数
    uint16_t steps_left; // 剩余步数
    uint8_t direction; // 运动方向0:停止 1:正转 2:反转
}Motor_info_t;
// 全局变量声明
extern Ball_Screw_position_t ball_screw_pos; // 滚珠丝杠状态变量
extern Screw_info_t screw_info;              // 丝杠信息变量
extern Motor_info_t motor_39;//  电机信息变量
extern Motor_info_t motor_57;//  电机信息变量
/* 函数声明 */
void Ball_Screw_init(void);//滚珠丝杠初始化函数
void Ball_screw_contrl(void);//检测滚珠丝杠是否达到限位
#endif