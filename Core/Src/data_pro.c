#include "data_pro.h"
#include "key.h"
#include "usart.h" // 用于 UART 传输函数
#include <stdint.h>
#include <stdio.h>
// 创建一个结构体变量
LidarData_t lidar_data[LIDAR_DATA_SIZE] = {0};

void printf_inf_le(LidarData_t lid[])
{
    printf("开始打印新一轮信息\r\n");
    for (int i = 0; i<LIDAR_DATA_SIZE; i++) {
        uint8_t meg[64];

        sprintf((char*)meg, "INDEX: %u,A:%u°,D:%umm,Q:%u\r\n", 
                        (unsigned int)(i),
                        (unsigned int)(lid[i].angle/64), 
                        (unsigned int)(lid[i].distance/4),
                        (unsigned int)lid[i].quality);
        printf((uint8_t *)meg);
    }
}