#ifndef DATA_PRO_H_
#define DATA_PRO_H_
#include "rplidar_c1.h"

// 定义存放角度、距离和质量的结构体
typedef struct {
    uint16_t angle;   // 角度，单位为1/64度（需要根据实际数据转换）
    uint16_t distance; // 距离，单位为毫米（mm）
    uint8_t quality;  // 质量，表示数据的可靠性（0~255）
} LidarData_t;

#define LIDAR_DATA_SIZE 256// 定义存储数据的数组大小

void printf_inf_le(LidarData_t lid[]);

#endif /* DATA_PRO_H_ */
