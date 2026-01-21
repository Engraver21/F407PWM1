#ifndef DATA_PRO_H_
#define DATA_PRO_H_
#include "rplidar_c1.h"
#include <stdint.h>

// 定义存放角度、距离和质量的结构体
typedef struct {
    uint16_t ind;
    uint16_t angle;   // 角度，单位为1/64度（需要根据实际数据转换）
    uint16_t distance; // 距离，单位为毫米（mm）
    uint8_t quality;  // 质量，表示数据的可靠性（0~255）
} LidarData_t;

// object_detect.h

// 定义一个物体
typedef struct {
    uint16_t id;            // 物体编号
    float angle_center;     // 物体中心角度
    float distance_min;     // 物体最近距离 (也就是离车头的距离)
    float distance_avg;     // 平均距离
    uint16_t start_index;   // 起始点索引
    uint16_t end_index;     // 结束点索引
    uint16_t point_count;   // 包含多少个点
} DetectedObject_t;

#define MAX_OBJECTS 10      // 最多同时识别10个物体
#define GAP_THRESHOLD 150   // 距离跳变阈值 (mm)，超过这个值认为不是同一个物体
#define MIN_POINTS 3        // 一个物体至少由3个点组成，小于3个认为是噪点

#define LIDAR_DATA_SIZE 256// 定义存储数据的数组大小

void data_collect(LidarData_t lid[], uint16_t count);
void Find_Objects(LidarData_t* data, uint16_t total_points);

void Print_Objects() ;
#endif /* DATA_PRO_H_ */
