#ifndef DATA_PRO_H_
#define DATA_PRO_H_
#include "rplidar_c1.h"
#include <stdint.h>

// 定义存放角度、距离和质量的结构体
typedef struct {
    uint16_t angle;   // 角度，单位为1/64度（需要根据实际数据转换）
    uint16_t distance; // 距离，单位为毫米（mm）
    uint8_t quality;  // 质量，表示数据的可靠性（0~255）
} LidarData_t;

// 创建一个结构体，用于保存识别到的物体信息
typedef struct {
    float x;      // 物体中心 X (mm)
    float y;      // 物体中心 Y (mm)
    float width;  // 估算宽度 (mm) (可视面宽度)
    int   points; // 该物体包含了多少个激光点
    float distance; // 物体中心到雷达的距离
} LidarObject_t;

// 定义最大允许识别多少个物体（比如你只需要3个，定10个留余量）
#define MAX_OBJECTS 10 
// 定义聚类阈值：如果两点间距大于 50mm，视为不同物体
#define CLUSTER_THRESHOLD 30.0f 
// 定义最少点数：如果一个物体只有不到3个点，视为噪点过滤掉
#define MIN_POINTS_PER_OBJ 3


#define LIDAR_MAX_POINTS 4096 // A1一圈通常不到500点，512足够

// --- 声明外部变量，让所有文件都能看到同一个东西 ---
extern LidarData_t lidar_buffer_A[LIDAR_MAX_POINTS];
extern LidarData_t lidar_buffer_B[LIDAR_MAX_POINTS];
extern LidarData_t* writing_ptr; // 供雷达解析程序写入的缓冲区
extern LidarData_t* reading_ptr; // 供打印程序读取的缓冲区
extern uint16_t ready_point_count; // 这一圈实际有效点的数量
extern volatile bool beg_co_sig;   // 一圈完成标志


#define LIDAR_DATA_SIZE 4096// 定义存储数据的数组大小

// 新增：数据处理与刷新函数
void data_collect(LidarData_t* lid, uint16_t count);
// 新增：识别物体
void Lidar_Analyze_Objects(LidarData_t* buffer, uint16_t count);
// 新增：其他处理函数
void other_prtocess(uint16_t count, LidarObject_t buffer[],bool complete_sig);


#endif /* DATA_PRO_H_ */
