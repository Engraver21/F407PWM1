#include "data_pro.h"
#include "key.h"
#include "usart.h" // 用于 UART 传输函数
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
#include "rplidar_c1.h"
// 创建一个结构体变量
LidarData_t lidar_data[LIDAR_DATA_SIZE] = {0};
extern  uint16_t point_index ;// 用于存储点的索引

// 定义 PI，如果 math.h 里没有定义 M_PI
#ifndef M_PI
#define M_PI 3.1415926f
#endif

void data_collect(LidarData_t lid[], uint16_t count)
{
    // VOFA+ 不需要 "--- New Scan ---" 这种文本头，
    // 如果你保留它，VOFA+ 会把它当做日志忽略，不影响绘图，但建议注释掉以节省带宽
    // printf("--- New Scan: %d points ---\r\n", count);
    
    for (int i = 0; i < count; i++) { 
        
        // 1. 过滤无效点 (质量为0 或者 距离为0)
        if (lid[i].quality > 0 && lid[i].distance > 0) {
            
            // 2. 转换为浮点数实际值
            // 角度: Q6格式 -> 实际度数
            float angle_deg = lid[i].angle / 64.0f; 
            
            // 距离: Q2格式 -> 毫米
            float dist_mm = lid[i].distance / 4.0f;

            // 3. 极坐标转直角坐标 (Polar -> Cartesian)
            // math.h 的三角函数使用弧度制，所以需要 角度 * PI / 180
            float angle_rad = angle_deg * M_PI / 180.0f;

            // 计算 X, Y
            // 注意：这是标准数学坐标系。
            // 如果你在 VOFA+ 里看到图是歪的，可以尝试交换 sin/cos 或加负号
            float x = dist_mm * cos(angle_rad);
            float y = dist_mm * sin(angle_rad);

            // 4. 【关键】使用 FireWater 协议格式打印
            // 格式： "X坐标,Y坐标\n"
            // %.1f 保留1位小数足够了，节省串口带宽
            printf("%.1f,%.1f\n", x, y); 
        }
    }
    
    // VOFA+ 同样不需要结束符，为了干净的数据流，建议注释掉
    // printf("--- End ---\r\n");
}


DetectedObject_t objects[MAX_OBJECTS];
uint8_t object_count = 0;

void Find_Objects(LidarData_t* data, uint16_t total_points) {
    object_count = 0;// 重置对象计数器
    
    // 临时变量，用于记录当前正在跟踪的物体
    uint16_t current_obj_start_idx = 0;// 当前正在处理的物体的起始索引
    uint16_t current_obj_point_count = 0;// 当前正在处理的物体的点数
    float current_obj_dist_sum = 0;// 当前正在处理的物体的距离和
    float current_obj_min_dist = 10000.0f;// 当前正在处理的物体的最小距离
    
    // 简单的状态机：0=寻找新物体, 1=正在记录物体
    uint8_t state = 0; 

    for (int i = 1; i < total_points; i++) {//
        // 1. 获取当前点和上一个点的距离
        float dist_curr = data[i].distance / 4.0f;     // 转换为毫米
        float dist_prev = data[i-1].distance / 4.0f;   // 毫秒
        float angle_curr = data[i].angle / 64.0f;       // 度
        float angle_prev = data[i-1].angle / 64.0f;     // 度

        // 过滤掉无效点 (距离为0)
        if (dist_curr < 10 || dist_prev < 10) continue;

        // 2. 计算跳变
        float dist_diff = abs(dist_curr - dist_prev);//计算两点之间的距离差
        float angle_diff = angle_curr - angle_prev;// 计算角度差
        if(angle_diff < 0) angle_diff += 360; // 处理过零点情况

        // 判断是否属于同一个物体：距离跳变小 且 角度连续（例如断层不大于5度）
        bool is_same_object = (dist_diff < GAP_THRESHOLD) && (angle_diff < 5.0f);

        if (state == 0) {
            // 状态：开始新物体
            current_obj_start_idx = i - 1; // 从上一个点开始算
            current_obj_point_count = 1;    //当前物体的点数
            current_obj_dist_sum = dist_prev;// 当前物体的距离和
            current_obj_min_dist = dist_prev;// 当前物体的最小距离
            state = 1;                      // 进入状态：正在记录物体
        } 
        else if (state == 1) { // 状态：正在记录物体
            if (is_same_object) {// 还是同一个物体，累加数据
                current_obj_point_count++;// 当前物体的点数加1
                current_obj_dist_sum += dist_curr;// 当前物体的距离和加当前点
                if (dist_curr < current_obj_min_dist) // 更新最小距离
                current_obj_min_dist = dist_curr;
            } 
            else {
                // 跳变发生！上一个物体结束了
                // 只有当点数足够多时，才算有效物体（过滤噪点）
                if (current_obj_point_count >= MIN_POINTS) {// 至少有3个点
                    if (object_count < MAX_OBJECTS) {// 最多只保存10个物体
                        objects[object_count].id = object_count + 1;
                        // 计算中心角度 = (起始角 + 结束角) / 2
                        float start_ang = data[current_obj_start_idx].angle / 64.0f;
                        float end_ang = data[i-1].angle / 64.0f;
                        
                        // 简单的角度平均处理（注意360度过零问题这里暂简略处理）
                        objects[object_count].angle_center = (start_ang + end_ang) / 2.0f;
                        
                        objects[object_count].distance_min = current_obj_min_dist;
                        objects[object_count].distance_avg = current_obj_dist_sum / current_obj_point_count;
                        objects[object_count].point_count = current_obj_point_count;
                        
                        object_count++;
                    }
                }
                // 重置状态，开始判断当前点是不是下一个物体的起点
                state = 0; 
                // 为了简单，不回退i，直接让下一次循环处理新物体
            }
        }
    }
}

// 打印识别结果
void Print_Objects() {
    printf("\r\n=== DETECTED %d OBJECTS ===\r\n", object_count);
    for (int k = 0; k < object_count; k++) {
        printf("OBJ#%d: Angle:%.1f deg, Dist:%.0f mm (Width/Pts:%d)\r\n", 
            objects[k].id, 
            objects[k].angle_center, 
            objects[k].distance_min,
            objects[k].point_count);
    }
}