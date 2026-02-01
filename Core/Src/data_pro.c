#include "data_pro.h"
#include "key.h"
#include "usart.h" // 用于 UART 传输函数
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "rplidar_c1.h"

extern  uint16_t point_index ;// 用于存储点的索引

// 建议将常数提取，增加运算效率
#define RPLIDAR_ANG_SCALE  64.0f
#define RPLIDAR_DIST_SCALE 4.0f
#define DEG2RAD            0.0174532925f // (PI / 180.0f)

// 实际开辟内存空间
LidarData_t lidar_buffer_A[LIDAR_MAX_POINTS];
LidarData_t lidar_buffer_B[LIDAR_MAX_POINTS];

// 初始化指针指向
LidarData_t* writing_ptr = lidar_buffer_A;
LidarData_t* reading_ptr = lidar_buffer_B;

uint16_t ready_point_count = 0;
volatile bool beg_co_sig = false;
void data_collect(LidarData_t* lid, uint16_t count) {
    for (int i = 0; i < count; i++) {
        // 1. 过滤无效点：思岚 A1 的距离为 0 或品质分度不够时应跳过
        // 注：lid[i].distance 是原始 16 位数据，低 2 位是 Check 位，高 14 位是距离
        // 如果你的底层驱动已经处理了移位，则直接判断 > 0
        if (lid[i].distance > 0) {
            
            // 2. 转换为实际物理量
            float angle_deg = (float)lid[i].angle / RPLIDAR_ANG_SCALE;
            float dist_mm   = (float)lid[i].distance / RPLIDAR_DIST_SCALE;

            // 3. 极坐标转直角坐标（修正顺时针镜像问题）
            // 思岚 A1 顺时针旋转，为了在绘图软件中得到正向图形：
            // 方案 A：使用 360 - angle
            // 方案 B：直接在 sin 计算时加负号
            float angle_rad = angle_deg * DEG2RAD;

            float x = dist_mm * cosf(angle_rad);
            float y = dist_mm * sinf(angle_rad); // 加负号修正镜像

            // 4. VOFA+ FireWater 协议：X,Y\n
            printf("%.2f,%.2f\n", x, y);
        }
    }
}


// 辅助：计算两点距离平方（不开根号，速度快）
float dist_sq(float x1, float y1, float x2, float y2) {
    return (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2);
}


void Lidar_Analyze_Objects(LidarData_t* buffer, uint16_t count) {
    // 1. 初始化
    LidarObject_t objects[MAX_OBJECTS];
    int obj_count = 0; // 当前找到的物体数量

    // 聚类临时变量
    float sum_x = 0, sum_y = 0;
    int current_pts = 0;
    float last_x = 0, last_y = 0;

    // 清空对象数组，防止残留数据
    memset(objects, 0, sizeof(objects));

    for (int i = 0; i < count; i++) {
        // --- 过滤无效点 ---
        if (buffer[i].distance == 0) continue; 

        // --- 坐标转换 ---
        float angle_deg = buffer[i].angle / 64.0f;
        float dist_mm   = buffer[i].distance / 4.0f;
        float angle_rad = angle_deg * 0.01745329f; // PI / 180

        // 坐标系修正 (根据你的情况 X, Y)
        float x = dist_mm * cosf(angle_rad);
        float y = -dist_mm * sinf(angle_rad); 

        // --- 聚类逻辑 ---
        if (current_pts == 0) {
            // 新簇的第一个点
            sum_x = x;
            sum_y = y;
            current_pts = 1;
        } 
        else {
            // 计算与上一个点的距离平方
            float d2 = (x - last_x)*(x - last_x) + (y - last_y)*(y - last_y);
            
            if (d2 < (CLUSTER_THRESHOLD * CLUSTER_THRESHOLD)) {
                // 距离很近，属于同一个物体 -> 累加
                sum_x += x;
                sum_y += y;
                current_pts++;
            } 
            else {
                // --- 距离突变！结算【上一个】物体 (例如 Obj0, Obj1) ---
                if (current_pts >= MIN_POINTS_PER_OBJ && obj_count < MAX_OBJECTS) {
                    objects[obj_count].x = sum_x / current_pts;
                    objects[obj_count].y = sum_y / current_pts;
                    objects[obj_count].points = current_pts;
                    // 计算距离
                    objects[obj_count].distance = sqrtf(objects[obj_count].x * objects[obj_count].x + 
                                                        objects[obj_count].y * objects[obj_count].y);
                    obj_count++; // 索引加1
                }
                
                // 开启【新】物体 (例如开始 Obj2)
                sum_x = x;
                sum_y = y;
                current_pts = 1;
            }
        }
        
        last_x = x;
        last_y = y;
    }

    // ==========================================================
    // 【核心修正点】: 循环结束后的收尾工作
    // 这里专门负责结算【最后一个物体】(也就是你的 Obj2)
    // ==========================================================
    if (current_pts >= MIN_POINTS_PER_OBJ && obj_count < MAX_OBJECTS) {
        // 1. 存入 X 和 Y
        objects[obj_count].x = sum_x / current_pts;
        objects[obj_count].y = sum_y / current_pts;
        objects[obj_count].points = current_pts;
        
        // 2. 【关键】必须在这里也算一次距离！
        objects[obj_count].distance = sqrtf(objects[obj_count].x * objects[obj_count].x + 
                                            objects[obj_count].y * objects[obj_count].y);
        
        obj_count++; // 结算完成
    }


    // --- 打印输出 ---
    printf("Found %d objects:\n", obj_count);
    for(int k=0; k<obj_count; k++) {
        // 这里的 Dist 现在肯定有值了
        printf("Obj%d: X:%.1f Y:%.1f Dist:%.1f\n", k, objects[k].x, objects[k].y, objects[k].distance);
    }
}