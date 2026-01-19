#ifndef INC_RPLIDAR_C1_H_
#define INC_RPLIDAR_C1_H_

#include "main.h" // 根据您使用的 MCU 系列进行修改（例如：stm32g4xx_hal.h）
#include <stdint.h>
#include <stdbool.h> // 用于 bool 类型
#include <stdio.h>

// --- 配置 ---
#define LIDAR_DMA_BUFFER_SIZE 4096 // DMA 缓冲区大小（从 main.c 移入）
// --- 配置结束 ---

// LIDAR 状态（从 main.c 移入）
typedef enum {
    LIDAR_STATE_IDLE,
    WAITING_FOR_DESCRIPTOR_A5,
    WAITING_FOR_DESCRIPTOR_5A,
    RECEIVING_DESCRIPTOR,
    RECEIVING_SCAN_PACKET
} RPLIDAR_State_TypeDef;

// LIDAR 库的主数据结构
typedef struct {
    // --- HAL 句柄（通过 Init 赋值）---
    UART_HandleTypeDef* lidar_uart; // 连接 LIDAR 的 UART（例如：&huart1）
    UART_HandleTypeDef* pc_uart;    // 连接 PC 的 UART（例如：&huart2）
    DMA_HandleTypeDef* lidar_dma;   // LIDAR UART RX DMA（例如：&hdma_usart1_rx）

    // --- 内部状态和缓冲区 ---
    volatile RPLIDAR_State_TypeDef state;      // 当前状态机状态
    volatile bool       is_active;      // LIDAR 是否处于活动状态（开始/停止命令）
    uint8_t             dma_buffer[LIDAR_DMA_BUFFER_SIZE]; // 保存 DMA 数据的缓冲区
    uint32_t            dma_read_index; // DMA 缓冲区读取索引
    uint8_t             packet_buffer[5]; // 保存接收到的 5 字节数据包（描述符/数据）
    uint8_t             packet_index;   // packet_buffer 的索引
    uint8_t             pc_rx_byte;     // 来自 PC 的单字节命令（'s' 或 'x'）
    uint32_t            packet_counter; // 数据降采样计数器

    // --- 过滤器设置 ---
    uint16_t            filter_start_angle_x64; // 起始角度限制（度数 * 64）
    uint16_t            filter_end_angle_x64;   // 结束角度限制（度数 * 64）
    bool                filter_wrap_around;     // 角度过滤器是否跨越 0 度？（使用 || 还是 && 逻辑？）
    uint16_t            filter_min_dist_x4;     // 最小距离限制（mm * 4）
    uint16_t            filter_max_dist_x4;     // 最大距离限制（mm * 4）


        // --- 平均值计算（新增部分）---
        volatile uint32_t   total_distance_sum;   // 一圈内有效距离的总和 (x4)
        volatile uint16_t   total_distance_count; // 一圈内有效点的数量
        volatile bool       new_revolution;     // 当完成新的一圈时变为 'true'
        volatile uint16_t   last_avg_distance_x4; // 计算出的上一次平均距离 (x4)
        volatile uint16_t   last_point_count;     // 上一次平均计算中使用的点数
} RPLIDAR_Handle_t;

// --- 库函数 ---

/**
 * @brief 初始化 LIDAR 库，设置 UART 和 DMA 句柄。
 * @param lidar: RPLIDAR_Handle_t 结构体指针。
 * @param lidar_uart_handle: 连接 LIDAR 的 UART 句柄（例如：&huart1）。
 * @param pc_uart_handle: 连接 PC 的 UART 句柄（例如：&huart2）。
 * @param lidar_dma_handle: LIDAR UART RX DMA 句柄（例如：&hdma_usart1_rx）。
 * @retval HAL_StatusTypeDef: HAL_OK（成功）或 HAL_ERROR。
 */
HAL_StatusTypeDef RPLIDAR_Init(RPLIDAR_Handle_t* lidar,
                               UART_HandleTypeDef* lidar_uart_handle,
                               UART_HandleTypeDef* pc_uart_handle,
                               DMA_HandleTypeDef* lidar_dma_handle);


void RPLIDAR_Process(RPLIDAR_Handle_t* lidar);

/**
 * @brief 发送开始 LIDAR 扫描的命令。
 * @param lidar: RPLIDAR_Handle_t 结构体指针。
 */
void RPLIDAR_StartScan(RPLIDAR_Handle_t* lidar);

/**
 * @brief 发送停止 LIDAR 扫描的命令。
 * @param lidar: RPLIDAR_Handle_t 结构体指针。
 */
void RPLIDAR_StopScan(RPLIDAR_Handle_t* lidar);

/**
 * @brief 设置角度过滤器参数。
 * @param lidar: RPLIDAR_Handle_t 结构体指针。
 * @param start_angle_x64: 起始角度（度数 * 64）。
 * @param end_angle_x64: 结束角度（度数 * 64）。
 * @param wrap_around: 如果范围跨越 0 度（例如：350-10）为 true，
 * 正常范围（例如：10-50）为 false。
 */
void RPLIDAR_SetAngleFilter(RPLIDAR_Handle_t* lidar,
                            uint16_t start_angle_x64,
                            uint16_t end_angle_x64,
                            bool wrap_around);

/**
 * @brief 设置距离过滤器参数。
 * @param lidar: RPLIDAR_Handle_t 结构体指针。
 * @param min_dist_x4: 最小有效距离（mm * 4）。
 * @param max_dist_x4: 最大有效距离（mm * 4）。
 */
void RPLIDAR_SetDistanceFilter(RPLIDAR_Handle_t* lidar,
                               uint16_t min_dist_x4,
                               uint16_t max_dist_x4);

/**
 * @brief 当 PC UART RX 中断到达时调用的函数（在 HAL_UART_RxCpltCallback 内部调用）。
 * @param lidar: RPLIDAR_Handle_t 结构体指针。
 * @param huart: 触发中断的 UART 句柄。
 */
void RPLIDAR_RxCallback(RPLIDAR_Handle_t* lidar, UART_HandleTypeDef *huart);


#endif /* INC_RPLIDAR_C1_H_ */