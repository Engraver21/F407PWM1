#include "rplidar_c1.h"
#include <stdio.h>
#include <string.h> 


static const uint8_t CMD_START_SCAN[] = {0xA5, 0x20};
static const uint8_t CMD_STOP_SCAN[]  = {0xA5, 0x25};
static const uint8_t EXPECTED_SCAN_DESCRIPTOR[] = {0x05, 0x00, 0x00, 0x40, 0x81};




HAL_StatusTypeDef RPLIDAR_Init(RPLIDAR_Handle_t* lidar,
                               UART_HandleTypeDef* lidar_uart_handle,//* 连接 LIDAR 的 UART 句柄（&huart6） */
                               UART_HandleTypeDef* pc_uart_handle,//* 连接 PC 的 UART 句柄（&huart1） */
                               DMA_HandleTypeDef* lidar_dma_handle)// 初始化 LIDAR 库
{
    if (!lidar || !lidar_uart_handle || !pc_uart_handle || !lidar_dma_handle) {
        return HAL_ERROR; // Geçersiz argümanlar

    }

 
    lidar->lidar_uart = lidar_uart_handle;
    lidar->pc_uart = pc_uart_handle;
    lidar->lidar_dma = lidar_dma_handle;

 
    lidar->state = LIDAR_STATE_IDLE;
    lidar->is_active = false;
    lidar->dma_read_index = 0;
    lidar->packet_index = 0;
    lidar->packet_counter = 0;

	lidar->total_distance_sum = 0;
	lidar->total_distance_count = 0;
	lidar->new_revolution = false;
	lidar->last_avg_distance_x4 = 0;
	lidar->last_point_count = 0;

 
    RPLIDAR_SetAngleFilter(lidar, 0, (360 * 64), false); // 0-360 derece
    RPLIDAR_SetDistanceFilter(lidar, (50 * 4), (3000 * 4)); // 50mm - 3000mm

    
    if (HAL_UART_Receive_IT(lidar->pc_uart, &lidar->pc_rx_byte, 1) != HAL_OK) {
        return HAL_ERROR;
    }

    
    if (HAL_UART_Receive_DMA(lidar->lidar_uart, lidar->dma_buffer, LIDAR_DMA_BUFFER_SIZE) != HAL_OK) {
    
        return HAL_ERROR;
    }

    return HAL_OK;
}

// void RPLIDAR_Process(RPLIDAR_Handle_t* lidar)// 主处理函数，需在主循环中定期调用
// {
//     uint32_t dma_write_index = LIDAR_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(lidar->lidar_dma);

//     while (lidar->dma_read_index != dma_write_index)
//     {
//         uint8_t current_byte = lidar->dma_buffer[lidar->dma_read_index];
//         lidar->dma_read_index = (lidar->dma_read_index + 1) % LIDAR_DMA_BUFFER_SIZE;

    
//         if (!lidar->is_active && lidar->state != LIDAR_STATE_IDLE) {
//              lidar->state = LIDAR_STATE_IDLE;
//         }
    
//         if(lidar->state == LIDAR_STATE_IDLE && !lidar->is_active){
//             continue;
//         }


//         switch (lidar->state)
//         {
//             case LIDAR_STATE_IDLE:
    
//                 break;

//             case WAITING_FOR_DESCRIPTOR_A5:
//                 if (current_byte == 0xA5) {
//                     lidar->state = WAITING_FOR_DESCRIPTOR_5A;
//                 }
//                 break;

//             case WAITING_FOR_DESCRIPTOR_5A:
//                 if (current_byte == 0x5A) {
//                     lidar->packet_index = 0;
//                     lidar->state = RECEIVING_DESCRIPTOR;
//                 } else if (current_byte != 0xA5) {
//                     lidar->state = WAITING_FOR_DESCRIPTOR_A5; 
//                 }
//                 break;

//             case RECEIVING_DESCRIPTOR:
//                 lidar->packet_buffer[lidar->packet_index++] = current_byte;
//                 if (lidar->packet_index >= 5) {
//                     if (memcmp(lidar->packet_buffer, EXPECTED_SCAN_DESCRIPTOR, 5) == 0) {
//                         const char msg_ok[] = "SCAN Descriptor OK.\r\n";
//                         HAL_UART_Transmit(lidar->pc_uart, (uint8_t*)msg_ok, sizeof(msg_ok) - 1, 20);
//                         lidar->packet_index = 0;
//                         lidar->state = RECEIVING_SCAN_PACKET;
//                     } else {
//                         const char msg_err[] = "HATA: Gecersiz Descriptor!\r\n";
//                         HAL_UART_Transmit(lidar->pc_uart, (uint8_t*)msg_err, sizeof(msg_err) - 1, 20);
//                         lidar->state = WAITING_FOR_DESCRIPTOR_A5; 
//                     }
//                 }
//                 break;

//             case RECEIVING_SCAN_PACKET:
                            
//                             lidar->packet_buffer[lidar->packet_index++] = current_byte;

                            
//                             if (lidar->packet_index >= 5) {

//                                 lidar->packet_index = 0; 

//                                 uint8_t sync_quality     = lidar->packet_buffer[0];
//                                 uint8_t angle_low_byte   = lidar->packet_buffer[1];
//                                 uint8_t sync_bit         = (sync_quality & 0x01);
//                                 uint8_t inverse_sync     = (sync_quality & 0x02) >> 1;
//                                 uint8_t check_bit        = (angle_low_byte & 0x01);

                                
//                                 if ((sync_bit != inverse_sync) && (check_bit == 1)) {

                                
//                                     if (sync_bit == 1 && lidar->total_distance_count > 10) { 
//                                     lidar->last_avg_distance_x4 = (uint16_t)(lidar->total_distance_sum / lidar->total_distance_count);
                                    
//                                     char avg_msg[64];
//                                     // 打印格式：平均距离(mm) | 有效点数
//                                     int len = sprintf(avg_msg, "Valid Scan -> Dist: %u mm, Pts: %u\r\n", 
//                                                 (unsigned int)(lidar->last_avg_distance_x4 / 4), 
//                                                 (unsigned int)lidar->total_distance_count);
                                    
//                                     HAL_UART_Transmit(lidar->pc_uart, (uint8_t*)avg_msg, len, 10);

//                                     // 清零
//                                     lidar->total_distance_sum = 0;
//                                     lidar->total_distance_count = 0;
//                                     }
//                                     else if (sync_bit == 1) {
//                                         // 虽然是新的一圈，但点数太少，认为是噪音，直接清零不打印
//                                         lidar->total_distance_sum = 0;
//                                         lidar->total_distance_count = 0;
//                                     }

                                    
//                                     uint16_t raw_angle = (lidar->packet_buffer[2] << 8) | angle_low_byte;
//                                     uint16_t raw_dist  = (lidar->packet_buffer[4] << 8) | lidar->packet_buffer[3];
//                                     uint16_t angle_data_x64 = (raw_angle >> 1);
//                                     uint16_t dist_data_x4 = raw_dist;
                                    
//                                     bool angle_ok = false;
//                                     if (lidar->filter_wrap_around) {
//                                         angle_ok = (angle_data_x64 >= lidar->filter_start_angle_x64) ||
//                                                    (angle_data_x64 <= lidar->filter_end_angle_x64);
//                                     } else {
//                                         angle_ok = (angle_data_x64 >= lidar->filter_start_angle_x64) &&
//                                                    (angle_data_x64 <= lidar->filter_end_angle_x64);
//                                     }

                                    
//                                     if (angle_ok) {

//                                         uint16_t distance_to_send_x4 = 0;

                                    
//                                         bool dist_ok = (dist_data_x4 >= lidar->filter_min_dist_x4) &&
//                                                        (dist_data_x4 <= lidar->filter_max_dist_x4);

//                                         if (dist_ok) {
                                    
//                                             lidar->total_distance_sum += dist_data_x4;
//                                             lidar->total_distance_count++;
//                                             distance_to_send_x4 = dist_data_x4;
//                                         } else {
                                    
                                    
//                                             distance_to_send_x4 = lidar->last_avg_distance_x4;
//                                         }

                                    
                                    
//                                         // uint8_t tx_buf[6];
//                                         // tx_buf[0] = 0xAA; // Start
//                                         // tx_buf[1] = (angle_data_x64 & 0xFF);
//                                         // tx_buf[2] = (angle_data_x64 >> 8) & 0xFF;
//                                         // tx_buf[3] = (distance_to_send_x4 & 0xFF);
//                                         // tx_buf[4] = (distance_to_send_x4 >> 8) & 0xFF;
//                                         // tx_buf[5] = 0xBB; // Stop
//                                         // HAL_UART_Transmit(lidar->pc_uart, tx_buf, 6, 5);
//                                     }
//                                 }
//                             }
//                             break;
//         } // switch
//     } // while
// }
#include <stdio.h> // 确保包含 stdio.h

void RPLIDAR_Process(RPLIDAR_Handle_t* lidar)
{
    // 1. 计算 DMA 写指针位置 (使用取余防止越界)
    uint32_t dma_write_index = (LIDAR_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(lidar->lidar_dma)) % LIDAR_DMA_BUFFER_SIZE;

    // 2. 安全计数器：防止一次处理太多数据卡死主循环
    // 即使波特率很高，也建议限制单次处理量，把 CPU 时间分给电机控制
    int process_safety_count = 0; 
    const int MAX_PROCESS_PER_LOOP = 100; 

    while ((lidar->dma_read_index != dma_write_index) && (process_safety_count < MAX_PROCESS_PER_LOOP))
    {
        process_safety_count++; 

        // 从环形缓冲区取出一个字节
        uint8_t current_byte = lidar->dma_buffer[lidar->dma_read_index];
        lidar->dma_read_index = (lidar->dma_read_index + 1) % LIDAR_DMA_BUFFER_SIZE;

        // 状态机异常保护
        if (!lidar->is_active && lidar->state != LIDAR_STATE_IDLE) {
             lidar->state = LIDAR_STATE_IDLE;
        }
        if(lidar->state == LIDAR_STATE_IDLE && !lidar->is_active){
            continue;
        }

        // 核心状态机
        switch (lidar->state)
        {
            case LIDAR_STATE_IDLE:
                break;

            case WAITING_FOR_DESCRIPTOR_A5:
                if (current_byte == 0xA5) {
                    lidar->state = WAITING_FOR_DESCRIPTOR_5A;
                }
                break;

            case WAITING_FOR_DESCRIPTOR_5A:
                if (current_byte == 0x5A) {
                    lidar->packet_index = 0;
                    lidar->state = RECEIVING_DESCRIPTOR;
                } else if (current_byte != 0xA5) {
                    lidar->state = WAITING_FOR_DESCRIPTOR_A5; 
                }
                break;

            case RECEIVING_DESCRIPTOR:
                lidar->packet_buffer[lidar->packet_index++] = current_byte;
                if (lidar->packet_index >= 5) {
                    // 检查 SCAN 命令的描述符: 05 00 00 40 81
                    if (memcmp(lidar->packet_buffer, EXPECTED_SCAN_DESCRIPTOR, 5) == 0) {
                        lidar->packet_index = 0;
                        lidar->state = RECEIVING_SCAN_PACKET;
                    } else {
                        lidar->state = WAITING_FOR_DESCRIPTOR_A5; 
                    }
                }
                break;

            case RECEIVING_SCAN_PACKET:
                lidar->packet_buffer[lidar->packet_index++] = current_byte;

                // 攒够 5 个字节（一个完整的测距点包）
                if (lidar->packet_index >= 5) {
                    lidar->packet_index = 0; 

                    // 解析协议位
                    uint8_t sync_quality     = lidar->packet_buffer[0];
                    uint8_t angle_low_byte   = lidar->packet_buffer[1];
                    uint8_t sync_bit         = (sync_quality & 0x01);
                    uint8_t inverse_sync     = (sync_quality & 0x02) >> 1;
                    uint8_t check_bit        = (angle_low_byte & 0x01);

                    // 校验检查
                    if ((sync_bit != inverse_sync) && (check_bit == 1)) {
                        
                        //如果是新的一圈，重置计数
                        if (sync_bit == 1) {
                            lidar->total_distance_count = 0;
                        }

                        // --- 核心解析 ---
                        uint16_t raw_angle = (lidar->packet_buffer[2] << 8) | angle_low_byte;
                        uint16_t raw_dist  = (lidar->packet_buffer[4] << 8) | lidar->packet_buffer[3];
                        
                        uint16_t angle_data_x64 = (raw_angle >> 1); // 去掉校验位
                        uint16_t dist_data_x4 = raw_dist;

                        // 1. 简单过滤：只要距离不为0就认为是有效点
                        if (dist_data_x4 > 0) {
                            lidar->total_distance_count++;

                            // 换算物理单位
                            float real_angle = angle_data_x64 / 64.0f; // 度
                            float real_dist = dist_data_x4 / 4.0f;     // 毫米

                            // ==============================================
                            // 🟢 修复方案：强制转为 int 打印
                            // ==============================================
                            
                            // 把角度转为整数（如果想保留小数，可以乘100再转int）
                            int angle_int = (int)real_angle; 
                            // 距离本身就是毫米，转int足够了
                            int dist_int = (int)real_dist;

                            char msg[64];
                            
                            // 使用 %d 而不是 %f，这样 100% 能打印出来！
                            // 格式：A:角度 D:距离
                            int len = sprintf(msg, "A:%d D:%d\r\n", angle_int, dist_int);
                            
                            // 发送数据
                            HAL_UART_Transmit(lidar->pc_uart, (uint8_t*)msg, len, 2);
                        }
                    }
                }
                break;
        } // switch
    } // while
}
void RPLIDAR_StartScan(RPLIDAR_Handle_t* lidar)// 开始扫描
{
    if (!lidar || !lidar->lidar_uart) return;
    lidar->is_active = true;
    lidar->packet_counter = 0;
    lidar->packet_index = 0;
    lidar->state = WAITING_FOR_DESCRIPTOR_A5;

    
    lidar->total_distance_sum = 0;
    lidar->total_distance_count = 0;
    lidar->new_revolution = false;
    lidar->last_avg_distance_x4 = 0;
    lidar->last_point_count = 0;

    HAL_UART_Transmit(lidar->lidar_uart, (uint8_t*)CMD_START_SCAN, sizeof(CMD_START_SCAN), 100);
    //printf("StartScan komutu gonderildi.\r\n");
}

void RPLIDAR_StopScan(RPLIDAR_Handle_t* lidar)// 停止扫描
{
    if (!lidar || !lidar->lidar_uart) return;
    lidar->is_active = false;
    lidar->state = LIDAR_STATE_IDLE; 
    HAL_UART_Transmit(lidar->lidar_uart, (uint8_t*)CMD_STOP_SCAN, sizeof(CMD_STOP_SCAN), 100);
}

void RPLIDAR_SetAngleFilter(RPLIDAR_Handle_t* lidar,
                            uint16_t start_angle_x64,
                            uint16_t end_angle_x64,
                            bool wrap_around)// 设置角度过滤器参数
{
    if (!lidar) return;
    lidar->filter_start_angle_x64 = start_angle_x64;
    lidar->filter_end_angle_x64 = end_angle_x64;
    lidar->filter_wrap_around = wrap_around;
}

void RPLIDAR_SetDistanceFilter(RPLIDAR_Handle_t* lidar,
                               uint16_t min_dist_x4,
                               uint16_t max_dist_x4)// 设置距离过滤器参数
{
    if (!lidar) return;
    lidar->filter_min_dist_x4 = min_dist_x4;
    lidar->filter_max_dist_x4 = max_dist_x4;
}

void RPLIDAR_RxCallback(RPLIDAR_Handle_t* lidar, UART_HandleTypeDef *huart)// PC UART 接收回调
{
    
    if (lidar && huart == lidar->pc_uart)
    {
        if (lidar->pc_rx_byte == 's') {
            RPLIDAR_StartScan(lidar);
    
             const char msg_start[] = "LIDAR Baslatildi.\r\n";
             HAL_UART_Transmit(lidar->pc_uart, (uint8_t*)msg_start, sizeof(msg_start)-1, 10);
        } else if (lidar->pc_rx_byte == 'x') {
            RPLIDAR_StopScan(lidar);
    
             const char msg_stop[] = "LIDAR Durduruldu.\r\n";
             HAL_UART_Transmit(lidar->pc_uart, (uint8_t*)msg_stop, sizeof(msg_stop)-1, 10);
        }

    
        HAL_UART_Receive_IT(lidar->pc_uart, &lidar->pc_rx_byte, 1);
    }
    
    
}
