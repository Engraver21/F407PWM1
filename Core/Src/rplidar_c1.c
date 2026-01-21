#include "rplidar_c1.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h> 
#include "usart.h" // 用于 UART 传输函数
#include "data_pro.h"
extern LidarData_t lidar_data[LIDAR_DATA_SIZE];

uint16_t point_index = 0;// 用于存储点的索引
volatile bool beg_co_sig = false;// 数据是否已满

static const uint8_t CMD_START_SCAN[] = {0xA5, 0x20};
static const uint8_t CMD_STOP_SCAN[]  = {0xA5, 0x25};
static const uint8_t EXPECTED_SCAN_DESCRIPTOR[] = {0x05, 0x00, 0x00, 0x40, 0x81};
extern HAL_StatusTypeDef status;


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

 
    RPLIDAR_SetAngleFilter(lidar, 50*64, (200 * 64), false); // 0-360 derece
    RPLIDAR_SetDistanceFilter(lidar, (10 * 4), (3000 * 4)); // 50mm - 3000mm

    
    if (HAL_UART_Receive_IT(lidar->pc_uart, &lidar->pc_rx_byte, 1) != HAL_OK) {
        return HAL_ERROR;
    }

    
    if (HAL_UART_Receive_DMA(lidar->lidar_uart, lidar->dma_buffer, LIDAR_DMA_BUFFER_SIZE) != HAL_OK) {
    
        return HAL_ERROR;
    }

    return HAL_OK;
}

void RPLIDAR_Process(RPLIDAR_Handle_t* lidar)// 主处理函数，需在主循环中定期调用
{
    uint32_t dma_write_index = LIDAR_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(lidar->lidar_dma);

    while (lidar->dma_read_index != dma_write_index)
    {
        uint8_t current_byte = lidar->dma_buffer[lidar->dma_read_index];
        lidar->dma_read_index = (lidar->dma_read_index + 1) % LIDAR_DMA_BUFFER_SIZE;

    
        if (!lidar->is_active && lidar->state != LIDAR_STATE_IDLE) {
             lidar->state = LIDAR_STATE_IDLE;
        }
    
        if(lidar->state == LIDAR_STATE_IDLE && !lidar->is_active){
            continue;
        }


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
                    if (memcmp(lidar->packet_buffer, EXPECTED_SCAN_DESCRIPTOR, 5) == 0) {
                        const char msg_ok[] = "SCAN Descriptor OK.\r\n";
                        HAL_UART_Transmit(lidar->pc_uart, (uint8_t*)msg_ok, sizeof(msg_ok) - 1, 20);
                        lidar->packet_index = 0;
                        lidar->state = RECEIVING_SCAN_PACKET;
                    } else {
                        const char msg_err[] = "HATA: Gecersiz Descriptor!\r\n";
                        HAL_UART_Transmit(lidar->pc_uart, (uint8_t*)msg_err, sizeof(msg_err) - 1, 20);
                        lidar->state = WAITING_FOR_DESCRIPTOR_A5; 
                    }
                }
                break;

            case RECEIVING_SCAN_PACKET:
                            
                            lidar->packet_buffer[lidar->packet_index++] = current_byte;
                            
                            if (lidar->packet_index >= 5) {
                                lidar->packet_index = 0; 

                                uint8_t sync_quality     = lidar->packet_buffer[0];//扫描起始位和质量
                                uint8_t quality          = sync_quality >> 2;       //取出质量
                                uint8_t angle_low_byte   = lidar->packet_buffer[1];//低六位角度
                                uint8_t sync_bit         = (sync_quality & 0x01);   //正起始位
                                uint8_t inverse_sync     = (sync_quality & 0x02) >> 1;//负起始位
                                uint8_t check_bit        = (angle_low_byte & 0x01);  //校验位
                                
                                if ((sync_bit != inverse_sync) && (check_bit == 1)) {//校验成功
                                    if (sync_bit == 1) {//打印新的一圈
                                    }

                                    uint16_t raw_angle = (lidar->packet_buffer[2] << 8) | angle_low_byte;//低七位高八位全部计算
                                    uint16_t raw_dist  = (lidar->packet_buffer[4] << 8) | lidar->packet_buffer[3];//计算距离
                                    uint16_t angle_data_x64 = (raw_angle >> 1);//计算正确的角度数据64倍，使用需要除以64
                                    uint16_t dist_data_x4 = raw_dist;//正确距离四倍，使用除以4

                                    
                                    bool angle_ok = false;
                                    if (lidar->filter_wrap_around) {//角度过滤器
                                        angle_ok = (angle_data_x64 >= lidar->filter_start_angle_x64) ||//测量的角度小于起始角度
                                                   (angle_data_x64 <= lidar->filter_end_angle_x64);//或者侧量的角度大于末尾角度
                                    } else {
                                        angle_ok = (angle_data_x64 >= lidar->filter_start_angle_x64) &&//测量的角度大于起始角度
                                                   (angle_data_x64 <= lidar->filter_end_angle_x64);//并且测量的角度小于末尾角度
                                    }

                                     if (angle_ok) {//角度在过滤的范围内
                                        {
                                            point_index++;
                                            lidar_data[point_index].angle = angle_data_x64;
                                            lidar_data[point_index].distance = dist_data_x4;
                                            lidar_data[point_index].quality = quality;

                                            if (point_index==255) {
                                                beg_co_sig = true;
                                                return;
                                            }
                                        }

                                    //     bool dist_ok = (dist_data_x4 >= lidar->filter_min_dist_x4) &&//距离过滤器
                                    //     (dist_data_x4 <= lidar->filter_max_dist_x4);
                                    //     if (dist_ok) {
                                    //     }
                                    }
                                }
                            }//if (packet_index >= 5)
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
