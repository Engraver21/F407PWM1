#include "rplidar_c1.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h> 
#include "math.h"
#include <sys/_intsup.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal_uart.h"
#include "usart.h" 
#include "data_pro.h"
extern volatile LidarData_t lidar_data[LIDAR_DATA_SIZE];

uint16_t point_index = 0;// 用于存储点的索引

static const uint8_t CMD_START_SCAN[] = {0xA5, 0x20};
static const uint8_t CMD_STOP_SCAN[]  = {0xA5, 0x25};
static const uint8_t EXPECTED_SCAN_DESCRIPTOR[] = {0x05, 0x00, 0x00, 0x40, 0x81};
// 1. 定义指令 (注意这里不需要 char，用 uint8_t 更规范)
static const uint8_t cmd_health[] = {0xA5, 0x52};
uint8_t health_res[10] = {0}; // 准备一个临时缓冲区
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

	// lidar->total_distance_sum = 0;
	// lidar->total_distance_count = 0;
	// lidar->new_revolution = false;
	// lidar->last_avg_distance_x4 = 0;
	// lidar->last_point_count = 0;

    RPLIDAR_SetAngleFilter(lidar, 150*64, (185 * 64), false); // 0-360 derece
    RPLIDAR_SetDistanceFilter(lidar, (400 * 4), (650 * 4)); // 50mm - 3000mm
    RPLIDAR_SetQualityFilter( lidar, 1);//设置质量过滤器
    
    if (HAL_UART_Receive_IT(lidar->pc_uart, &lidar->pc_rx_byte, 1) != HAL_OK) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef ret = HAL_UART_Receive_DMA(lidar->lidar_uart, lidar->dma_buffer, LIDAR_DMA_BUFFER_SIZE) ;
    if (ret != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_OK;
}
void RPLIDAR_Process(RPLIDAR_Handle_t* lidar)
{
    // 1. 获取 DMA 写入位置 (Head)
    uint32_t dma_write_index = LIDAR_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(lidar->lidar_dma);

    while (lidar->dma_read_index != dma_write_index)// 当读索引不等于写索引时，处理数据
    {
        uint8_t current_byte = lidar->dma_buffer[lidar->dma_read_index];// 获取当前字节
        lidar->dma_read_index = (lidar->dma_read_index + 1) % LIDAR_DMA_BUFFER_SIZE;// 更新读索引，环绕缓冲区
    
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

                        lidar->packet_index = 0;
                        lidar->state = RECEIVING_SCAN_PACKET;
                    } else {
                        lidar->state = WAITING_FOR_DESCRIPTOR_A5; 
                    }
                }
                break;

            case RECEIVING_SCAN_PACKET:
                lidar->packet_buffer[lidar->packet_index++] = current_byte;
                if (lidar->packet_index >= 5) {
                    lidar->packet_index = 0;

                    uint8_t sync_quality = lidar->packet_buffer[0];
                    uint8_t angle_low    = lidar->packet_buffer[1];
                    uint8_t sync_bit     = (sync_quality & 0x01);       // S 位：1表示新一圈开始
                    uint8_t inv_sync     = (sync_quality & 0x02) >> 1;  // !S 位
                    uint8_t check_bit    = (angle_low & 0x01);          // C 位：恒为1

                    // 1. 基础校验
                    if ((sync_bit != inv_sync) && (check_bit == 1)) {
                        
                        // 【核心逻辑】检测到新的一圈开始
                        if (sync_bit == 1 && point_index > 0) {
                            ready_point_count = point_index; // 记录这一圈有多少点
                            
                            // 交换 Ping-Pong 缓冲区指针
                            LidarData_t* temp = writing_ptr;
                            writing_ptr = reading_ptr;
                            reading_ptr = temp;

                            beg_co_sig = true; // 通知主循环可以打印 reading_ptr 了
                            point_index = 0;   // 重置索引，写新的缓冲区
                        }

                        // 2. 解析数据
                        uint16_t raw_quality = sync_quality >> 2; // 质量：高6位
                        uint16_t raw_angle = (lidar->packet_buffer[2] << 8) | angle_low>>1;
                        raw_angle = raw_angle>>1; // 右移1位，去掉C位
                        uint16_t raw_dist  = (lidar->packet_buffer[4] << 8) | lidar->packet_buffer[3];
                        bool ang_ok = false;
                        bool dist_ok = false;
                        bool quality_ok = false;
                        if (raw_angle>=lidar->filter_start_angle_x64 && raw_angle<=lidar->filter_end_angle_x64)
                        {
                            ang_ok = true;
                        }
                        if (raw_dist>=lidar->filter_min_dist_x4 && raw_dist<=lidar->filter_max_dist_x4)
                        {
                            dist_ok = true;
                        }
                        if (raw_quality>=lidar->lid_min_quality)
                        {
                            quality_ok = true;
                        }

                        // 3. 存入当前正在写入的缓冲区（writing_ptr）
                        if (point_index < LIDAR_MAX_POINTS && ang_ok== true && dist_ok== true && quality_ok == true) {
                            // 确保 writing_ptr 已经初始化，否则这里会 HardFault
                            if (writing_ptr != NULL) { 
                                writing_ptr[point_index].angle = (raw_angle );
                                writing_ptr[point_index].distance = raw_dist;
                                writing_ptr[point_index].quality = (sync_quality >> 2);
                                point_index++;
                            }
                        }
                    }
                }
                break;
            default:
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

    // lidar->total_distance_sum = 0;
    // lidar->total_distance_count = 0;
    // lidar->new_revolution = false;
    // lidar->last_avg_distance_x4 = 0;
    // lidar->last_point_count = 0;

    HAL_UART_Transmit(lidar->lidar_uart, (uint8_t*)CMD_START_SCAN, sizeof(CMD_START_SCAN), 100);
    //printf("StartScan komutu gonderildi.\r\n");
    printf("start scan\r\n");
}

void RPLIDAR_StopScan(RPLIDAR_Handle_t* lidar)// 停止扫描
{
    if (!lidar || !lidar->lidar_uart) return;
    lidar->is_active = false;
    lidar->state = LIDAR_STATE_IDLE; 
    HAL_UART_Transmit(lidar->lidar_uart, (uint8_t*)CMD_STOP_SCAN, sizeof(CMD_STOP_SCAN), 100);
    printf("stop scan\r\n");
    
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

void RPLIDAR_SetQualityFilter(RPLIDAR_Handle_t* lidar,
                               uint16_t min_quality)// 设置距离过滤器参数
{
    if (!lidar) return;
    // lidar->lid_min_quality = min_quality;
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
  void RPLIDAR_viewStatus(RPLIDAR_Handle_t* lidar)// 停止扫描
{
    if (!lidar || !lidar->lidar_uart) return;
    HAL_UART_Transmit(lidar->lidar_uart, (uint8_t*)cmd_health, sizeof(cmd_health), 100);

    // 2. 给雷达一点准备时间
    HAL_Delay(500);

    HAL_StatusTypeDef res = HAL_UART_Receive(&huart6, health_res, 10, 500);
    
    if(res == HAL_OK) {
    printf("Health Response: ");
    for(int i = 0; i < 10; i++) {
        printf("%02X ", health_res[i]); // 以十六进制格式打印每一个字节
    }
    printf("\r\n");
    
    // 解析健康状态：第 7 个字节是状态位 (Index从0开始的话是第7位)
    // 应答格式：A5 5A 03 00 00 00 06 [Status] [Error_Low] [Error_High]
    uint8_t status = health_res[7]; 
    if(status == 0) printf("Lidar Health: OK\r\n");
    else if(status == 1) printf("Lidar Health: Warning\r\n");
    else if(status == 2) printf("Lidar Health: ERROR (Protection Mode!)\r\n");
    } else {
        printf("Lidar No Response!\r\n");
    }
    printf("stop scan\r\n");
    
}