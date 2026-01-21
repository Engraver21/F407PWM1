/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "delay.h"
#include "usart.h"   // 确保能找到 huart1
#include "rplidar_c1.h" // 包含 RPLIDAR 库头文件
#include "string.h"
#include "key.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "motor_ctrl.h"
#include "Ball_screw_contrl.h"
#include "data_pro.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint32_t key=0;
extern LidarData_t lidar_data[LIDAR_DATA_SIZE];
extern  uint16_t point_index ;// 用于存储点的索引
extern volatile bool beg_co_sig;

volatile bool is_one_scan_ready; 
uint16_t volatile total_points_captured;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
RPLIDAR_Handle_t hlidar; // 定义雷达控制句柄
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM14_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART6_UART_Init();

  /* USER CODE BEGIN 2 */
  // &hdma_usart6_rx: 串口2的DMA接收句柄 (去 usart.c 或 usart.h 确认名字，通常是这个)
  RPLIDAR_Init(&hlidar, &huart6, &huart1, &hdma_usart6_rx);
  delay_init(168);// 初始化延时函数，参数为系统时钟频率MHz
 // Ball_Screw_init(); // 初始化滚珠丝杠

  // 2. 启动 DMA 接收 (这一步非常重要，你的库底层应该封装了 HAL_UART_Receive_DMA)
  // 如果 RPLIDAR_Init 里没调用 HAL_UART_Receive_DMA，你需要在这里手动调用：
  // 注意：看你的库函数 RPLIDAR_StartScan 内部实现，通常它会发指令并启动接收。
  RPLIDAR_StartScan(&hlidar);//* 启动雷达扫描 */
  HAL_UART_Receive_DMA(&huart6, hlidar.dma_buffer, LIDAR_DMA_BUFFER_SIZE);
  
  // printf("System Initialized.\r\n");
  // uint8_t msg[] = "Hello, UART with DMA!";
  // UART_DMA_Transmit(msg, sizeof(msg)); // 使用 DMA 发送数据
  RPLIDAR_StartScan(&hlidar);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    RPLIDAR_Process(&hlidar); // 定期调用雷达处理函数
    if (beg_co_sig == true) {
      data_collect(lidar_data, point_index);
      point_index  =0;
      //printf("收到新的一圈\r\n");
      beg_co_sig = false;
    }
    // 2. [新增] 串口“起搏器”：检查串口是否因为错误而挂起了
    if (hlidar.lidar_uart->ErrorCode != HAL_UART_ERROR_NONE)
    {
        // 如果有错误，强制清除并重启
        HAL_UART_AbortReceive(hlidar.lidar_uart);
        HAL_UART_Receive_DMA(hlidar.lidar_uart, hlidar.dma_buffer, LIDAR_DMA_BUFFER_SIZE);
    }
    
    // 3. [新增] 如果 DMA 悄悄停了 (State 不是 BUSY_RX)，也重启它
    if (hlidar.lidar_uart->RxState == HAL_UART_STATE_READY)
    {
          HAL_UART_Receive_DMA(hlidar.lidar_uart, hlidar.dma_buffer, LIDAR_DMA_BUFFER_SIZE);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// 当串口接收中断发生时（收到一个字节），HAL库会自动调用这个函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // 将中断事件传递给雷达库处理
  // 假设你的雷达句柄叫 hlidar (在 main.c 顶部定义的那个)
  RPLIDAR_RxCallback(&hlidar, huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    // 判断是不是雷达的串口 (假设是 huart6)
    // 如果你的雷达是 huart2，请改为 huart2
    if (huart->Instance == USART6) 
    {
        // 1. 发生错误了（比如 Overrun），先停止之前的 DMA
        HAL_UART_DMAStop(huart);

        // 2. 重新开启 DMA 接收！(这步最关键，让它继续干活)
        // 注意：这里需要引用你的全局变量 hlidar
        extern RPLIDAR_Handle_t hlidar; 
        
        // 重新启动接收，覆盖缓冲区
        HAL_UART_Receive_DMA(huart, hlidar.dma_buffer, LIDAR_DMA_BUFFER_SIZE);
        
        // 可选：重置读指针，防止处理脏数据
        // hlidar.dma_read_index = 0; 
    }
}
/* USER CODE END 4 */
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
