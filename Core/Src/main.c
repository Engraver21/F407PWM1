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
#include "iwdg.h"
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
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 重写 _write 函数 (GCC/STM32CubeIDE 专用)
int _write(int file, char *ptr, int len)
{
    // 使用阻塞模式发送，确保数据发送完成
    // 注意：不要在这里使用 DMA，因为 printf 调用频率高时 DMA 会因为 BUSY 而丢包
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 1000);
    return len;
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
RPLIDAR_Handle_t hlidar; // 定义雷达控制句柄
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

extern int objects_fed_count;              // 告诉外部有多少个有效物体
extern bool data_is_ready;             // 可选：数据更新完成标志
extern LidarObject_t objects_fed[MAX_OBJECTS];// 用于存储识别到的物体信息
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
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  delay_init(168);// 初始化延时函数，参数为系统时钟频率MHz
  // &hdma_usart6_rx: 串口2的DMA接收句柄 (去 usart.c 或 usart.h 确认名字，通常是这个)
  RPLIDAR_Init(&hlidar, &huart6, &huart1, &hdma_usart6_rx);
 // Ball_Screw_init(); // 初始化滚珠丝杠
  RPLIDAR_StartScan(&hlidar);//* 启动雷达扫描 */
 // HAL_Delay(500); // 给它一点时间反应
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
  {
    // 这里的字符串就是你要的信息
    printf("\r\n===================================\r\n");
    printf("[警告] 系统刚刚因为看门狗超时(死机)而重启！\r\n");
    printf("===================================\r\n");

    delay_ms(1000);
    __HAL_RCC_CLEAR_RESET_FLAGS();
  }
  else
  {
    printf("\r\n[信息] 系统正常上电启动。\r\n");  
  }

  // 2. 启动 DMA 接收 (这一步非常重要，你的库底层应该封装了 HAL_UART_Receive_DMA)
  // 如果 RPLIDAR_Init 里没调用 HAL_UART_Receive_DMA，你需要在这里手动调用：
  // 注意：看你的库函数 RPLIDAR_StartScan 内部实现，通常它会发指令并启动接收。

  printf("Before UART transmission\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)"Test message.\r\n", 15, HAL_MAX_DELAY);
  printf("After UART transmission\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    RPLIDAR_Process(&hlidar); // 定期调用雷达处理函数
    //data_collect(writing_ptr, ready_point_count);
    Lidar_Analyze_Objects(reading_ptr, ready_point_count);
    other_prtocess(objects_fed_count,objects_fed,data_is_ready);
    key = key_scan(0);
    if(key==KEY0_PRES){
      RPLIDAR_StartScan(&hlidar);//启动雷达信息
      printf("Start Scan\r\n");
    }else if(key==KEY1_PRES){
      //  Ball_Screw_move_a_step(0); // 滚珠丝杠反转一步{
      RPLIDAR_StopScan(&hlidar);//停止雷达信息
      printf("Stop Scan\r\n");
    }
    HAL_IWDG_Refresh(&hiwdg); // 喂狗
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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

// void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
// {
//     // 判断是不是雷达的串口 (假设是 huart6)
//     // 如果你的雷达是 huart2，请改为 huart2
//     if (huart->Instance == USART6) 
//     {
//         // 1. 发生错误了（比如 Overrun），先停止之前的 DMA
//         HAL_UART_DMAStop(huart);

//         // 2. 重新开启 DMA 接收！(这步最关键，让它继续干活)
//         // 注意：这里需要引用你的全局变量 hlidar
//         extern RPLIDAR_Handle_t hlidar; 
        
//         // 重新启动接收，覆盖缓冲区
//         HAL_UART_Receive_DMA(huart, hlidar.dma_buffer, LIDAR_DMA_BUFFER_SIZE);
        
//         // 可选：重置读指针，防止处理脏数据
//         // hlidar.dma_read_index = 0; 
//     }
// }

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        uint32_t isrflags = huart->Instance->SR; // 读取状态寄存器
        
        // 检查各种错误标志 (ORE:溢出, NE:噪声, FE:帧错误, PE:校验错误)
        if ((isrflags & (UART_FLAG_ORE | UART_FLAG_NE | UART_FLAG_FE | UART_FLAG_PE)) != RESET)
        {
            // 读取 DR 寄存器通常可以清除这些错误标志 (STM32 F4/F1 特性)
            // 或者使用宏清除
            __HAL_UART_CLEAR_OREFLAG(huart);
            __HAL_UART_CLEAR_NEFLAG(huart);
            __HAL_UART_CLEAR_FEFLAG(huart);
            __HAL_UART_CLEAR_PEFLAG(huart);
            // 遇到严重错误，建议强行重启 DMA
            // 先停止，以防万一
            HAL_UART_DMAStop(huart); 
            // 重新开始接收
            HAL_UART_Receive_DMA(huart, hlidar.dma_buffer, LIDAR_DMA_BUFFER_SIZE);
        }
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
