/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "stdio.h"
#include "stdlib.h"
#include "my_main.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
char message_0[20];
char message_1[20];
char message_1x[20];
char message_2[20];
char message_4[20];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int key=0;
int key_real=0;
int t=3;
int t_new=1;
int angle;
extern int flag_2;
char message[20]="0";
char en_speed[20]="";
char en_place[20]="";
char adc_xx[20];
char key_x[20];
char angle_n[20];
int angle1;
int angle2;
extern uint16_t adc_value;
extern int flag_y;
extern int Encoder_Cnt;
extern float Encoder_Integral;
extern PID_InitTypeDef Turn_PID_1;
extern PID_InitTypeDef Turn_PID_2;
int y_screen;
uint8_t order[10];
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/**************************************************************************
函数功能：单位时间读取ADC
入口参数：TIM
返回  值：ADC
**************************************************************************/

/**************************************************************************
函数功能：卡尔曼滤波
入口参数：inData
返回  值：inData
**************************************************************************/
//卡尔曼滤波

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
    if(huart==&huart2)
    {
        key_real=order[0];
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2,order,sizeof(order));
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);
    }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
     if(GPIO_Pin==KEY1_Pin)
     {
         if(key_real==1)
         {
             Load(-2000);
         }
         else
         {
             key+=1;
         }
     }
     else if(GPIO_Pin==KEY2_Pin)
     {
         if (key_real == 1)
         {
             Load(2000);
         } else
         {
             key += 2;
         }
     }
     else if(GPIO_Pin==KEY3_Pin)
     {
         if (key_real == 1)
         {
             Load(0);
         } else
         {
             key += 3;
         }
     }
     else if(GPIO_Pin==KEY4_Pin)
     {
         key_real = key;
         key = 0;
     }
     else if(GPIO_Pin==KEY5_Pin)
     {
            key=0;
            key_real=0;
            Load(0);
     }
}
float encoder_to_angle(int32_t encoder_count, int32_t cpr) {
    // 1. 计算单圈位置（处理负数）
    int32_t single_turns = (encoder_count % cpr + cpr) % cpr;

    // 2. 转换为0~360°
    float angle_raw = (single_turns * 360.0f) / cpr;

    // 3. 调整到-180°~+180°
    return (angle_raw >= 180) ? (angle_raw - 360) : angle_raw;
}
uint8_t angle_to_screen(int8_t angle) {
    // 将-90~90映射到63~0
    return (uint8_t)(64 - 1 - ((angle - (-90)) * 64) / (90 - (-90)));
}
// 调用示例
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2,order,sizeof(order));
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY);
    PID_Init(&Turn_PID_1,
             -0.0756,//78,    // Kpa（比例项主导） 2000->15; 15/2000
             0,    // Kpb（误差>200时增强响应）
             -0.18,    // Kd（抑制震荡）
             0,    // gkd（未使用）
             1000
    );

// 外环（缓慢平衡编码器积分）
    PID_Init(&Turn_PID_2,
             -432,   // Kpa（小增益防超调）15->7200   7200/15
             0,   // Kpb（抑制积分饱和）
             -60,    // Kd（快速阻尼）
             0,
             7200
    );
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      angle1 = (Encoder_Integral * 360.0 / 1074.0);  // 根据编码器位置和每圈脉冲数计算角度
      if(adc_value<130&&adc_value>0)
      {
          angle2=-(adc_value-150)*360/4096;
      }
      else if(adc_value>2260&&adc_value<4096)
      {
          angle2=17-(adc_value-4096)*360/4096;
      }
      else
      {
          angle2=-(adc_value-130)*360/4096+4;
      }
      angle = encoder_to_angle(Encoder_Integral, 1074);
      sprintf(key_x,"按键值累加%d",key);
      sprintf(angle_n,"当前功能%d",key_real);
      sprintf(en_place,"en_place %d",angle);
      sprintf(adc_xx,"adc %d",adc_value);
      if(key_real!=8)
      {
          OLED_NewFrame();
          OLED_PrintString(0, 0, key_x, &font15x15, OLED_COLOR_NORMAL);
          OLED_PrintString(0, 15, angle_n, &font15x15, OLED_COLOR_NORMAL);
      }
      if(key_real==0)
      {
          sprintf(message_0,"已暂停");
          OLED_PrintString(0, 30, message_0, &font15x15, OLED_COLOR_NORMAL);
      }
      else if(key_real==1)
      {
          sprintf(message_1,"当前角度值%d",angle);
          sprintf(message_1x,"一正转二反转");
          OLED_PrintString(0, 30, message_1, &font15x15, OLED_COLOR_NORMAL);
          OLED_PrintString(0, 45, message_1x, &font15x15, OLED_COLOR_NORMAL);
      }
      else if(key_real==2||key_real==3)
      {
          sprintf(message_2,"当前摆杆角度%d",angle2);
          OLED_PrintString(0, 30, message_2, &font15x15, OLED_COLOR_NORMAL);
      }
      else if(key_real==4)
      {
          sprintf(message_4,"倒立平衡状态");
          OLED_PrintString(0, 30, message_4, &font15x15, OLED_COLOR_NORMAL);
      }
      else if(key_real==8)
      {
          if(t_new)
          {
              t_new--;
              OLED_NewFrame();
          }
          OLED_DrawLine(0, 32, 125, 32, OLED_COLOR_NORMAL);
          // X轴箭头
          OLED_DrawLine(123, 30, 125, 32, OLED_COLOR_NORMAL);
          OLED_DrawLine(123, 34, 125, 32, OLED_COLOR_NORMAL);
          // Y轴（垂直线）
          OLED_DrawLine(3, 0, 3, 64, OLED_COLOR_NORMAL);
          // Y轴箭头
          OLED_DrawLine(3, 0, 0, 3, OLED_COLOR_NORMAL);
          OLED_DrawLine(3, 0, 6, 3, OLED_COLOR_NORMAL);

          // 获取当前和下一个点的Y坐标
          uint8_t y_current = angle_to_screen(angle);
          uint8_t y_next = angle_to_screen(angle);
          // 绘制线段
          OLED_SetPixel(t,y_current+25,OLED_COLOR_NORMAL);
          OLED_DrawLine(t,y_current+25,t+1,y_next+25,OLED_COLOR_NORMAL);
          t++;
          if(t>124)
          {
              t=3;
              OLED_NewFrame();
          }
      }
//      OLED_PrintString(0, 32, en_place, &font16x16, OLED_COLOR_NORMAL);
//      OLED_PrintString(0, 48, adc_xx, &font16x16, OLED_COLOR_NORMAL);
      OLED_ShowFrame();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
