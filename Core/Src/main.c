/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "IIC.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "stdio.h"
#include "sr04.h"
#include "motor.h"
#include "encoder.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ================= 蓝牙遥测（发到电脑上的 VOFA+） =================
   格式叫 FireWater：逗号分隔、换行结尾。VOFA+ 里把解析方式选成 "FireWater"。
   七个通道依次为：
     1 roll 倾角(度)   2 gyrox 角速度(原始值)   3 Vertical_out 直立环输出(PWM)
     4 Velocity_out 速度环输出(度)   5 左右轮脉冲之和   6 stop 摔倒标志
     7 Control_dt_ms 实测控制周期(ms)
   注意 9600 波特率下一帧约 24 字符要占用约 25 ms，发送频率上不去。
   想更快：给蓝牙模块发 AT+BAUD8 改成 115200（改完重新上电），
   再把 usart.c 里的波特率也改成 115200。 */
#define BT_DECIM    5      /* 每 N 次 Control() 发一帧 → 100/N Hz，5 即 20 Hz */
#define BT_OLED_MS  200    /* 屏幕刷新间隔(ms)。故意放慢，因为串口发送是阻塞的 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 这些变量定义在别的文件（主要是 pid.c），此处声明以便使用 */
extern float roll;
extern float Speed_Pos;
extern float Speed_Pos_Raw;
extern float Control_dt_ms;
extern int Encoder_Left,Encoder_Right;
extern short gyrox;
extern short gyroz;
extern int Vertical_out,Turn_out;
extern float Velocity_out;
extern float Med_Angle;
extern uint8_t stop;
extern volatile uint32_t Control_Tick;
uint8_t display_buf[20];            /* 屏幕显示缓冲 */
uint8_t tx_buf[64];                 /* 串口发送缓冲 */
uint32_t sys_tick;
uint32_t oled_tick;                 /* 上次刷新屏幕的时刻 */
uint32_t bt_tick;                   /* 上次发送遥测的时刻 */
uint32_t bt_ok;                     /* 累计发送成功帧数 */
extern float distance;
extern uint8_t distance_valid;      /* 0 = 上次测距无回波，数据已过期 */
extern uint8_t rx_buf[2];
extern uint8_t Bluetooth_data;      /* 手机 App 最近发来的字节 */
extern uint32_t bt_rx_count;        /* 接收到的字节总数 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Read(void);
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
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	OLED_Init();
	OLED_Clear();
	MPU_Init();                 /* 初始化 MPU6050 */
	mpu_dmp_init();             /* 加载 DMP 固件，由它内部解算姿态角 */
	OLED_ShowString(0,00,"Init Sucess",16);

	/* 打开 MPU6050 中断：DMP 每算好一组数据就拉低该引脚，
	   中断服务函数里执行一次 Control()，这就是控制节拍的来源。 */
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);        /* 启动两路电机 PWM */
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);  /* 启动左右轮编码器 */
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);

	/* 打开串口接收中断。手机发来的字节在 USART3_IRQHandler 里
	   被解析成方向指令（见 stm32f1xx_it.c）。 */
	HAL_UART_Receive_IT(&huart3,rx_buf,1);
	Load(0,0);                  /* 先停住电机，避免上电就窜出去 */
	OLED_Clear();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		int bt_len;

		/* 每 BT_DECIM 次控制周期发一帧遥测，保证采样间隔均匀 */
		if((uint32_t)(Control_Tick - bt_tick) >= BT_DECIM)
		{
			bt_tick = Control_Tick;
			bt_len = sprintf((char *)tx_buf,"%.2f,%hd,%d,%.2f,%d,%d,%.1f\n",
			                 roll,                        /* 1 倾角(度)      */
			                 gyrox,                       /* 2 角速度(原始值) */
			                 Vertical_out,                /* 3 直立环输出    */
			                 Velocity_out,                /* 4 速度环输出(度) */
			                 Encoder_Left+Encoder_Right,  /* 5 轮速          */
			                 (int)stop,                   /* 6 摔倒标志      */
			                 Control_dt_ms);              /* 7 控制周期(ms)  */
			if(HAL_UART_Transmit(&huart3,tx_buf,(uint16_t)bt_len,100)==HAL_OK)
				bt_ok++;                                 /* 只统计成功帧 */
		}

		/* 屏幕调试信息，刷新较慢，方便不上电脑也能看到状态 */
		if((uint32_t)(HAL_GetTick() - oled_tick) >= BT_OLED_MS)
		{
			oled_tick = HAL_GetTick();
			sprintf((char *)display_buf,"roll:%.1f ",roll);
			OLED_ShowString(0,0,display_buf,16);
			sprintf((char *)display_buf,"distance=%.2f",distance);
			OLED_ShowString(0,2,display_buf,16);
			sprintf((char *)display_buf,"");
			OLED_ShowString(0,4,display_buf,16);

		}
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
