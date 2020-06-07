/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_def.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_iwdg.h"
#include "xd_dig_led.h"
#include "xd_tm1650.h"
#include "xd_key.h"
#include "xd_relay.h"
#include "xd_infrared.h"
#include "xd_x9c103.h"
#include "xd_sensor.h"
#include "multi_button.h"
#include <stdio.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;

osThreadId_t defaultTaskHandle;
osThreadId_t myTask_Key_ScanHandle;
osThreadId_t myTask03Handle;
osTimerId_t myTimer01Handle;
osTimerId_t myTimer02Handle;
/* USER CODE BEGIN PV */
userOpAttr_t g_config = {
	.o3_set_minute = 0,
	.user_timer_max = 120,
	.tm1650_status = 0,
	.x9c103_gear = 0,
	.power_key_status = 0,
};

uint32_t push_key_value = 0;
uint32_t release_key_value = 0;
uint8_t ir_key = 0;
uint8_t ir_add = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM3_Init(void);
static void MX_IWDG_Init(void);
static void MX_ADC1_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void Callback01(void *argument);
void Callback02(void *argument);

static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void Device_Init(void)
{
	//memset(&g_config, 0, sizeof(g_config));


	/* 检查是否为独立看门狗复位 */
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
	{
		/* 独立看门狗复位 */
		/*  亮红灯 */
		LED_G_YELLOW_Off();
		LED_H_YELLOW_On();

		LED_I_RED_Off();
		LED_J_RED_On();	
		/* 清除标志 */
		__HAL_RCC_CLEAR_RESET_FLAGS();
		/*如果一直不喂狗，会一直复位，加上前面的延时，会看到红灯闪烁
		在1s 时间内喂狗的话，则会持续亮绿灯*/
	}
	else
	{
		/*不是独立看门狗复位(可能为上电复位或者手动按键复位之类的) */
		/* 亮蓝灯 */
		LED_A_BLUE_On();
	}        

	
	g_config.user_timer_max = 120;
	X9C103_Value_Clear();



	IR_InfraInit();  //启动TIM4
	I2C_Initializes();
    	XD_Key_init();

  	
	//HAL_TIM_Base_Start_IT(&htim1); //使用定时器的时候调用这个函数启动
	//HAL_TIM_Base_Stop_IT(&htim1);  //停止定时器的时候调用这个函数关闭
	HAL_TIM_Base_Start_IT(&htim3); //使用定时器的时候调用这个函数启动

	//HAL_Delay(200);   //这里加个延时，否则上电不显示

	//关显示
	TM1650_Close_Display();	


	//档位灯全灭
	X9C103_Value_Clear();
	X9C103_Gear_Display(0);	

	//风扇转
	DEVICE_STOP;
	g_config.current_cmd = STOP;
	g_config.fan_run_status = STOP;
	g_config.o3_run_status = STOP;

	
	g_config.fan_stop_sec = 30;
	g_config.manaul_status = MANAUL_FAN_STOP_30S;
	g_config.o3_stop_sec = 30;

	g_config.debug_mode = DISABLE;

	
  	osTimerStart(myTimer02Handle, 5);
  	__HAL_IWDG_START(&hiwdg);

}

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
  MX_TIM5_Init();
  MX_TIM3_Init();
  MX_IWDG_Init();
  MX_ADC1_Init();
  MX_UART4_Init();
  MX_UART5_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  	Device_Init();
	//printf("hello world\n");
	//printf("hello world\n");
	//UART_Open(UART, 115200);
	//TM1650_Open_Display();
  /* USER CODE END 2 */

  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of myTimer01 */
  const osTimerAttr_t myTimer01_attributes = {
    .name = "myTimer01"
  };
  myTimer01Handle = osTimerNew(Callback01, osTimerOnce, NULL, &myTimer01_attributes);

  /* definition and creation of myTimer02 */
  const osTimerAttr_t myTimer02_attributes = {
    .name = "myTimer02"
  };
  myTimer02Handle = osTimerNew(Callback02, osTimerPeriodic, NULL, &myTimer02_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
    	Device_Init();

  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 128
  };
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* definition and creation of myTask_Key_Scan */
  const osThreadAttr_t myTask_Key_Scan_attributes = {
    .name = "myTask_Key_Scan",
    .priority = (osPriority_t) osPriorityLow,
    .stack_size = 128
  };
  myTask_Key_ScanHandle = osThreadNew(StartTask02, NULL, &myTask_Key_Scan_attributes);

  /* definition and creation of myTask03 */
  const osThreadAttr_t myTask03_attributes = {
    .name = "myTask03",
    .priority = (osPriority_t) osPriorityLow,
    .stack_size = 128
  };
  myTask03Handle = osThreadNew(StartTask03, NULL, &myTask03_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  
  while (1)
  {
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL14;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM5_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 56000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 112-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xffff;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 4;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_8 
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15 
                          |GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC2 PC6 PC7 PC8 
                           PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 PA5 PA8 
                           PA9 PA10 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_8 
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB15 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */

  /* Infinite loop */
	for(;;)
	{
		HAL_IWDG_Refresh(&hiwdg);

		/////
		switch (g_config.manaul_status) {
		case MANAUL_FAN_STOP_30S:
		
			//默认该状态
			if (g_config.current_cmd == RUN){
				g_config.manaul_status = MANAUL_FAN_RUN;
			}

			break;
		case MANAUL_FAN_RUN:
			//如果风扇停止则运行起来
			if (g_config.fan_run_status != RUN){
				RELAY_FAN_RUN();
				g_config.fan_run_sec = 1;  //
			}

			//运行检测
			if ( g_config.fan_run_sec <60){
				g_config.fan_run_sec++;
			}
				
			//30 s
			if (g_config.fan_run_sec >= 30){
				g_config.manaul_status = MANAUL_FAN_RUN_30S;
			}
			
			break;
		case MANAUL_FAN_RUN_30S:
			//启动臭氧
			if (g_config.current_cmd == RUN && g_config.o3_set_minute > 0){
				g_config.manaul_status = MANAUL_O3_RUN;
			}

			//停止臭氧
			if (g_config.current_cmd == STOP){
				g_config.manaul_status = MANAUL_O3_STOP;
			}

			break;

		case MANAUL_O3_RUN:
			//启动臭氧
			if (g_config.o3_run_status != RUN){
				RELAY_O3_RUN();
				g_config.o3_run_minute = 0;
				g_config.o3_run_sec = 0;
			}

			//运行检测
			if ( g_config.o3_run_sec <60){
				g_config.o3_run_sec++;
			}

			//30 s,  switch 
			if (g_config.fan_run_sec >= 10){
				g_config.manaul_status = MANAUL_O3_RUN_30S;
			}
			
			break;
		case MANAUL_O3_RUN_30S:

			//停止臭氧
			if (g_config.current_cmd == STOP){
				g_config.manaul_status = MANAUL_O3_STOP;
			}
		
			break;
		case MANAUL_O3_STOP:
			//如果风扇停止则运行起来
			if (g_config.o3_run_status != STOP){
				RELAY_O3_STOP();
				g_config.o3_stop_sec = 1;
			}

			//30 s
			if ( g_config.o3_stop_sec <60){
				g_config.o3_stop_sec++;
			}

			//30s arrive,  switch 
			if (g_config.o3_stop_sec >= 10){
				g_config.manaul_status = MANAUL_O3_STOP_30S;
			}
			

			break;
		case MANAUL_O3_STOP_30S:


			//停止风扇
			if (g_config.current_cmd == STOP){
				g_config.manaul_status = MANAUL_FAN_STOP;
			}
		
		case MANAUL_FAN_STOP:
			// 这个是个保护状态
			if (g_config.fan_run_status != STOP){
				RELAY_FAN_STOP();
				g_config.fan_stop_sec = 0;
				g_config.fan_stop_minute = 0;
			}
			
			//30 s
			if ( g_config.fan_stop_sec <60){
				g_config.fan_stop_sec++;
			}

		
			//30s arrive,  switch 
			if (g_config.fan_stop_sec >= 10){
				g_config.manaul_status = MANAUL_FAN_STOP_30S;
			}
			
			break;
			
		default:
			break;
		}
		
		osDelay(800);

	}	
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask_Key_Scan thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
	int i = 0;
  
  /* Infinite loop */
	for(;;)
	{
		g_config.cur_adc_value = ADC_ReadData();
		g_config.cur_sw_value = (((float)g_config.cur_adc_value) / 4096) * 3.3;


		// 采样到数组
		for (i = SAMPLE_NUM -1; i > 0; i--){
			g_config.a_cur_value[i] = g_config.a_cur_value[i - 1]; 
		}
		g_config.a_cur_value[0] = g_config.cur_sw_value; 


		//电流保护
		if(g_config.a_cur_value[0] < 0.32 && g_config.a_cur_value[1] < 0.32 && g_config.a_cur_value[2] < 0.32 
			&& g_config.a_cur_value[3] < 0.32 && g_config.a_cur_value[4] < 0.32 && g_config.fan_run_status == RUN){
			RELAY_O3_STOP();  //停机保护
			DEVICE_STOP;  //停机
		}
		

		//连续运行保护
		if(g_config.o3_run_minute > 120){
			DEVICE_STOP;  //停机
		}
		

		

		//风扇运行秒数
		if (g_config.dev_run_mode == MANAUL){
		
			if (g_config.o3_set_minute == 0){
				DEVICE_STOP;  //停机
			}
			else{
				DEVICE_RUN;  //开机
			}
		}
		else{

			//空气传感器
			g_config.kq_sensor_a = READ_KQ_SENSOR_A();
			g_config.kq_sensor_b = READ_KQ_SENSOR_B();


			if (g_config.kq_sensor_a  == 0 && g_config.kq_sensor_b == 0){
				g_config.kq_quality = 1;
			}
			else if (g_config.kq_sensor_a  == 0 && g_config.kq_sensor_b == 1){
				g_config.kq_quality = 2;
			}
			else if (g_config.kq_sensor_a  == 1 && g_config.kq_sensor_b == 0){
				g_config.kq_quality = 3;
			}
			else if (g_config.kq_sensor_a  == 1 && g_config.kq_sensor_b == 1){
				g_config.kq_quality = 4;
			}



			//如果风扇停止则运行起来
			if (g_config.kq_quality > 3 || g_config.fan_stop_minute > 20){
				//如果风扇停止则运行起来
				if (g_config.fan_stop_minute > 10){
					DEVICE_RUN;  //防止停机
				}
			}

			if (g_config.kq_quality < 3 || g_config.o3_run_minute > 30){
				//如果风扇停止则运行起来
				if (g_config.o3_run_minute > 3){
					DEVICE_STOP;  //防止停机
				}
			}

			
		}



		osDelay(2000);
	}
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */

	/* Infinite loop */
	for(;;)
	{

		//调试模式显示电流
		if(g_config.debug_mode == ENABLE){
		
			///tmep code
			TM1650_First_Display_Num(0xd);
			TM1650_Display_Float_2_Num(g_config.cur_sw_value);
			osDelay(1000);
			continue;
		}


		if(g_config.dev_run_mode == MANAUL){
		
			if (g_config.tm1650_status == DISABLE){
				TM1650_Close_Display();	
			}
			else{
				TM1650_Open_Display();	
				TM1650_Display_Num(g_config.o3_set_minute);
			}
		}
		else{

			///tmep code
			TM1650_First_Display_Num(0xa);
			TM1650_Second_Display_Num(0);
			TM1650_Thirth_Display_Num(g_config.kq_quality);
			
		}

		osDelay(1000);

	}
  /* USER CODE END StartTask03 */
}

/* Callback01 function */
void Callback01(void *argument)
{
  /* USER CODE BEGIN Callback01 */
  	//启动延时30秒

  /* USER CODE END Callback01 */
}

/* Callback02 function */
void Callback02(void *argument)
{
  /* USER CODE BEGIN Callback02 */
	button_ticks();
  /* USER CODE END Callback02 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  //static uint8_t  flag = 0;
	//定时器频率为 f = 72M / Prescaler / Period = 72000 000 / 72 /1000 = 1000Hz；


	
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	else if(htim->Instance == TIM5)
	{
		IR_InfraPeriodHandle(htim);
	}
	else if(htim->Instance == TIM3)
	{
		//1秒定时器
		if (g_config.power_key_status == DISABLE){
			LED_J_Flash();
		}


		//手动情况下倒计时
		g_config.sec_count++;
		if (g_config.sec_count > 59){
			g_config.sec_count = 0;
			if (g_config.o3_set_minute > 0 && g_config.dev_run_mode == MANAUL){
				g_config.o3_set_minute--;
			}

			//o3 运行分钟
			if (g_config.o3_run_minute < 600){
				g_config.o3_run_minute++;
			}

			//o3 运行分钟
			if (g_config.fan_stop_minute < 600){
				g_config.fan_stop_minute++;
			}
		}
	}
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
