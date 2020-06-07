/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */



/** 
  * @brief  HAL DMA Callback ID structure definition
  */
typedef enum
{
	MANAUL_FAN_RUN          = 1,    /*!< Full transfer     */
	MANAUL_FAN_RUN_30S,    /*!< Full transfer     */
	MANAUL_FAN_STOP,    /*!< Full transfer     */
	MANAUL_FAN_STOP_30S,    /*!< Full transfer     */
	MANAUL_O3_RUN,    /*!< Full transfer     */
	MANAUL_O3_RUN_30S,    /*!< Full transfer     */
	MANAUL_O3_STOP,    /*!< Full transfer     */
	MANAUL_O3_STOP_30S,    /*!< Full transfer     */
}DEV_MANAUL_RUN_STATUS_TypeDef;

#define SAMPLE_NUM   8


/// Attributes structure for thread.
typedef struct {

	uint32_t                 manaul_status;   ///�ֶ�����״̬
	uint32_t                 auto_status;   ///�ֶ�����״̬
	uint32_t                 current_cmd;   ///��ǰ�豸ָ� run��stop	
	uint32_t                 press_key_value;   ///<��Դ��bits
	uint32_t                 debug_mode;   ///debug mode


	uint32_t                 power_key_status;   ///<��Դ��bits
	uint32_t                 dev_run_mode;   ///�豸��ģʽ
	uint32_t                 tm1650_status;   ///�Զ����ֶ��л�


	//uint32_t                 fan_run_cmd;   ///��������״̬
	uint32_t                 fan_run_status;   ///��������״̬
	uint32_t                 fan_run_sec;   ///��������30��
	uint32_t                 fan_stop_sec;   ///��������30��


	//uint32_t                 o3_run_cmd;   ///��������״̬
	uint32_t                 o3_run_status;   ///o3 ����״̬
	uint32_t                 o3_set_minute;   ///�û��յ����õĶ�ʱ
	uint32_t                 o3_run_sec;   ///o3 ����30��
	uint32_t                 o3_stop_sec;   ///��������30��

	//����ֹͣ���Ӽ�����
	uint32_t                 fan_stop_minute;   ///��������30��
	uint32_t                 o3_run_minute;   ///��������30��



	uint32_t                 x9c103_gear;   ///��λ ��0-9��


	uint32_t                 sec_count;   ///�û���ʱ������λ��
	uint32_t                 user_timer_max;   ///�û���ʱ������λ��


	uint8_t                 kq_sensor_a;   ///
	uint8_t                 kq_sensor_b;   ///
	uint8_t                 kq_quality;   ///
	uint8_t                 resv;   ///

	uint32_t               cur_adc_value;   ///  ����ֵ
	float                    cur_sw_value;   ///  ����ֵ
	float                    a_cur_value[SAMPLE_NUM];   ///  ����ֵ
	
	uint32_t                  reserved;   ///< reserved (must be 0)
} userOpAttr_t;


/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IR_Pin GPIO_PIN_1
#define IR_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define KEY1_SCAN_COUNT 4



#define RUN   1
#define STOP   0


#define MANAUL   0
#define AUTO   1


#define DEVICE_RUN  ( g_config.current_cmd = RUN)
#define DEVICE_STOP    (g_config.current_cmd = STOP)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
