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
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_def.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_rcc.h"
#include "xd_dig_led.h"
#include "xd_tm1650.h"
#include "xd_key.h"
#include "xd_relay.h"
#include "xd_infrared.h"
#include "xd_x9c103.h"
#include "xd_sensor.h"
#include "main.h"
#include "multi_button.h"


extern userOpAttr_t g_config;
 
int32_t O3_Realy_Start(void)
{

	if (g_config.fan_run_status == RUN){
		RELAY_OUT_3_SET;
		g_config.o3_run_status = RUN;
		return 0;
	}
	else{
		return -1;
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
// ·½·¨Ò»
  
int32_t O3_Realy_Stop(void)
{
	RELAY_OUT_3_RESET;
	g_config.o3_run_status = STOP;
	return 0;
}




int32_t FAN_Realy_Start(void)
{
	RELAY_OUT_2_SET;
	g_config.fan_run_status = RUN;
	return 0;
}

  
int32_t FAN_Realy_Stop(void)
{

	if (g_config.o3_run_status == STOP){
		RELAY_OUT_2_RESET;
		g_config.fan_run_status = STOP;
		return 0;
	}
	else{
		return -1;
	}
}





