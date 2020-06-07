/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : xd_config.h
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
#ifndef __XD_CONFIG_H
#define __XD_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif




#include "system.h"
#include "rtc.h"
#include "board.h"
 
//配置相关,用于内部flash
//必须保证为4字节对其，并且大小必须是4的整数倍
#define DEVICE_BOARD_CONFIG_ID		0xA5A87A6E 			//标记ID
typedef struct
{
	
/****************头部的注册相关信息，不管什么设备的配置开头必须是以下格式**************************************************/
	uin32_t				ID;							//ID,标记是否进行过配置
	char 			SN[16];						//唯一序列号-不能进行随意修改
/***********************************************************************************************************************/
	uin32_t		       timer_minute_num;		// //定时时间，单位分钟
///////////////////////////////////////////////////////////////////////////////
//////
}CONFIG_TYPE;
extern CONFIG_TYPE 	g_SYS_Config;							//系统配置
 
bool CONFIG_WriteSN(char pSN[16]);							//修改SN;
void CONFIG_Default(CONFIG_TYPE *pConfig, bool isAdmin);	//恢复配置为出厂模式
bool CONFIG_Check(CONFIG_TYPE *pConfig,bool isCheckID);		//检查配置参数是否合法
void CONFIG_Init(void);										//初始化配置
bool CONFIG_WriteConfig(CONFIG_TYPE *pConfig);				
//写入配置，用于远程或上位机更新配置
bool CONFIG_CheckSN(char pSN[16]);							//检查SN的有效性
 
 



#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
