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
 
//�������,�����ڲ�flash
//���뱣֤Ϊ4�ֽڶ��䣬���Ҵ�С������4��������
#define DEVICE_BOARD_CONFIG_ID		0xA5A87A6E 			//���ID
typedef struct
{
	
/****************ͷ����ע�������Ϣ������ʲô�豸�����ÿ�ͷ���������¸�ʽ**************************************************/
	uin32_t				ID;							//ID,����Ƿ���й�����
	char 			SN[16];						//Ψһ���к�-���ܽ��������޸�
/***********************************************************************************************************************/
	uin32_t		       timer_minute_num;		// //��ʱʱ�䣬��λ����
///////////////////////////////////////////////////////////////////////////////
//////
}CONFIG_TYPE;
extern CONFIG_TYPE 	g_SYS_Config;							//ϵͳ����
 
bool CONFIG_WriteSN(char pSN[16]);							//�޸�SN;
void CONFIG_Default(CONFIG_TYPE *pConfig, bool isAdmin);	//�ָ�����Ϊ����ģʽ
bool CONFIG_Check(CONFIG_TYPE *pConfig,bool isCheckID);		//������ò����Ƿ�Ϸ�
void CONFIG_Init(void);										//��ʼ������
bool CONFIG_WriteConfig(CONFIG_TYPE *pConfig);				
//д�����ã�����Զ�̻���λ����������
bool CONFIG_CheckSN(char pSN[16]);							//���SN����Ч��
 
 



#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
