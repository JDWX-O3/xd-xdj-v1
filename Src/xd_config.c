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
#include "xd_config.h"

#define ConfigItemFlashAddress    ((u32)0x08036000)//��д��ʼ��ַ���ڲ�

/*
��Ȼ�ٷ��ṩ��Ҳ��֪һ�����������������������������£�����32
λϵͳ��һ�����ֽ�=4byte=32bite��һ���ǰ���=2byte=16bite��һ�����ֽ�=1byte=8bite�����в�����

FLASH_Status FLASH_ErasePage(uint32_t Page_Address);
FLASH_Status FLASH_EraseAllPages(void);
FLASH_Status FLASH_EraseOptionBytes(void);

*/

/*
 ��Ȼ�ٷ����Ĳ�ֹ����һ������д���ݣ��ٷ��ṩ��3��

FLASH_Status FLASH_ProgramWord(uint32_t Address, uint32_t Data);
//һ��дһ���֣�����32ϵͳ��һ��д����4���ֽڣ�uint32_t ������С��32bit

FLASH_Status FLASH_ProgramHalfWord(uint32_t Address, uint16_t Data);
//һ��дһ�����֣�����32ϵͳ��һ��д����2���ֽڣ�uint16_t ������С��16bit

FLASH_Status FLASH_ProgramOptionByteData(uint32_t Address, uint8_t Data);
//һ��дһ���ֽڣ�����32ϵͳ��һ��д����1���ֽڣ�uint8_t ������С��8bit
*/


void writeFlashTest2(void)
{
	uint32_t PageError = 0;                    //����PageError,������ִ�����������ᱻ����Ϊ�����FLASH��ַ
	uint16_t Write_Flash_Data = 0x0001;����������
	uint32_t config_Flash_Add = ConfigItemFlashAddress;
	FLASH_EraseInitTypeDef My_Flash;  //���� FLASH_EraseInitTypeDef �ṹ��Ϊ My_Flash
	HAL_FLASH_Unlock();               //����Flash

	My_Flash.TypeErase = FLASH_TYPEERASE_PAGES;  //����Flashִ��ҳ��ֻ����������
	My_Flash.PageAddress = config_Flash_Add;  //����Ҫ�����ĵ�ַ
	My_Flash.NbPages = 1;                        
	//˵��Ҫ������ҳ�����˲���������Min_Data = 1��Max_Data =(���ҳ��-��ʼҳ��ֵ)֮���ֵ

	HAL_FLASHEx_Erase(&My_Flash, &PageError);  //���ò�����������

	//��Flash������д��FLASH_TYPEPROGRAM_HALFWORD ����������	Flash��ַ��16λ�ģ����⻹��32λ��64λ�Ĳ��������з���HAL��Ķ��弴��
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, config_Flash_Add, Write_Flash_Data); 
	HAL_FLASH_Lock(); //��סFlash
}

//��ȡָ����ַ�İ���(16λ����)
//Ҳ�ǰ��հ��ֶ�������ÿ�ζ�2���ֽ����ݷ���
uint16_t FLASH_ReadHalfWord(uint32_t address)
{
	return *(__IO uint16_t*)address;
}


void write_to_flash(void)
{
	u16 buff[1200];
	u16 count_len = 2272 / 2;
	//FLASH_WriteMoreData(StartServerManageFlashAddress,buff,count_len);
}
 
 void read_from_flash(void)
{
	u16 buff[1200];
	u16 count_len = 2272 / 2;
	//FLASH_WriteMoreData(StartServerManageFlashAddress,buff,count_len);
}


