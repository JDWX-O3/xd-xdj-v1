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

#define ConfigItemFlashAddress    ((u32)0x08036000)//读写起始地址（内部

/*
当然官方提供的也不知一个擦除函数，而是三个，具体如下，对于32
位系统：一个是字节=4byte=32bite；一个是半字=2byte=16bite；一个是字节=1byte=8bite；进行擦除。

FLASH_Status FLASH_ErasePage(uint32_t Page_Address);
FLASH_Status FLASH_EraseAllPages(void);
FLASH_Status FLASH_EraseOptionBytes(void);

*/

/*
 当然官方给的不止是这一个函数写数据，官方提供了3个

FLASH_Status FLASH_ProgramWord(uint32_t Address, uint32_t Data);
//一次写一个字，对于32系统，一次写的是4个字节，uint32_t 变量大小，32bit

FLASH_Status FLASH_ProgramHalfWord(uint32_t Address, uint16_t Data);
//一次写一个半字，对于32系统，一次写的是2个字节，uint16_t 变量大小，16bit

FLASH_Status FLASH_ProgramOptionByteData(uint32_t Address, uint8_t Data);
//一次写一个字节，对于32系统，一次写的是1个字节，uint8_t 变量大小，8bit
*/


void writeFlashTest2(void)
{
	uint32_t PageError = 0;                    //设置PageError,如果出现错误这个变量会被设置为出错的FLASH地址
	uint16_t Write_Flash_Data = 0x0001;　　　　　
	uint32_t config_Flash_Add = ConfigItemFlashAddress;
	FLASH_EraseInitTypeDef My_Flash;  //声明 FLASH_EraseInitTypeDef 结构体为 My_Flash
	HAL_FLASH_Unlock();               //解锁Flash

	My_Flash.TypeErase = FLASH_TYPEERASE_PAGES;  //标明Flash执行页面只做擦除操作
	My_Flash.PageAddress = config_Flash_Add;  //声明要擦除的地址
	My_Flash.NbPages = 1;                        
	//说明要擦除的页数，此参数必须是Min_Data = 1和Max_Data =(最大页数-初始页的值)之间的值

	HAL_FLASHEx_Erase(&My_Flash, &PageError);  //调用擦除函数擦除

	//对Flash进行烧写，FLASH_TYPEPROGRAM_HALFWORD 声明操作的	Flash地址的16位的，此外还有32位跟64位的操作，自行翻查HAL库的定义即可
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, config_Flash_Add, Write_Flash_Data); 
	HAL_FLASH_Lock(); //锁住Flash
}

//读取指定地址的半字(16位数据)
//也是按照半字读出，即每次读2个字节数据返回
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


