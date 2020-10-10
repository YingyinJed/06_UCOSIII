#ifndef __KEY_H
#define __KEY_H	 

#include "main.h"
#include "delay.h"

/*下面的方式是通过直接操作库函数方式读取IO*/
#define KEY_0 		HAL_GPIO_ReadPin(KEY_0_GPIO_Port,KEY_0_Pin) //PE4
#define KEY_1 		HAL_GPIO_ReadPin(KEY_1_GPIO_Port,KEY_1_Pin)	//PE3 
#define KEY_2 		HAL_GPIO_ReadPin(KEY_2_GPIO_Port,KEY_2_Pin) //PE2
#define WK_UP 		HAL_GPIO_ReadPin(WK_UP_GPIO_Port,WK_UP_Pin)	//PA0

#define KEY0_PRES 	1
#define KEY1_PRES		2
#define KEY2_PRES		3
#define WKUP_PRES   4

uint8_t KEY_Scan(uint8_t mode);  		//按键扫描函数

#endif
