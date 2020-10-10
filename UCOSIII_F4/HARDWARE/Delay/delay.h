#ifndef __DELAY_H
#define __DELAY_H

#include "main.h"
#include "tim.h"
//利用分频出84MHz的频率时钟驱动TIM7完成的以下延时
void delay_us(uint16_t us);
//利用滴答时钟完成的延时
void delay_ms(int32_t nms);

#endif
