#ifndef __DRIVER_SR04_H__
#define __DRIVER_SR04_H__

#include "main.h"

#define Tirg_ON		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET)
#define Tirg_OFF	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET)

void SR04_Tirgger(void);//�����ź�

uint32_t GetDistance(void);

#endif
