#ifndef __AX_MOTOR_H
#define __AX_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

//接口函数
void AX_MOTOR_Init(void); //电机PWM控制初始化
void AX_MOTOR_A_SetSpeed(int16_t speed);   //电机A控制
void AX_MOTOR_B_SetSpeed(int16_t speed);   //电机B控制

#endif


