#ifndef __AX_MOTOR_H
#define __AX_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

//�ӿں���
void AX_MOTOR_Init(void); //���PWM���Ƴ�ʼ��
void AX_MOTOR_A_SetSpeed(int16_t speed);   //���A����
void AX_MOTOR_B_SetSpeed(int16_t speed);   //���B����

#endif


