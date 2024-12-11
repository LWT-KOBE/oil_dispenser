#ifndef __TASK_H
#define __TASK_H
#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "ax_motor.h" 
#include "ax_encoder.h" 
#include "vofa.h"
#include "usart.h"
#include "Key.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define Motor_up   500
#define Motor_down 500
#define Motor_End  500

typedef struct{
	
	u8 start_flag;
	u8 start_flag1;
	u8 MotorA_flag;
	u8 MotorB_flag;
	u8 MotorC_flag;
	u8 MotorD_flag;
	
	int start_num;
	int start1_num;
	int end_num;
	int end_num1;
	int end_num2;
	
	int Test_Motor1;
	int Test_Motor2;
	int Test_Motor3;
	int Test_Motor4;
	
	int Test_Motor1_d;
	int Test_Motor2_d;
	int Test_Motor3_d;
	int Test_Motor4_d;
	
	int Motor_NUM1;
	int Motor_NUM2;
	int Motor_NUM3;
	int Motor_NUM4;
	
	int MotorA_END;
	int MotorB_END;
	int MotorC_END;
	int MotorD_END;
	
	int frist_a;
	int frist_b;
	int frist_c;
	int frist_d;
	int frist_function;

	
}MOTOR;

typedef struct{
	uint8_t finish;				//��ɱ�־λ
	uint8_t change;				//�ı��־λ
	uint8_t first_num;			//��һ���ϵ�״̬
	int mileage;				//���
	int time;					//����ʱ��
	int time_period;			//ʱ����
	float speed;				//��ǰ�ٶ�
	float Encoder_pr;			//����������ֵ
	double Current_Mileage;		//��ǰ���
	double target_mileage;		//�������
}Encoding_Wheel;


void *aqCalloc(size_t count, size_t size);			//��̬����ռ�
void aqFree(void *ptr, size_t count, size_t size);	//�ͷſռ�

extern Encoding_Wheel EW;

#endif
