#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "Timer.h"
#include "task.h"
#include "motor.h"
#include "system.h"
#include "Key.h"
#include "ax_motor.h" 
#include "ax_encoder.h" 
#include "vofa.h"
#include "usart.h"
#include "MyCAN.h"
#include "bsp_can.h"
#include "can.h" 
#include "LED.h"

u8 a1[4] = {5,4,3,2};
CanTxMsg TxMessage;              //发送缓冲区
int main(void)
{
	/*模块初始化*/
	Key_Init();
	//Motor_Init();
	LED_Init();
	AX_MOTOR_Init();
	Timer_Init();		//定时中断初始化
	//AX_ENCODER_B_Init();
	AX_ENCODER_A_Init();
	uart_init(115200);
	MyCAN_Init(16);		//CAN波特率250K
	//CAN_Config();
	
	//CAN_Mode_Init(4,9,8,1,0);
	
	EW.first_num = PBin(6);
	while (1)
	{	
		//MyCAN_Transmit(0x76, 4, a1);
//		CAN_SetMsg(&TxMessage);
//		
//		/*把报文存储到发送邮箱，发送*/
//        CAN_Transmit(CANx, &TxMessage);
		//AX_MOTOR_B_SetSpeed(1100);
		
	}
}


