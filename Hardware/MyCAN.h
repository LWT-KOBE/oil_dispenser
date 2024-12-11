#ifndef __MYCAN_H
#define __MYCAN_H

#include "stm32f10x.h" 
#include "task.h"

//联合体用于转换数据
typedef union{
	u8 		u8_temp[4];
	float float_temp;
	s32 	s32_temp;
	u32		u32_temp;
} formatTrans32Struct_t;

typedef union{
	u8 		u8_temp[2];
	s16 	s16_temp;
	u16		u16_temp;	
} formatTrans16Struct_t;


typedef struct {
	
	u8 Reset_flag;		//重置flash
	
	u8 Rebot_flag;		//重启设备
	
	u8 Flash_SaveFlag;	//保存配置
	//设定电机的位置——float型
	formatTrans32Struct_t SetPos[4];
	
	//设定电机的速度——float型	
	formatTrans32Struct_t SetVel[4];
	
	//设定电机的电流	
	formatTrans16Struct_t SetCur[4];	
	
	//设定电机的速度加速度
	formatTrans16Struct_t Acce[4];
	
	//设定电机的速度减速度
	formatTrans16Struct_t Dece[4];
	
	uint32_t loops;
} CAN1Struct_t;


/*Odrive 的CAN接收结构体*/
typedef struct {
	
	u8 recv[8];
	u8 Recv[8];
	uint8_t current_presence[4];		//电流存在

	formatTrans16Struct_t Iq_measured[4];//电流
	
	formatTrans32Struct_t shadow_count[4];//shadow	
	formatTrans32Struct_t count_in_cpr[4];//CPR
	formatTrans32Struct_t pos_estimate[4];//位置	
	formatTrans32Struct_t vel_estimate[4];//速度
		

	formatTrans32Struct_t vel_limit[4]; //速度限制——接收
	formatTrans32Struct_t current_limit[4];//电流限制——接收
	formatTrans32Struct_t Target_Torque[4];//目标力矩
	formatTrans32Struct_t Torque_Slope[4];//力矩斜率
	formatTrans32Struct_t NMT[4];//CANOPen网络管理
	

}CAN1DataRecv_t;

extern CAN1DataRecv_t can1_rx;
extern CAN1Struct_t   can1_tx;
/*CAN发送结构体*/
typedef struct {
	uint8_t cmd;
	uint8_t data[8];
}CANSendStruct_t;

void MyCAN_Init(u8 num);
void MyCAN_Transmit(uint32_t ID, uint8_t Length, uint8_t *Data);
uint8_t MyCAN_ReceiveFlag(void);
void MyCAN_Receive(uint32_t *ID, uint8_t *Length, uint8_t *Data);

#endif
