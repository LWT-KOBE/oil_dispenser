#ifndef __MYCAN_H
#define __MYCAN_H

#include "stm32f10x.h" 
#include "task.h"

//����������ת������
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
	
	u8 Reset_flag;		//����flash
	
	u8 Rebot_flag;		//�����豸
	
	u8 Flash_SaveFlag;	//��������
	//�趨�����λ�á���float��
	formatTrans32Struct_t SetPos[4];
	
	//�趨������ٶȡ���float��	
	formatTrans32Struct_t SetVel[4];
	
	//�趨����ĵ���	
	formatTrans16Struct_t SetCur[4];	
	
	//�趨������ٶȼ��ٶ�
	formatTrans16Struct_t Acce[4];
	
	//�趨������ٶȼ��ٶ�
	formatTrans16Struct_t Dece[4];
	
	uint32_t loops;
} CAN1Struct_t;


/*Odrive ��CAN���սṹ��*/
typedef struct {
	
	u8 recv[8];
	u8 Recv[8];
	uint8_t current_presence[4];		//��������

	formatTrans16Struct_t Iq_measured[4];//����
	
	formatTrans32Struct_t shadow_count[4];//shadow	
	formatTrans32Struct_t count_in_cpr[4];//CPR
	formatTrans32Struct_t pos_estimate[4];//λ��	
	formatTrans32Struct_t vel_estimate[4];//�ٶ�
		

	formatTrans32Struct_t vel_limit[4]; //�ٶ����ơ�������
	formatTrans32Struct_t current_limit[4];//�������ơ�������
	formatTrans32Struct_t Target_Torque[4];//Ŀ������
	formatTrans32Struct_t Torque_Slope[4];//����б��
	formatTrans32Struct_t NMT[4];//CANOPen�������
	

}CAN1DataRecv_t;

extern CAN1DataRecv_t can1_rx;
extern CAN1Struct_t   can1_tx;
/*CAN���ͽṹ��*/
typedef struct {
	uint8_t cmd;
	uint8_t data[8];
}CANSendStruct_t;

void MyCAN_Init(u8 num);
void MyCAN_Transmit(uint32_t ID, uint8_t Length, uint8_t *Data);
uint8_t MyCAN_ReceiveFlag(void);
void MyCAN_Receive(uint32_t *ID, uint8_t *Length, uint8_t *Data);

#endif
