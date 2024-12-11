#include "stm32f10x.h"                  // Device header
#include "MyCAN.h"
#include "task.h"

uint32_t heapUsed, heapHighWater, dataSramUsed;
CAN1DataRecv_t can1_rx;
CAN1Struct_t   can1_tx;

void *aqCalloc(size_t count, size_t size) {
    char *addr = 0;

    if (count * size) {
//        addr = calloc(count, size);
				addr = malloc(count*size);
//				addr = pvPortMalloc(count * size);

        heapUsed += count * size;
        if (heapUsed > heapHighWater)
            heapHighWater = heapUsed;

 //       if (addr == 0)
 //           AQ_NOTICE("Out of heap memory!\n");
    }

    return addr;
}

void aqFree(void *ptr, size_t count, size_t size) {
    if (ptr) {
        free(ptr);
			//	vPortFree(ptr);
        heapUsed -= count * size;
    }
}


/*
***************************************************
��������CAN1SendData
���ܣ��������CAN����_����֡
��ڲ�����	CANx��CAN1 orCAN2
			ID_CAN��CANID ���0����1��ID��ַ �涨�� AXIS0_ID 0x001   AXIS1_ID 0x002
			CMD_CAN��odrive�ĸ�������  
			len������֡���� 
			CanSendData��CAN���͵����ݽṹ
����ֵ����
Ӧ�÷�Χ���ڲ�����
��ע�� �ú������͵�������֡���м�
***************************************************
*/

void CANSendData(CAN_TypeDef *CANx, uint32_t ID_CAN, uint8_t len,CANSendStruct_t* CanSendData) {
	CanTxMsg *txMessage;
	uint8_t mbox;
	uint8_t count;
	uint16_t i = 0;
	txMessage = (CanTxMsg*)aqCalloc(8,sizeof(CanTxMsg));
	
	//CAN ID 	
	txMessage->StdId = ID_CAN;	
	txMessage->IDE = CAN_Id_Standard;
	txMessage->RTR = CAN_RTR_Data;
	txMessage->DLC = len;
	for (count = 0; count < len; count++) {
		txMessage->Data[count] = (uint8_t)CanSendData->data[count];
	}
	mbox = CAN_Transmit(CANx, txMessage);
	while (CAN_TransmitStatus(CANx,mbox) == 0x00) {
		i++;
		if (i >= 0xFFF)break;
	}
	aqFree(txMessage,8,sizeof(CanTxMsg));
}


//Ĭ��1M������
void MyCAN_Init(u8 num)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	//CAN���ų�ʼ��
	GPIO_InitTypeDef GPIO_InitStructure;
	//CANTX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//CANRX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);//�˿���ӳ��
	
	//����CANͨ�Ų�����
	CAN_InitTypeDef CAN_InitStructure;
	CAN_InitStructure.CAN_Prescaler = num;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_5tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = DISABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_Init(CAN1, &CAN_InitStructure);
	
	//CAN�������
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
//	CAN_FilterInitStructure.CAN_FilterNumber=0;	//������0
//	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 	//����λģʽ
//	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_16bit; 	//32λ�� 
//	CAN_FilterInitStructure.CAN_FilterIdHigh=(((u32)0x52 << 21)&0xffff0000)>>16;	//32λID
//	CAN_FilterInitStructure.CAN_FilterIdLow=0xffff;
//	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=(((u32)0x52 << 21)&0xffff0000)>>16;//32λMASK
//	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xffff;
//	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
//	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;//���������0
//	CAN_FilterInit(&CAN_FilterInitStructure);
	
	CAN_FilterInitStructure.CAN_FilterNumber=0;	//������0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 	//����λģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; 	//32λ�� 
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;	//32λID
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;//���������0
	CAN_FilterInit(&CAN_FilterInitStructure);			//�˲�����ʼ��
	
	
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�����ж�����   
	
	NVIC_InitTypeDef		NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;// �����ȼ�Ϊ4
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;// �����ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);
}

/*
***************************************************
��������MyCAN_Transmit
���ܣ�CAN1��������
��ڲ�����	ID���������ݵ�֡ID
			Length���������ݵĳ���
			Data�����͵�����
����ֵ����
Ӧ�÷�Χ���ⲿ����
***************************************************
*/
void MyCAN_Transmit(uint32_t ID, uint8_t Length, uint8_t *Data)
{
	CanTxMsg TxMessage;
	TxMessage.StdId = ID;
	TxMessage.ExtId = ID;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = Length;
	for (uint8_t i = 0; i < Length; i ++)
	{
		TxMessage.Data[i] = Data[i];
	}
	uint8_t TransmitMailbox = CAN_Transmit(CAN1, &TxMessage);
	while (CAN_TransmitStatus(CAN1, TransmitMailbox) != CAN_TxStatus_Ok);
}


/*
***************************************************
��������MyCAN_ReceiveFlag
���ܣ��ж��Ƿ���ܵ���CAN����
��ڲ�������
����ֵ���������ݳɹ��ı�־
Ӧ�÷�Χ���ⲿ����
***************************************************
*/
uint8_t MyCAN_ReceiveFlag(void)
{
	if (CAN_MessagePending(CAN1, CAN_FIFO0) > 0)
	{
		return 1;
	}
	return 0;
}

//CAN���ݽ��մ�����
/*
***************************************************
��������MyCAN_Transmit
���ܣ�CAN1��������
��ڲ�����	ID���������ݵ�֡ID
			Length���������ݵĳ���
			Data�����͵�����
����ֵ����
Ӧ�÷�Χ���ⲿ����
***************************************************
*/
void MyCAN_Receive(uint32_t *ID, uint8_t *Length, uint8_t *Data)
{
	CanRxMsg RxMessage;
	
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
	
	//��ͨ����
	if (RxMessage.IDE == CAN_Id_Standard)
	{
		*ID = RxMessage.StdId;
	}
	else
	{
		//...
	}
	
	//Զ��֡
	if (RxMessage.RTR == CAN_RTR_Data)
	{
		*Length = RxMessage.DLC;
		for (uint8_t i = 0; i < *Length; i ++)
		{
			Data[i] = RxMessage.Data[i];
		}
	}
	else
	{
		//...
	}
}


/*
***************************************************
��������ReadVel_Pos
���ܣ���ȡʵʱ�ٶȡ�λ��
��ڲ�����	
			CanSendData��CAN���͵����ݽṹ
			Spetsnaz������CANOPen�ŷ�������ٶȡ�λ�����ݽṹ��
			axis: ѡ����0����1����� �涨 �� axis0=0  axis1=1 
����ֵ����
Ӧ�÷�Χ���ڲ�����
***************************************************
*/
void ReadVel_Pos(CanRxMsg* CanRevData,CAN1DataRecv_t* Spetsnaz,uint8_t axis) {
		
	
	Spetsnaz->recv[0] = CanRevData->Data[0];	
	Spetsnaz->recv[1] = CanRevData->Data[1];	
	Spetsnaz->recv[2] = CanRevData->Data[2];	
	Spetsnaz->recv[3] = CanRevData->Data[3];
	
	Spetsnaz->recv[4] = CanRevData->Data[4];
	Spetsnaz->recv[5] = CanRevData->Data[5];
	Spetsnaz->recv[6] = CanRevData->Data[6];
	Spetsnaz->recv[7] = CanRevData->Data[7];
}

/*
***************************************************
��������ReadVel_Pos
���ܣ���ȡʵʱ�ٶȡ�λ��
��ڲ�����	
			CanSendData��CAN���͵����ݽṹ
			Spetsnaz������CANOPen�ŷ�������ٶȡ�λ�����ݽṹ��
			axis: ѡ����0����1����� �涨 �� axis0=0  axis1=1 
����ֵ����
Ӧ�÷�Χ���ڲ�����
***************************************************
*/
void Read_car(CanRxMsg* CanRevData,CAN1DataRecv_t* Spetsnaz,uint8_t axis) {
		
	
	Spetsnaz->Recv[0] = CanRevData->Data[0];	
	Spetsnaz->Recv[1] = CanRevData->Data[1];	
	Spetsnaz->Recv[2] = CanRevData->Data[2];	
	Spetsnaz->Recv[3] = CanRevData->Data[3];
	
	Spetsnaz->Recv[4] = CanRevData->Data[4];
	Spetsnaz->Recv[5] = CanRevData->Data[5];
	Spetsnaz->Recv[6] = CanRevData->Data[6];
	Spetsnaz->Recv[7] = CanRevData->Data[7];
}

//CAN�����жϷ�����
u32 rxbuf3;
void USB_LP_CAN1_RX0_IRQHandler(void){
	CanRxMsg can1_rx_msg;
	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET){
		CAN_ClearITPendingBit(CAN1, CAN_IT_FF0);
		CAN_ClearFlag(CAN1, CAN_FLAG_FF0);		
		CAN_Receive(CAN1, CAN_FIFO0, &can1_rx_msg);
		rxbuf3=can1_rx_msg.StdId;
		
		switch(can1_rx_msg.StdId){
			
			//��ȡCANOPen�������ڵ��ź�
			case 82:
				ReadVel_Pos(&can1_rx_msg,&can1_rx,0); 
				break;
			
			case 9:
				Read_car(&can1_rx_msg,&can1_rx,0); 
				break;
			
				
			default:	break;
		}
		
	}		
}

/*
***************************************************
��������CAN1_TX_IRQHandler
���ܣ�CAN1�����ж�
��ע��
***************************************************
*/
void CAN1_TX_IRQHandler(void){
	if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET){
		CAN_ClearITPendingBit(CAN1,CAN_IT_TME);

		/*********�������Զ��岿��**********/
        
        
	}
}

