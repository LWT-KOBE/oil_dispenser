//#include "can.h" 
//#include "delay.h"
//#include "usart.h"
//#include "key.h"
//u8 g_UT_CAN_ID = 0 ;
// 
//__IO ErrorStatus Rxflag0;
//__IO u8 RxRAM0[100];

//__IO u32 CAN_ID; 
//u8 CanLiveFlag = 0;//1����
//u16 CanDelayCheckCount = 0;
//extern u8 delaynum2;
////CAN��ʼ��
////tsjw:����ͬ����Ծʱ�䵥Ԫ.��Χ:CAN_SJW_1tq~ CAN_SJW_4tq
////tbs2:ʱ���2��ʱ�䵥Ԫ.   ��Χ:CAN_BS2_1tq~CAN_BS2_8tq;
////tbs1:ʱ���1��ʱ�䵥Ԫ.   ��Χ:CAN_BS1_1tq ~CAN_BS1_16tq
////brp :�����ʷ�Ƶ��.��Χ:1~1024;  tq=(brp)*tpclk1
////������=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
////mode:CAN_Mode_Normal,��ͨģʽ;CAN_Mode_LoopBack,�ػ�ģʽ;
////Fpclk1��ʱ���ڳ�ʼ����ʱ������Ϊ36M,�������CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_LoopBack);
////������Ϊ:36M/((8+9+1)*4)=500Kbps
////����ֵ:0,��ʼ��OK;
////    ����,��ʼ��ʧ��; 
//u8 CAN_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
//{ 
//	GPIO_InitTypeDef 		GPIO_InitStructure; 
//	CAN_InitTypeDef        	CAN_InitStructure;
//	CAN_FilterInitTypeDef  	CAN_FilterInitStructure;
//	CAN_FilterInitTypeDef  	CAN_FilterInitStructure1;
////	CAN_FilterInitTypeDef  	CAN_FilterInitStructure2;
//#if CAN_RX0_INT_ENABLE 
//	NVIC_InitTypeDef  		NVIC_InitStructure;
//#endif

//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//ʹ��PORTAʱ��	                   											 
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO , ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//��������
//	GPIO_Init(GPIOB, &GPIO_InitStructure);			//��ʼ��IO

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//��������
//	GPIO_Init(GPIOB, &GPIO_InitStructure);			//��ʼ��IO	
//		

//	GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);//�˿���ӳ��	

//	//CAN��Ԫ����
//	CAN_InitStructure.CAN_TTCM=DISABLE;			//��ʱ�䴥��ͨ��ģʽ  
//	CAN_InitStructure.CAN_ABOM=ENABLE;			//�����Զ����߹���	 
//	CAN_InitStructure.CAN_AWUM=ENABLE;			//˯��ģʽͨ����������(���CAN->MCR��SLEEPλ)
//	CAN_InitStructure.CAN_NART=DISABLE;			//��ֹ�����Զ����� 
//	CAN_InitStructure.CAN_RFLM=DISABLE;		 	//���Ĳ�����,�µĸ��Ǿɵ�  
//	CAN_InitStructure.CAN_TXFP=DISABLE;			//���ȼ��ɱ��ı�ʶ������ 
//	CAN_InitStructure.CAN_Mode= mode;	        //ģʽ���ã� mode:0,��ͨģʽ;1,�ػ�ģʽ; 
//	//���ò�����
//	CAN_InitStructure.CAN_SJW=tsjw;				//����ͬ����Ծ����(Tsjw)Ϊtsjw+1��ʱ�䵥λ  CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
//	CAN_InitStructure.CAN_BS1=tbs1; 			//Tbs1=tbs1+1��ʱ�䵥λCAN_BS1_1tq ~CAN_BS1_16tq
//	CAN_InitStructure.CAN_BS2=tbs2;				//Tbs2=tbs2+1��ʱ�䵥λCAN_BS2_1tq ~	CAN_BS2_8tq
//	CAN_InitStructure.CAN_Prescaler=brp;        //��Ƶϵ��(Fdiv)Ϊbrp+1	
//	CAN_Init(CAN1, &CAN_InitStructure);        	//��ʼ��CAN1 

////	CAN_FilterInitStructure.CAN_FilterNumber=0;	//������0
////	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 	//����λģʽ
////	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; 	//32λ�� 
////	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;	//32λID
////	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
////	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
////	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
////	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
////	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;//���������0
////	CAN_FilterInit(&CAN_FilterInitStructure);			//�˲�����ʼ��
//	 
//	CAN_FilterInitStructure.CAN_FilterNumber=0;	//������0
//	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 	//����λģʽ
//	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_16bit; 	//32λ�� 
//	CAN_FilterInitStructure.CAN_FilterIdHigh=(((u32)0x7F << 21)&0xffff0000)>>16;	//32λID
//	CAN_FilterInitStructure.CAN_FilterIdLow=0xffff;
//	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=(((u32)0x7D << 21)&0xffff0000)>>16;//32λMASK
//	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xffff;
//	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
//	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;//���������0
//	CAN_FilterInit(&CAN_FilterInitStructure);			//�˲�����ʼ��
//	
//	
//	CAN_FilterInitStructure1.CAN_FilterNumber=1;	//������0
//	CAN_FilterInitStructure1.CAN_FilterMode=CAN_FilterMode_IdMask; 	//����λģʽ
//	CAN_FilterInitStructure1.CAN_FilterScale=CAN_FilterScale_16bit; 	//32λ�� 
//	CAN_FilterInitStructure1.CAN_FilterIdHigh=(((u32)0x52 << 21)&0xffff0000)>>16;	//32λID
//	CAN_FilterInitStructure1.CAN_FilterIdLow=0xffff;
//	CAN_FilterInitStructure1.CAN_FilterMaskIdHigh=(((u32)0x52 << 21)&0xffff0000)>>16;//32λMASK
//	CAN_FilterInitStructure1.CAN_FilterMaskIdLow=0xffff;
//	CAN_FilterInitStructure1.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
//	CAN_FilterInitStructure1.CAN_FilterActivation=ENABLE;//���������0
//	CAN_FilterInit(&CAN_FilterInitStructure1);			//�˲�����ʼ��	
//	
//	CAN_FilterInitStructure1.CAN_FilterNumber=2;	//������0
//	CAN_FilterInitStructure1.CAN_FilterMode=CAN_FilterMode_IdMask; 	//����λģʽ
//	CAN_FilterInitStructure1.CAN_FilterScale=CAN_FilterScale_16bit; 	//32λ�� 
//	CAN_FilterInitStructure1.CAN_FilterIdHigh=(((u32)0x62 << 21)&0xffff0000)>>16;	//32λID
//	CAN_FilterInitStructure1.CAN_FilterIdLow=0xffff;
//	CAN_FilterInitStructure1.CAN_FilterMaskIdHigh=(((u32)0x62 << 21)&0xffff0000)>>16;//32λMASK
//	CAN_FilterInitStructure1.CAN_FilterMaskIdLow=0xffff;
//	CAN_FilterInitStructure1.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
//	CAN_FilterInitStructure1.CAN_FilterActivation=ENABLE;//���������0
//	CAN_FilterInit(&CAN_FilterInitStructure1);			//�˲�����ʼ��	

//	CAN_FilterInitStructure1.CAN_FilterNumber=3;	//������0
//	CAN_FilterInitStructure1.CAN_FilterMode=CAN_FilterMode_IdMask; 	//����λģʽ
//	CAN_FilterInitStructure1.CAN_FilterScale=CAN_FilterScale_16bit; 	//32λ�� 
//	CAN_FilterInitStructure1.CAN_FilterIdHigh=(((u32)0xff << 21)&0xffff0000)>>16;	//32λID
//	CAN_FilterInitStructure1.CAN_FilterIdLow=0xffff;
//	CAN_FilterInitStructure1.CAN_FilterMaskIdHigh=(((u32)0xff << 21)&0xffff0000)>>16;//32λMASK
//	CAN_FilterInitStructure1.CAN_FilterMaskIdLow=0xffff;
//	CAN_FilterInitStructure1.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
//	CAN_FilterInitStructure1.CAN_FilterActivation=ENABLE;//���������0
//	CAN_FilterInit(&CAN_FilterInitStructure1);			//�˲�����ʼ��	
//	
//	CAN_FilterInitStructure1.CAN_FilterNumber=4;	//������0
//	CAN_FilterInitStructure1.CAN_FilterMode=CAN_FilterMode_IdMask; 	//����λģʽ
//	CAN_FilterInitStructure1.CAN_FilterScale=CAN_FilterScale_16bit; 	//32λ�� 
//	CAN_FilterInitStructure1.CAN_FilterIdHigh=(((u32)0x53 << 21)&0xffff0000)>>16;	//32λID
//	CAN_FilterInitStructure1.CAN_FilterIdLow=0xffff;
//	CAN_FilterInitStructure1.CAN_FilterMaskIdHigh=(((u32)0x53 << 21)&0xffff0000)>>16;//32λMASK
//	CAN_FilterInitStructure1.CAN_FilterMaskIdLow=0xffff;
//	CAN_FilterInitStructure1.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
//	CAN_FilterInitStructure1.CAN_FilterActivation=ENABLE;//���������0
//	CAN_FilterInit(&CAN_FilterInitStructure1);			//�˲�����ʼ��		
//	

//#if CAN_RX0_INT_ENABLE 
//	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);				//FIFO0��Ϣ�Һ��ж�����.		    

//	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//#endif
//	return 0;
//}   
// 

//void SaveData(CanRxMsg temp_CAN_Msg)
//{ 
//  u8 i; 
//	
//	CAN_ID = temp_CAN_Msg.StdId;
//	
//	if(temp_CAN_Msg.IDE == CAN_Id_Standard)
//	{
//		for(i = 0; i < 8; i++)
//		{
//			RxRAM0[i] = temp_CAN_Msg.Data[i];
//		}
//	}	
//		Rxflag0 = SUCCESS;

//}



//#if CAN_RX0_INT_ENABLE	//ʹ��RX0�ж�
////�жϷ�����			    
//void USB_LP_CAN1_RX0_IRQHandler(void)
//{
//  	CanRxMsg RxMessage;

//    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);

//	//APP����CAN�ж���,ˢ�̼���ת����
//	//IAP_APP_CAN_ReStart(RxMessage);
//	SaveData(RxMessage);
//	//CheckCarCanCmd();
//	CanLiveFlag = 1;
//	CanDelayCheckCount = 0;
//	
//}
//#endif






////can�ڽ������ݲ�ѯ
////buf:���ݻ�����;	 
////����ֵ:0,�����ݱ��յ�;
////		 ����,���յ����ݳ���;
//u8 Can_Receive_Msg(u8 *buf)
//{		   		   
// 	u32 i;
//	CanRxMsg RxMessage;
//    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//û�н��յ�����,ֱ���˳� 
//    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//��ȡ����	
//    for(i=0;i<8;i++)
//    buf[i]=RxMessage.Data[i];  
//	return RxMessage.DLC;	
//}













