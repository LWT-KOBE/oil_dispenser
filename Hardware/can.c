//#include "can.h" 
//#include "delay.h"
//#include "usart.h"
//#include "key.h"
//u8 g_UT_CAN_ID = 0 ;
// 
//__IO ErrorStatus Rxflag0;
//__IO u8 RxRAM0[100];

//__IO u32 CAN_ID; 
//u8 CanLiveFlag = 0;//1在线
//u16 CanDelayCheckCount = 0;
//extern u8 delaynum2;
////CAN初始化
////tsjw:重新同步跳跃时间单元.范围:CAN_SJW_1tq~ CAN_SJW_4tq
////tbs2:时间段2的时间单元.   范围:CAN_BS2_1tq~CAN_BS2_8tq;
////tbs1:时间段1的时间单元.   范围:CAN_BS1_1tq ~CAN_BS1_16tq
////brp :波特率分频器.范围:1~1024;  tq=(brp)*tpclk1
////波特率=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
////mode:CAN_Mode_Normal,普通模式;CAN_Mode_LoopBack,回环模式;
////Fpclk1的时钟在初始化的时候设置为36M,如果设置CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_LoopBack);
////则波特率为:36M/((8+9+1)*4)=500Kbps
////返回值:0,初始化OK;
////    其他,初始化失败; 
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

//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//使能PORTA时钟	                   											 
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO , ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽
//	GPIO_Init(GPIOB, &GPIO_InitStructure);			//初始化IO

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//上拉输入
//	GPIO_Init(GPIOB, &GPIO_InitStructure);			//初始化IO	
//		

//	GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);//端口重映射	

//	//CAN单元设置
//	CAN_InitStructure.CAN_TTCM=DISABLE;			//非时间触发通信模式  
//	CAN_InitStructure.CAN_ABOM=ENABLE;			//软件自动离线管理	 
//	CAN_InitStructure.CAN_AWUM=ENABLE;			//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
//	CAN_InitStructure.CAN_NART=DISABLE;			//禁止报文自动传送 
//	CAN_InitStructure.CAN_RFLM=DISABLE;		 	//报文不锁定,新的覆盖旧的  
//	CAN_InitStructure.CAN_TXFP=DISABLE;			//优先级由报文标识符决定 
//	CAN_InitStructure.CAN_Mode= mode;	        //模式设置： mode:0,普通模式;1,回环模式; 
//	//设置波特率
//	CAN_InitStructure.CAN_SJW=tsjw;				//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位  CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
//	CAN_InitStructure.CAN_BS1=tbs1; 			//Tbs1=tbs1+1个时间单位CAN_BS1_1tq ~CAN_BS1_16tq
//	CAN_InitStructure.CAN_BS2=tbs2;				//Tbs2=tbs2+1个时间单位CAN_BS2_1tq ~	CAN_BS2_8tq
//	CAN_InitStructure.CAN_Prescaler=brp;        //分频系数(Fdiv)为brp+1	
//	CAN_Init(CAN1, &CAN_InitStructure);        	//初始化CAN1 

////	CAN_FilterInitStructure.CAN_FilterNumber=0;	//过滤器0
////	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 	//屏蔽位模式
////	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; 	//32位宽 
////	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;	//32位ID
////	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
////	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
////	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
////	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
////	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;//激活过滤器0
////	CAN_FilterInit(&CAN_FilterInitStructure);			//滤波器初始化
//	 
//	CAN_FilterInitStructure.CAN_FilterNumber=0;	//过滤器0
//	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 	//屏蔽位模式
//	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_16bit; 	//32位宽 
//	CAN_FilterInitStructure.CAN_FilterIdHigh=(((u32)0x7F << 21)&0xffff0000)>>16;	//32位ID
//	CAN_FilterInitStructure.CAN_FilterIdLow=0xffff;
//	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=(((u32)0x7D << 21)&0xffff0000)>>16;//32位MASK
//	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xffff;
//	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
//	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;//激活过滤器0
//	CAN_FilterInit(&CAN_FilterInitStructure);			//滤波器初始化
//	
//	
//	CAN_FilterInitStructure1.CAN_FilterNumber=1;	//过滤器0
//	CAN_FilterInitStructure1.CAN_FilterMode=CAN_FilterMode_IdMask; 	//屏蔽位模式
//	CAN_FilterInitStructure1.CAN_FilterScale=CAN_FilterScale_16bit; 	//32位宽 
//	CAN_FilterInitStructure1.CAN_FilterIdHigh=(((u32)0x52 << 21)&0xffff0000)>>16;	//32位ID
//	CAN_FilterInitStructure1.CAN_FilterIdLow=0xffff;
//	CAN_FilterInitStructure1.CAN_FilterMaskIdHigh=(((u32)0x52 << 21)&0xffff0000)>>16;//32位MASK
//	CAN_FilterInitStructure1.CAN_FilterMaskIdLow=0xffff;
//	CAN_FilterInitStructure1.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
//	CAN_FilterInitStructure1.CAN_FilterActivation=ENABLE;//激活过滤器0
//	CAN_FilterInit(&CAN_FilterInitStructure1);			//滤波器初始化	
//	
//	CAN_FilterInitStructure1.CAN_FilterNumber=2;	//过滤器0
//	CAN_FilterInitStructure1.CAN_FilterMode=CAN_FilterMode_IdMask; 	//屏蔽位模式
//	CAN_FilterInitStructure1.CAN_FilterScale=CAN_FilterScale_16bit; 	//32位宽 
//	CAN_FilterInitStructure1.CAN_FilterIdHigh=(((u32)0x62 << 21)&0xffff0000)>>16;	//32位ID
//	CAN_FilterInitStructure1.CAN_FilterIdLow=0xffff;
//	CAN_FilterInitStructure1.CAN_FilterMaskIdHigh=(((u32)0x62 << 21)&0xffff0000)>>16;//32位MASK
//	CAN_FilterInitStructure1.CAN_FilterMaskIdLow=0xffff;
//	CAN_FilterInitStructure1.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
//	CAN_FilterInitStructure1.CAN_FilterActivation=ENABLE;//激活过滤器0
//	CAN_FilterInit(&CAN_FilterInitStructure1);			//滤波器初始化	

//	CAN_FilterInitStructure1.CAN_FilterNumber=3;	//过滤器0
//	CAN_FilterInitStructure1.CAN_FilterMode=CAN_FilterMode_IdMask; 	//屏蔽位模式
//	CAN_FilterInitStructure1.CAN_FilterScale=CAN_FilterScale_16bit; 	//32位宽 
//	CAN_FilterInitStructure1.CAN_FilterIdHigh=(((u32)0xff << 21)&0xffff0000)>>16;	//32位ID
//	CAN_FilterInitStructure1.CAN_FilterIdLow=0xffff;
//	CAN_FilterInitStructure1.CAN_FilterMaskIdHigh=(((u32)0xff << 21)&0xffff0000)>>16;//32位MASK
//	CAN_FilterInitStructure1.CAN_FilterMaskIdLow=0xffff;
//	CAN_FilterInitStructure1.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
//	CAN_FilterInitStructure1.CAN_FilterActivation=ENABLE;//激活过滤器0
//	CAN_FilterInit(&CAN_FilterInitStructure1);			//滤波器初始化	
//	
//	CAN_FilterInitStructure1.CAN_FilterNumber=4;	//过滤器0
//	CAN_FilterInitStructure1.CAN_FilterMode=CAN_FilterMode_IdMask; 	//屏蔽位模式
//	CAN_FilterInitStructure1.CAN_FilterScale=CAN_FilterScale_16bit; 	//32位宽 
//	CAN_FilterInitStructure1.CAN_FilterIdHigh=(((u32)0x53 << 21)&0xffff0000)>>16;	//32位ID
//	CAN_FilterInitStructure1.CAN_FilterIdLow=0xffff;
//	CAN_FilterInitStructure1.CAN_FilterMaskIdHigh=(((u32)0x53 << 21)&0xffff0000)>>16;//32位MASK
//	CAN_FilterInitStructure1.CAN_FilterMaskIdLow=0xffff;
//	CAN_FilterInitStructure1.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
//	CAN_FilterInitStructure1.CAN_FilterActivation=ENABLE;//激活过滤器0
//	CAN_FilterInit(&CAN_FilterInitStructure1);			//滤波器初始化		
//	

//#if CAN_RX0_INT_ENABLE 
//	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);				//FIFO0消息挂号中断允许.		    

//	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
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



//#if CAN_RX0_INT_ENABLE	//使能RX0中断
////中断服务函数			    
//void USB_LP_CAN1_RX0_IRQHandler(void)
//{
//  	CanRxMsg RxMessage;

//    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);

//	//APP程序CAN中断内,刷固件跳转程序
//	//IAP_APP_CAN_ReStart(RxMessage);
//	SaveData(RxMessage);
//	//CheckCarCanCmd();
//	CanLiveFlag = 1;
//	CanDelayCheckCount = 0;
//	
//}
//#endif






////can口接收数据查询
////buf:数据缓存区;	 
////返回值:0,无数据被收到;
////		 其他,接收的数据长度;
//u8 Can_Receive_Msg(u8 *buf)
//{		   		   
// 	u32 i;
//	CanRxMsg RxMessage;
//    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//没有接收到数据,直接退出 
//    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//读取数据	
//    for(i=0;i<8;i++)
//    buf[i]=RxMessage.Data[i];  
//	return RxMessage.DLC;	
//}














