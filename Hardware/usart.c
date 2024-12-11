#include "system.h"
#include "usart.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"

#define 	RECV_BUF_SIZE 	400

#define 	SEND_BUF_SIZE 	200
//定义存储接收字符串的数组
char g_usart1_recv_buf[RECV_BUF_SIZE] = {0};


//定义存储发送字符串的数组
char g_usart1_send_buf[SEND_BUF_SIZE] = {0};

//定义接收字符的计数
int g_usart1_recv_cnt = 0;

//////////////////////////////////////////////////////////////////////////////////
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif
//////////////////////////////////////////////////////////////////////////////////
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32开发板
//串口1初始化
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/8/18
//版本：V1.5
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved
//********************************************************************************
//V1.3修改说明
//支持适应不同频率下的串口波特率设置.
//加入了对printf的支持
//增加了串口接收命令功能.
//修正了printf第一个字符丢失的bug
//V1.4修改说明
//1,修改串口初始化IO的bug
//2,修改了USART_RX_STA,使得串口最大接收字节数为2的14次方
//3,增加了USART_REC_LEN,用于定义串口最大允许接收的字节数(不大于2的14次方)
//4,修改了EN_USART1_RX的使能方式
//V1.5修改说明
//1,增加了对UCOSII的支持
//////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB
#if 1
#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE
{
    int handle;

};

FILE __stdout;
//定义_sys_exit()以避免使用半主机模式
void _sys_exit(int x)
{
    x = x;
}
//重定义fputc函数
int fputc(int ch, FILE *f)
{
    while((USART1->SR&0X40)==0);//循环发送,直到发送完毕
    USART1->DR = (u8) ch;
    return ch;
}
#endif

/*使用microLib的方法*/
/*
int fputc(int ch, FILE *f)
{
USART_SendData(USART1, (uint8_t) ch);

while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}

   return ch;
}
int GetKey (void)  {

   while (!(USART1->SR & USART_FLAG_RXNE));

   return ((int)(USART1->DR & 0x1FF));
}
*/

#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误
u8 	USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
u8  USART_TX_BUF[USART_REC_LEN];   //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记

void uart_init(u32 bound) {
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟

    //USART1_TX   GPIOA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9

    //USART1_RX	  GPIOA.10初始化
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10

    //Usart1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

    //USART 初始化设置

    USART_InitStructure.USART_BaudRate = bound;//串口波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

    USART_Init(USART1, &USART_InitStructure); //初始化串口1
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
    USART_Cmd(USART1, ENABLE);                    //使能串口1

}

void USART1_IRQHandler(void)                	//串口1中断服务程序
{
    u8 Res;
#if SYSTEM_SUPPORT_OS 		//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
    OSIntEnter();
#endif
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
    {
        Res =USART_ReceiveData(USART1);	//读取接收到的数据
		Serial1Data(Res);
        if((USART_RX_STA&0x8000)==0)//接收未完成
        {
            if(USART_RX_STA&0x4000)//接收到了0x0d
            {
                if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
                else USART_RX_STA|=0x8000;	//接收完成了
            }
            else //还没收到0X0D
            {
                if(Res==0x0d)USART_RX_STA|=0x4000;
                else
                {
                    USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
                    USART_RX_STA++;
                    if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收
                }
            }
        }
    }
#if SYSTEM_SUPPORT_OS 	//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
    OSIntExit();
#endif
}

#endif
void u1_printf(char* fmt,...)
{
    u16 i,j;
    va_list ap;
    va_start(ap,fmt);
    vsprintf((char*)USART_TX_BUF,fmt,ap);
    va_end(ap);
    i=strlen((const char*)USART_TX_BUF);//此次发送数据的长度
    for(j=0; j<i; j++) //循环发送数据
    {
        while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);  //等待上次传输完成
        USART_SendData(USART1,(uint8_t)USART_TX_BUF[j]); 	 //发送数据到串口1
    }
}


void u1_SendByte(uint8_t Byte)		//发送一个字节数据
{
	USART_SendData(USART1, Byte);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

void u1_SendArray(uint8_t *Array, uint16_t Length)	//发送一个数组
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		u1_SendByte(Array[i]);
	}
}

void u1_SendString(char *String)	//发送一个字符串数据
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		u1_SendByte(String[i]);
	}
}

uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		u1_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
	}
}



void uart3_init(u32 bound) {
    GPIO_InitTypeDef GPIO_InitStructure;
   USART_InitTypeDef USART_InitStructure;
   NVIC_InitTypeDef NVIC_InitStructure; 

   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE); //使能UART3所在GPIOB的时钟
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //使能串口的RCC时钟

   //串口使用的GPIO口配置
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//设置USART3的RX接口是PB11
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//接口模式 浮空输入
   GPIO_Init(GPIOB, &GPIO_InitStructure);

   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//设置USART3的TX接口是PB10
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//输出速度50MHz
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//接口模式 复用推挽输出
   GPIO_Init(GPIOB, &GPIO_InitStructure);

   //配置串口
   USART_InitStructure.USART_BaudRate = bound;
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
   USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
   USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

   USART_Init(USART3, &USART_InitStructure);//配置串口3
   USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//使能串口接收中断  
   //USART_ITConfig(USART3, USART_IT_TXE, ENABLE);//串口发送中断在发送数据时开启
   USART_Cmd(USART3, ENABLE);//使能串口3

   //串口中断配置
   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
   NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//允许USART3中断
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//中断等级
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);                //使能串口1

}


void usart3_send(u8 data)
{
	USART3->DR = data;
	while((USART3->SR&0x40)==0);	
}

void u3_SendByte(uint8_t Byte)		//发送一个字节数据
{
	USART_SendData(USART3, Byte);
	while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
}

void u3_SendArray(uint8_t *Array, uint16_t Length)	//发送一个数组
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		u3_SendByte(Array[i]);
	}
}

void u3_SendString(char *String)	//发送一个字符串数据
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		usart3_send(String[i]);
	}
}

void USART3_IRQHandler(void)                	//串口3中断服务程序
{
	u8 Res;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) //Check if data is received //判断是否接收到数据
	{
		Res = USART_ReceiveData(USART3);	//读取接收到的数据
		if((USART_RX_STA&0x8000)==0)//接收未完成
			{
			if(USART_RX_STA&0x4000)//接收到了0x0d
				{
				if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else USART_RX_STA|=0x8000;	//接收完成了 
				}
				else //还没收到0X0D
				{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
					{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
					}		 
				}
			}
		
		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	}
}

//串口1接收发送处理函数
void Serial1Data(uint8_t ucData){
		
		//保存接收的字符到字符串数组的对应的位置
		g_usart1_recv_buf[g_usart1_recv_cnt] =ucData;
		//接收字符的计数
		g_usart1_recv_cnt++;
		
	
			
	
		//当接收到符合格式的字符串，或者接收达到数组上限，就进行字符串数据的处理
		if(g_usart1_recv_cnt > RECV_BUF_SIZE-1 || g_usart1_recv_buf[g_usart1_recv_cnt-1] == '\n')
		{
			//strncmp函数是用来比较两个字符串前N个字节是否相同，strcmp函数是比较两个字符串是否相同
			/*
				参数1、2：要比较的两个字符串地址
				参数3：要比较的两个字符串的前n个字节
				返回0，则表示字符串前N个字节相同
			*/
			/*
						printf,scanf函数是标准输出输入函数，他会自动从标准设备中进行数据的输出与输入（stdout,stdin）
						sprintf,sscanf函数他们的输出输入已经实现重定向，他会自动把数据从所指定的内存空间中进行输出与输入（buf）
						参数1：要输出/输入的数据内存空间
						参数2：要输出/输入的字符串格式，格式化符号%d：整型，%c：字符，%s：字符串
						参数3：要格式化输出/输入的变量
				*/
			
			
			if(strncmp(g_usart1_recv_buf, "V=\n ", 2) == 0)//磁感阈值修改命令
			{
				
				//sscanf(g_usart1_recv_buf, "V=%d\n", &Store_Data[80]);//磁感阈值修改
				//Store_Save();
				//sprintf(g_usart1_send_buf, "修改后Angle=%f\r\n", OdriveData.SetVel[0].float_temp);//修改后显示
			}
			
			if(strncmp(g_usart1_recv_buf, "S=\n ", 2) == 0)//磁感阈值修改命令
			{
				//Store_Save();
				//sprintf(g_usart1_send_buf, "修改后Angle=%f\r\n", OdriveData.SetVel[0].float_temp);//修改后显示
			}
			
			
			//清空字符串数组
			memset(g_usart1_recv_buf, 0, sizeof(g_usart1_recv_buf));
			
			//计数值清空
			g_usart1_recv_cnt = 0;
		}
	
	
}

