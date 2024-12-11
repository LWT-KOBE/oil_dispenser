#include "system.h"
#include "usart.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"

#define 	RECV_BUF_SIZE 	400

#define 	SEND_BUF_SIZE 	200
//����洢�����ַ���������
char g_usart1_recv_buf[RECV_BUF_SIZE] = {0};


//����洢�����ַ���������
char g_usart1_send_buf[SEND_BUF_SIZE] = {0};

//��������ַ��ļ���
int g_usart1_recv_cnt = 0;

//////////////////////////////////////////////////////////////////////////////////
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif
//////////////////////////////////////////////////////////////////////////////////
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32������
//����1��ʼ��
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/8/18
//�汾��V1.5
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved
//********************************************************************************
//V1.3�޸�˵��
//֧����Ӧ��ͬƵ���µĴ��ڲ���������.
//�����˶�printf��֧��
//�����˴��ڽ��������.
//������printf��һ���ַ���ʧ��bug
//V1.4�޸�˵��
//1,�޸Ĵ��ڳ�ʼ��IO��bug
//2,�޸���USART_RX_STA,ʹ�ô����������ֽ���Ϊ2��14�η�
//3,������USART_REC_LEN,���ڶ��崮�����������յ��ֽ���(������2��14�η�)
//4,�޸���EN_USART1_RX��ʹ�ܷ�ʽ
//V1.5�޸�˵��
//1,�����˶�UCOSII��֧��
//////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB
#if 1
#pragma import(__use_no_semihosting)
//��׼����Ҫ��֧�ֺ���
struct __FILE
{
    int handle;

};

FILE __stdout;
//����_sys_exit()�Ա���ʹ�ð�����ģʽ
void _sys_exit(int x)
{
    x = x;
}
//�ض���fputc����
int fputc(int ch, FILE *f)
{
    while((USART1->SR&0X40)==0);//ѭ������,ֱ���������
    USART1->DR = (u8) ch;
    return ch;
}
#endif

/*ʹ��microLib�ķ���*/
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

#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���
u8 	USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
u8  USART_TX_BUF[USART_REC_LEN];   //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���

void uart_init(u32 bound) {
    //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��

    //USART1_TX   GPIOA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9

    //USART1_RX	  GPIOA.10��ʼ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10

    //Usart1 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

    //USART ��ʼ������

    USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

    USART_Init(USART1, &USART_InitStructure); //��ʼ������1
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
    USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ���1

}

void USART1_IRQHandler(void)                	//����1�жϷ������
{
    u8 Res;
#if SYSTEM_SUPPORT_OS 		//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
    OSIntEnter();
#endif
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
    {
        Res =USART_ReceiveData(USART1);	//��ȡ���յ�������
		Serial1Data(Res);
        if((USART_RX_STA&0x8000)==0)//����δ���
        {
            if(USART_RX_STA&0x4000)//���յ���0x0d
            {
                if(Res!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
                else USART_RX_STA|=0x8000;	//���������
            }
            else //��û�յ�0X0D
            {
                if(Res==0x0d)USART_RX_STA|=0x4000;
                else
                {
                    USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
                    USART_RX_STA++;
                    if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����
                }
            }
        }
    }
#if SYSTEM_SUPPORT_OS 	//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
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
    i=strlen((const char*)USART_TX_BUF);//�˴η������ݵĳ���
    for(j=0; j<i; j++) //ѭ����������
    {
        while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);  //�ȴ��ϴδ������
        USART_SendData(USART1,(uint8_t)USART_TX_BUF[j]); 	 //�������ݵ�����1
    }
}


void u1_SendByte(uint8_t Byte)		//����һ���ֽ�����
{
	USART_SendData(USART1, Byte);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

void u1_SendArray(uint8_t *Array, uint16_t Length)	//����һ������
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		u1_SendByte(Array[i]);
	}
}

void u1_SendString(char *String)	//����һ���ַ�������
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

   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE); //ʹ��UART3����GPIOB��ʱ��
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //ʹ�ܴ��ڵ�RCCʱ��

   //����ʹ�õ�GPIO������
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//����USART3��RX�ӿ���PB11
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//�ӿ�ģʽ ��������
   GPIO_Init(GPIOB, &GPIO_InitStructure);

   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//����USART3��TX�ӿ���PB10
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//����ٶ�50MHz
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//�ӿ�ģʽ �����������
   GPIO_Init(GPIOB, &GPIO_InitStructure);

   //���ô���
   USART_InitStructure.USART_BaudRate = bound;
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
   USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
   USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

   USART_Init(USART3, &USART_InitStructure);//���ô���3
   USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//ʹ�ܴ��ڽ����ж�  
   //USART_ITConfig(USART3, USART_IT_TXE, ENABLE);//���ڷ����ж��ڷ�������ʱ����
   USART_Cmd(USART3, ENABLE);//ʹ�ܴ���3

   //�����ж�����
   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
   NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//����USART3�ж�
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//�жϵȼ�
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);                //ʹ�ܴ���1

}


void usart3_send(u8 data)
{
	USART3->DR = data;
	while((USART3->SR&0x40)==0);	
}

void u3_SendByte(uint8_t Byte)		//����һ���ֽ�����
{
	USART_SendData(USART3, Byte);
	while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
}

void u3_SendArray(uint8_t *Array, uint16_t Length)	//����һ������
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		u3_SendByte(Array[i]);
	}
}

void u3_SendString(char *String)	//����һ���ַ�������
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		usart3_send(String[i]);
	}
}

void USART3_IRQHandler(void)                	//����3�жϷ������
{
	u8 Res;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) //Check if data is received //�ж��Ƿ���յ�����
	{
		Res = USART_ReceiveData(USART3);	//��ȡ���յ�������
		if((USART_RX_STA&0x8000)==0)//����δ���
			{
			if(USART_RX_STA&0x4000)//���յ���0x0d
				{
				if(Res!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
				else USART_RX_STA|=0x8000;	//��������� 
				}
				else //��û�յ�0X0D
				{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
					{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
					}		 
				}
			}
		
		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	}
}

//����1���շ��ʹ�����
void Serial1Data(uint8_t ucData){
		
		//������յ��ַ����ַ�������Ķ�Ӧ��λ��
		g_usart1_recv_buf[g_usart1_recv_cnt] =ucData;
		//�����ַ��ļ���
		g_usart1_recv_cnt++;
		
	
			
	
		//�����յ����ϸ�ʽ���ַ��������߽��մﵽ�������ޣ��ͽ����ַ������ݵĴ���
		if(g_usart1_recv_cnt > RECV_BUF_SIZE-1 || g_usart1_recv_buf[g_usart1_recv_cnt-1] == '\n')
		{
			//strncmp�����������Ƚ������ַ���ǰN���ֽ��Ƿ���ͬ��strcmp�����ǱȽ������ַ����Ƿ���ͬ
			/*
				����1��2��Ҫ�Ƚϵ������ַ�����ַ
				����3��Ҫ�Ƚϵ������ַ�����ǰn���ֽ�
				����0�����ʾ�ַ���ǰN���ֽ���ͬ
			*/
			/*
						printf,scanf�����Ǳ�׼������뺯���������Զ��ӱ�׼�豸�н������ݵ���������루stdout,stdin��
						sprintf,sscanf�������ǵ���������Ѿ�ʵ���ض��������Զ������ݴ���ָ�����ڴ�ռ��н�����������루buf��
						����1��Ҫ���/����������ڴ�ռ�
						����2��Ҫ���/������ַ�����ʽ����ʽ������%d�����ͣ�%c���ַ���%s���ַ���
						����3��Ҫ��ʽ�����/����ı���
				*/
			
			
			if(strncmp(g_usart1_recv_buf, "V=\n ", 2) == 0)//�Ÿ���ֵ�޸�����
			{
				
				//sscanf(g_usart1_recv_buf, "V=%d\n", &Store_Data[80]);//�Ÿ���ֵ�޸�
				//Store_Save();
				//sprintf(g_usart1_send_buf, "�޸ĺ�Angle=%f\r\n", OdriveData.SetVel[0].float_temp);//�޸ĺ���ʾ
			}
			
			if(strncmp(g_usart1_recv_buf, "S=\n ", 2) == 0)//�Ÿ���ֵ�޸�����
			{
				//Store_Save();
				//sprintf(g_usart1_send_buf, "�޸ĺ�Angle=%f\r\n", OdriveData.SetVel[0].float_temp);//�޸ĺ���ʾ
			}
			
			
			//����ַ�������
			memset(g_usart1_recv_buf, 0, sizeof(g_usart1_recv_buf));
			
			//����ֵ���
			g_usart1_recv_cnt = 0;
		}
	
	
}

