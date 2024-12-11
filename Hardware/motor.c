#include "Delay.h"
#include "motor.h"

/**
  * 函    数：按键初始化
  * 参    数：无
  * 返 回 值：无
  */
void Motor_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);		//开启GPIOB的时钟
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*设置GPIO初始化后的默认电平*/
	GPIO_SetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7);
	
//	/********************************************GPIOA******************************************************/
//	/*开启时钟*/
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);		//开启GPIOA的时钟
//	
//	
//	
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_6 | GPIO_Pin_7;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);

//	/*设置GPIO初始化后的默认电平*/
//	GPIO_SetBits(GPIOA,GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_0 | GPIO_Pin_1);
//	
//	/********************************************GPIOA******************************************************/
//	/*开启时钟*/
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);		//开启GPIOA的时钟
//	
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4 | GPIO_Pin_5;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);

//	/*设置GPIO初始化后的默认电平*/
//	GPIO_SetBits(GPIOB,GPIO_Pin_4 | GPIO_Pin_5);
	
	
}


