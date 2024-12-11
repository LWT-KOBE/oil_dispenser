#include "ax_motor.h" 

/**
  * @��  ��  ���PWM���Ƴ�ʼ����PWMƵ��Ϊ20KHZ	
  * @��  ��  ��
  * @����ֵ  ��
  */
void AX_MOTOR_Init(void)
{ 
	
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure; 

	//GPIO����ʱ��ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA, ENABLE);

	//����IO��Ϊ���ù���-��ʱ��ͨ��
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�100MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�100MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//TIMʱ��ʹ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	//Time base configuration
	TIM_TimeBaseStructure.TIM_Period = 3600-1;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	//PWM1 Mode configuration: Channel1 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;	    //ռ�ձȳ�ʼ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	
	//PWM1 Mode configuration: Channel1
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	//PWM1 Mode configuration: Channel2
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	//PWM1 Mode configuration: Channel3
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	//PWM1 Mode configuration: Channel4
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM3, ENABLE);

	//TIM enable counter
	TIM_Cmd(TIM3, ENABLE);   

	//ʹ��MOEλ
	TIM_CtrlPWMOutputs(TIM3,ENABLE);	
}

/**
  * @��  �� ���PWM�ٶȿ���
  * @��  �� speed ���ת����ֵ����Χ-3600~3600
  * @����ֵ ��
  */
void AX_MOTOR_A_SetSpeed(int16_t speed)
{
	int16_t temp;
	
	temp = speed;
	
	if(temp>3600)
		temp = 3600;
	if(temp<-3600)
		temp = -3600;
	
	if(temp > 0)
	{
		TIM_SetCompare1(TIM3, 3600);
		TIM_SetCompare2(TIM3, (3600 - temp));
	}
	else
	{
		TIM_SetCompare2(TIM3, 3600);
		TIM_SetCompare1(TIM3, (3600 + temp));
	}
}

/**
  * @��  �� ���PWM�ٶȿ���
  * @��  �� speed ���ת����ֵ����Χ-3600~3600
  * @����ֵ ��
  */
void AX_MOTOR_B_SetSpeed(int16_t speed)
{
	int16_t temp;
	
	temp = speed;
	
	if(temp>3600)
		temp = 3600;
	if(temp<-3600)
		temp = -3600;
	
	if(temp > 0)
	{
		TIM_SetCompare3(TIM3, 3600);
		TIM_SetCompare4(TIM3, (3600 - temp));
	}
	else
	{
		TIM_SetCompare4(TIM3, 3600);
		TIM_SetCompare3(TIM3, (3600 + temp));
	}
}

