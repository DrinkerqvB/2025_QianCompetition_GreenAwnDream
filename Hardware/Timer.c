#include "Timer.h"


void TIM7_Init(void)
{

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; 
	NVIC_InitTypeDef NVIC_InitStructure;
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);   //ʹ�ܶ�ʱ��

  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	
	TIM_InternalClockConfig(TIM7);
  
	//����ʱ����Ԫ��ʹ��ÿ��ʱ1ms�����
  TIM_TimeBaseStructure.TIM_Prescaler = 84-1; 							// No prescaling     //����Ƶ
  TIM_TimeBaseStructure.TIM_Period = 1000000/SWITCHFREQ-1;  //�趨�������Զ���װֵ
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //ѡ��ʱ�ӷ�Ƶ������Ƶ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM���ϼ���    
  TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);  //��ʼ����ʱ��
  

  TIM_ClearFlag(TIM7, TIM_FLAG_Update);//���TIM�ĸ��±�־λ
  TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
	NVIC_Init(&NVIC_InitStructure);
	
  TIM_SetCounter(TIM7,0);
  TIM_Cmd(TIM7, ENABLE); 

}


