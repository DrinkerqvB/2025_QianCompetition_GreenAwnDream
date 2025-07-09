#include "Timer.h"


void TIM7_Init(void)
{

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; 
	NVIC_InitTypeDef NVIC_InitStructure;
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);   //使能定时器

  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	
	TIM_InternalClockConfig(TIM7);
  
	//设置时基单元，使得每计时1ms后溢出
  TIM_TimeBaseStructure.TIM_Prescaler = 84-1; 							// No prescaling     //不分频
  TIM_TimeBaseStructure.TIM_Period = 1000000/SWITCHFREQ-1;  //设定计数器自动重装值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //选择时钟分频：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数    
  TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);  //初始化定时器
  

  TIM_ClearFlag(TIM7, TIM_FLAG_Update);//清除TIM的更新标志位
  TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
	NVIC_Init(&NVIC_InitStructure);
	
  TIM_SetCounter(TIM7,0);
  TIM_Cmd(TIM7, ENABLE); 

}


