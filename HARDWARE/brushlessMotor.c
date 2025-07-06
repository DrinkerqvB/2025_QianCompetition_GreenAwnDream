#include "brushlessMotor.h"
#include <math.h>

PID_HandleTypeDef pid_current_q, pid_current_d;


uint8_t Rotor_Angle;









/////////////////////////////////////////////////////


// 输入：Ualpha, Ubeta, 当前扇区
// 输出：T1, T2（有效矢量时间），T0（零矢量时间）
void Calculate_Time(float Ualpha, float Ubeta, uint8_t sector, float *T1, float *T2, float *T0)
{
    float X = Ubeta;
    float Y = 0.5f * (sqrt(3.0f)) * Ualpha - 0.5f * Ubeta;
    float Z = -0.5f * (sqrt(3.0f) * Ualpha) - 0.5f * Ubeta;

    switch (sector) {
        case 1: *T1 = -Z; *T2 = X;  break;
        case 2: *T1 = Y;  *T2 = -Z; break;
        case 3: *T1 = X;  *T2 = Y;  break;
        case 4: *T1 = -X; *T2 = Z;  break;
        case 5: *T1 = -Y; *T2 = -X; break;
        case 0: *T1 = Z;  *T2 = -Y; break;
    }
    *T0 = 1 - (*T1 + *T2); // 零矢量时间
}

// 输入：T1, T2, T0, 当前扇区
// 输出：三相PWM占空比（0~PWM_PERIOD）
void Calculate_Duty(uint8_t sector, float T1, float T2, float T0, uint16_t *dutyU, uint16_t *dutyV, uint16_t *dutyW)
{
    float Ta, Tb, Tc;

    switch (sector) {
        case 1: Ta = T1 + T2 + T0/2; Tb = T2 + T0/2; Tc = T0/2; break;
        case 2: Ta = T1 + T0/2;      Tb = T1 + T2 + T0/2; Tc = T0/2; break;
        case 3: Ta = T0/2;           Tb = T1 + T2 + T0/2; Tc = T2 + T0/2; break;
        case 4: Ta = T0/2;           Tb = T1 + T0/2;      Tc = T1 + T2 + T0/2; break;
        case 5: Ta = T2 + T0/2;      Tb = T0/2;           Tc = T1 + T2 + T0/2; break;
        case 0: Ta = T1 + T2 + T0/2; Tb = T0/2;           Tc = T1 + T0/2; break;
    }

    *dutyU = (uint16_t)(Ta * PWM_PERIOD);
    *dutyV = (uint16_t)(Tb * PWM_PERIOD);
    *dutyW = (uint16_t)(Tc * PWM_PERIOD);
}

//void Motor_SetDutyCycle(BrushlessMotor Motor,uint8_t DutyCycle)
//{
//	if(Motor==Motor_Left){
//		
//	}
//}

/**************************************************************************
函数功能：通过编码器来读取电角度
入口参数：LeftWheel 或 RightWheel
返回  值：无 
**************************************************************************/
void  Motor_GetRotorAngle(void)
{
	uint16_t angle;
	
	
	angle=PolePairs*360*TIM_GetCounter(TIM2)/ABZ_Resolution;
	angle=angle%360;
	Motor_Left.angle=target_limit_int(angle,0,359);

	angle=PolePairs*360*TIM_GetCounter(TIM3)/ABZ_Resolution;
	angle=angle%360;
	Motor_Right.angle=target_limit_int(angle,0,359);
	
	
}

/**************************************************************************
函数功能：通过转子角度来判断转子所在扇区
入口参数：转子角度
返回  值：1~6的整数
**************************************************************************/
void Motor_JudgeRotorRegion(void)
{
	Motor_Left.RotorRegion=target_limit_int(Motor_Left.angle/60+1,1,6);
	Motor_Right.RotorRegion=target_limit_int(Motor_Right.angle/60+1,1,6);
}

/**************************************************************************
函数功能：TIM4初始化
入口参数：无
返回  值：无 
**************************************************************************/
void TIM4_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 1. 使能时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    
    // 2. 配置GPIO: 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 3. 引脚复用
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
    
    TIM_InternalClockConfig(TIM4);
	
    // 4. 定时器时基配置
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0000;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
    TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    
    // 5. PWM输出配置
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    
    // 通道1-3配置
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
    
    
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
    
    
    // 6. 使能定时器
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_CtrlPWMOutputs(TIM4, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}



/**************************************************************************
函数功能：TIM8初始化
入口参数：无
返回  值：无 
**************************************************************************/
void TIM8_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 1. 使能时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    
    // 2. 配置GPIO: 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    // 3. 引脚复用
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM8);
	
	TIM_InternalClockConfig(TIM8);
    
    // 4. 定时器时基配置
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0000;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
    TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
    
    // 5. PWM输出配置
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
	TIM_BDTRStructInit(&TIM_BDTRInitStructure);
	TIM_BDTRInitStructure.TIM_DeadTime=0x1F;
	TIM_BDTRInitStructure.TIM_OSSRState=TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState=TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel=TIM_LOCKLevel_1;
	TIM_BDTRInitStructure.TIM_Break=TIM_Break_Disable;
	TIM_BDTRConfig(TIM8,&TIM_BDTRInitStructure);
	
	
    
    // 通道1-3配置
    TIM_OC1Init(TIM8, &TIM_OCInitStructure);
    TIM_OC2Init(TIM8, &TIM_OCInitStructure);
    TIM_OC3Init(TIM8, &TIM_OCInitStructure);
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);
    
    TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);
    
    // 6. 使能定时器
    TIM_ARRPreloadConfig(TIM8, ENABLE);
    TIM_CtrlPWMOutputs(TIM8, ENABLE);
    TIM_Cmd(TIM8, ENABLE);
	
}

