#include "delay.h"

/*由于使用了FreeRtos了，滴答定时器不能使用，先改为基本定时器1(APB2)来延时*/


void delay_ms(uint16_t num)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE); // 启用 TIM1 时钟
    
    TIM_TimeBaseInitTypeDef Delay; // 定义定时器基础配置结构体
    Delay.TIM_ClockDivision = TIM_CKD_DIV1; // 时钟分频器设置
    Delay.TIM_CounterMode = TIM_CounterMode_Up; // 计数模式设置为向上计数
    Delay.TIM_Period = 0xfffe;  // 设置自动重装载寄存器的值  确保计数器的值大于 1ms。
    Delay.TIM_Prescaler = 21000-1; // 设置预分频器
    TIM_TimeBaseInit(TIM1 , &Delay); // 初始化 TIM1
    
    TIM_Cmd(TIM1 , ENABLE); // 启动定时器
    
    TIM_SetCounter(TIM1 , 0); // 清空计数值
    while(TIM_GetCounter(TIM1) < (8*num)); // 等待计数器达到指定值
}

void delay_us(uint16_t num)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE); // 启用 TIM1 时钟
    
    TIM_TimeBaseInitTypeDef Delay; // 定义定时器基础配置结构体
    Delay.TIM_ClockDivision = TIM_CKD_DIV1; // 时钟分频器设置
    Delay.TIM_CounterMode = TIM_CounterMode_Up; // 计数模式设置为向上计数
    Delay.TIM_Period = 0xfffe;  // 设置自动重装载寄存器的值  确保计数器的值大于 1ms。
    Delay.TIM_Prescaler = 168-1; // 设置预分频器
    TIM_TimeBaseInit(TIM1 , &Delay); // 初始化 TIM1
    
    TIM_Cmd(TIM1 , ENABLE); // 启动定时器
    
    TIM_SetCounter(TIM1 , 0); // 清空计数值
    while(TIM_GetCounter(TIM1) < num); // 等待计数器达到指定值
}
