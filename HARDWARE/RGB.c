#include "RGB.h"

extern Modbus_Typedef Modbus_Type;

void RGB_Init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed=GPIO_Fast_Speed;
	
	GPIO_Init(GPIOE,&GPIO_InitStructure);
}


void RGB_DeInit(void)
{
	GPIO_ResetBits(GPIOE,GPIO_Pin_2);
	GPIO_ResetBits(GPIOE,GPIO_Pin_3);
	GPIO_ResetBits(GPIOE,GPIO_Pin_4);
}
	

void RGB_SelectiveLight(Modbus_Typedef type)
{
	switch(type){
		case Modbus_Type_A:
			RGB_DeInit();
			GPIO_SetBits(GPIOE,GPIO_Pin_3);
			break;
		case Modbus_Type_B:
			RGB_DeInit();
			GPIO_SetBits(GPIOE,GPIO_Pin_4);
			break;
		default:
			RGB_DeInit();
			break;
	
	}
}
	
