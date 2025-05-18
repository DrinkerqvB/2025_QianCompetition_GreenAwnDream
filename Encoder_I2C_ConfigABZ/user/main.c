#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "I2C.h"

uint8_t MT6701_data_0x30;
uint8_t MT6701_data_0x31;

int main()
{
	MyI2C_Init();
	OLED_Init();
	
	for(;;){
		MT6701_data_0x30=MyI2C_ReceiveData(0x06,0x30);
		MT6701_data_0x31=MyI2C_ReceiveData(0x06,0x31);
		
		OLED_ShowBinNum(1,1,MT6701_data_0x30,8);
		OLED_ShowBinNum(2,1,MT6701_data_0x31,8);
	}
}
