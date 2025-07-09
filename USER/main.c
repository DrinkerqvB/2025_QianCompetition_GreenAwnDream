#include "system.h"

int main()
{
	systemInit();
	
	while(1){
		Tracking_task();
		Balance_task();
		ESP8266_task();
	}
}



//放置定时执行函数
void TIM7_IRQHandler(void)
{
	//static uint16_t count=0;
	if(TIM_GetITStatus(TIM7,TIM_IT_Update)==SET){
		//count++;
		
		//if(count%1==0){
			Get_Velocity_Form_Encoder();
			FOCLoop_task();
		//}
		
		//if(count>=1000){count = 0;}
		TIM_ClearITPendingBit(TIM7,TIM_IT_Update);
	}

}
