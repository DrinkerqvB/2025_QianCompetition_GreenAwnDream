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
	static uint16_t count=0;
	if(TIM_GetITStatus(TIM7,TIM_IT_Update)==SET){
		count++;
		
		//每1ms执行一次
		if(count%1==0){
			Get_Velocity_Form_Encoder();
			FOCLoop_task();
		}
		
		//每50ms执行一次
		if(count%50==0){
			PowerMeter_RequestData();
		}
		
		
		//计数器复位
		if(count>=1000){count = 0;}
		
		TIM_ClearITPendingBit(TIM7,TIM_IT_Update);
	}

}
