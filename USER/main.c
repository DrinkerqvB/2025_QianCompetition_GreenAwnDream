#include "system.h"

FlagStatus  FunctionFlag_1Hz    =RESET;  //每1000ms 执行一次的函数，就绪标志位
FlagStatus  FunctionFlag_2Hz    =RESET;  //每 500ms 执行一次的函数，就绪标志位
FlagStatus  FunctionFlag_5Hz    =RESET;  //每 200ms 执行一次的函数，就绪标志位
FlagStatus  FunctionFlag_10Hz   =RESET;  //每 100ms 执行一次的函数，就绪标志位
FlagStatus  FunctionFlag_20Hz   =RESET;  //每  50ms 执行一次的函数，就绪标志位
FlagStatus  FunctionFlag_50Hz   =RESET;  //每  20ms 执行一次的函数，就绪标志位
FlagStatus  FunctionFlag_100Hz  =RESET;  //每  10ms 执行一次的函数，就绪标志位
FlagStatus  FunctionFlag_200Hz  =RESET;  //每   5ms 执行一次的函数，就绪标志位
FlagStatus  FunctionFlag_500Hz  =RESET;  //每   2ms 执行一次的函数，就绪标志位
FlagStatus  FunctionFlag_1000Hz =RESET;  //每   1ms 执行一次的函数，就绪标志位

void Timed_task(void);

int main()
{
	systemInit();
	
	while(1){
		Tracking_task();
		Balance_task();
		ESP8266_task();
		Timed_task();
	}
}

void Timed_task(void){
	//每1ms执行一次
	if(FunctionFlag_1000Hz==SET){
		
		Get_Velocity_Form_Encoder();
		FOCLoop_task();
		
		FunctionFlag_1000Hz=RESET;
	}
	//每2ms执行一次
	if(FunctionFlag_500Hz==SET){
		
		
		FunctionFlag_500Hz=RESET;
	}
	//每5ms执行一次
	if(FunctionFlag_200Hz==SET){
		
		
		FunctionFlag_200Hz=RESET;
	}
	//每10ms执行一次
	if(FunctionFlag_100Hz==SET){
		
		
		FunctionFlag_100Hz=RESET;
	}
	//每20ms执行一次
	if(FunctionFlag_50Hz==SET){
		
		
		FunctionFlag_50Hz=RESET;
	}
	//每50ms执行一次
	if(FunctionFlag_20Hz==SET){
		
		PowerMeter_RequestData();
		
		FunctionFlag_20Hz=RESET;
		
	}
	//每100ms执行一次
	if(FunctionFlag_10Hz==SET){
		
		
		FunctionFlag_10Hz=RESET;
	}
	//每200ms执行一次
	if(FunctionFlag_5Hz==SET){
		
		
		FunctionFlag_5Hz=RESET;
	}
	
	//每500ms执行一次
	if(FunctionFlag_2Hz==SET){
		
		
		FunctionFlag_2Hz=RESET;
	}
	//每1000ms执行一次
	if(FunctionFlag_1Hz==SET){
		
		
		FunctionFlag_1Hz=RESET;
	}
}




//设置定时执行函数就绪标志位
void TIM7_IRQHandler(void)
{
	static uint16_t count=0;
	if(TIM_GetITStatus(TIM7,TIM_IT_Update)==SET){
		count++;
		
		//每1ms执行一次
		if(count%1==0){
			FunctionFlag_1000Hz=SET;
		}
		//每2ms执行一次
		if(count%2==0){
			FunctionFlag_500Hz=SET;
		}
		//每5ms执行一次
		if(count%5==0){
			FunctionFlag_200Hz=SET;
		}
		//每10ms执行一次
		if(count%10==0){
			FunctionFlag_100Hz=SET;
		}
		//每20ms执行一次
		if(count%20==0){
			FunctionFlag_50Hz=SET;
		}
		//每50ms执行一次
		if(count%50==0){
			FunctionFlag_20Hz=SET;
		}
		//每100ms执行一次
		if(count%100==0){
			FunctionFlag_10Hz=SET;
		}
		//每200ms执行一次
		if(count%200==0){
			FunctionFlag_5Hz=SET;
		}
		
		//每500ms执行一次
		if(count%500==0){
			FunctionFlag_2Hz=SET;
		}
		//每1000ms执行一次
		if(count%1000==0){
			FunctionFlag_1Hz=SET;
		}
		
		
		//计数器复位
		if(count>=1000){count = 0;}
		
		TIM_ClearITPendingBit(TIM7,TIM_IT_Update);
	}

}
