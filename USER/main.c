#include "system.h"

FlagStatus  FunctionFlag_1Hz    =RESET;  //ÿ1000ms ִ��һ�εĺ�����������־λ
FlagStatus  FunctionFlag_2Hz    =RESET;  //ÿ 500ms ִ��һ�εĺ�����������־λ
FlagStatus  FunctionFlag_5Hz    =RESET;  //ÿ 200ms ִ��һ�εĺ�����������־λ
FlagStatus  FunctionFlag_10Hz   =RESET;  //ÿ 100ms ִ��һ�εĺ�����������־λ
FlagStatus  FunctionFlag_20Hz   =RESET;  //ÿ  50ms ִ��һ�εĺ�����������־λ
FlagStatus  FunctionFlag_50Hz   =RESET;  //ÿ  20ms ִ��һ�εĺ�����������־λ
FlagStatus  FunctionFlag_100Hz  =RESET;  //ÿ  10ms ִ��һ�εĺ�����������־λ
FlagStatus  FunctionFlag_200Hz  =RESET;  //ÿ   5ms ִ��һ�εĺ�����������־λ
FlagStatus  FunctionFlag_500Hz  =RESET;  //ÿ   2ms ִ��һ�εĺ�����������־λ
FlagStatus  FunctionFlag_1000Hz =RESET;  //ÿ   1ms ִ��һ�εĺ�����������־λ

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
	//ÿ1msִ��һ��
	if(FunctionFlag_1000Hz==SET){
		
		Get_Velocity_Form_Encoder();
		FOCLoop_task();
		
		FunctionFlag_1000Hz=RESET;
	}
	//ÿ2msִ��һ��
	if(FunctionFlag_500Hz==SET){
		
		
		FunctionFlag_500Hz=RESET;
	}
	//ÿ5msִ��һ��
	if(FunctionFlag_200Hz==SET){
		
		
		FunctionFlag_200Hz=RESET;
	}
	//ÿ10msִ��һ��
	if(FunctionFlag_100Hz==SET){
		
		
		FunctionFlag_100Hz=RESET;
	}
	//ÿ20msִ��һ��
	if(FunctionFlag_50Hz==SET){
		
		
		FunctionFlag_50Hz=RESET;
	}
	//ÿ50msִ��һ��
	if(FunctionFlag_20Hz==SET){
		
		PowerMeter_RequestData();
		
		FunctionFlag_20Hz=RESET;
		
	}
	//ÿ100msִ��һ��
	if(FunctionFlag_10Hz==SET){
		
		
		FunctionFlag_10Hz=RESET;
	}
	//ÿ200msִ��һ��
	if(FunctionFlag_5Hz==SET){
		
		
		FunctionFlag_5Hz=RESET;
	}
	
	//ÿ500msִ��һ��
	if(FunctionFlag_2Hz==SET){
		
		
		FunctionFlag_2Hz=RESET;
	}
	//ÿ1000msִ��һ��
	if(FunctionFlag_1Hz==SET){
		
		
		FunctionFlag_1Hz=RESET;
	}
}




//���ö�ʱִ�к���������־λ
void TIM7_IRQHandler(void)
{
	static uint16_t count=0;
	if(TIM_GetITStatus(TIM7,TIM_IT_Update)==SET){
		count++;
		
		//ÿ1msִ��һ��
		if(count%1==0){
			FunctionFlag_1000Hz=SET;
		}
		//ÿ2msִ��һ��
		if(count%2==0){
			FunctionFlag_500Hz=SET;
		}
		//ÿ5msִ��һ��
		if(count%5==0){
			FunctionFlag_200Hz=SET;
		}
		//ÿ10msִ��һ��
		if(count%10==0){
			FunctionFlag_100Hz=SET;
		}
		//ÿ20msִ��һ��
		if(count%20==0){
			FunctionFlag_50Hz=SET;
		}
		//ÿ50msִ��һ��
		if(count%50==0){
			FunctionFlag_20Hz=SET;
		}
		//ÿ100msִ��һ��
		if(count%100==0){
			FunctionFlag_10Hz=SET;
		}
		//ÿ200msִ��һ��
		if(count%200==0){
			FunctionFlag_5Hz=SET;
		}
		
		//ÿ500msִ��һ��
		if(count%500==0){
			FunctionFlag_2Hz=SET;
		}
		//ÿ1000msִ��һ��
		if(count%1000==0){
			FunctionFlag_1Hz=SET;
		}
		
		
		//��������λ
		if(count>=1000){count = 0;}
		
		TIM_ClearITPendingBit(TIM7,TIM_IT_Update);
	}

}
