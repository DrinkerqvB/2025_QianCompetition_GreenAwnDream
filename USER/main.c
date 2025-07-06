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
