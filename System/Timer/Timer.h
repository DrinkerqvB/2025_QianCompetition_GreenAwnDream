#ifndef __TIMER_H
#define __TIMER_H
#include "system.h"
#define SWITCHFREQ  1000  //HZ（1ms触发一次中断）!!!!!不可随意修改，否则会影响定时执行函数的功能!!!!!

void TIM7_Init(void);


#endif
