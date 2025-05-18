#ifndef __TRACKING_H
#define __TRACKING_H

#include "sys.h"
#include "system.h"

#define Package_size 100
#define IR_Num 8 //探头数量
#define TRACKING_STK_SIZE  256 //任务堆栈大小
#define TRACKING_TASK_PRIO  3 //任务优先级


void Tracking_task(void *pvParameters);
void Tracking_Init(void);
void USART5_SEND(uint8_t *BufferPtr, uint16_t Length);
void usart5_send(u8 data);
void uart5_init(u32 bound);
//int UART5_IRQHandler(void);
void Deal_IR_Usart(u8 rxtemp);
void Deal_Usart_Data(void);
//u8 Check_Sum(unsigned char Count_Number,unsigned char Mode);

#endif
