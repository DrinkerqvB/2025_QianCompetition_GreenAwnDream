#ifndef __TRACKING_H
#define __TRACKING_H

#include "sys.h"
#include "system.h"

#define Package_size 100
#define IR_Num 8 //̽ͷ����
#define TRACKING_STK_SIZE  256 //�����ջ��С
#define TRACKING_TASK_PRIO  3 //�������ȼ�
// ֻ����������
extern const float IR_Weights[8]; 
extern u8 IR_Data_number[IR_Num];
extern u8 g_new_package_flag;


void Tracking_task();
void Tracking_Init(void);
void USART5_SEND(uint8_t *BufferPtr, uint16_t Length);
void usart5_send(u8 data);
void uart5_init(u32 bound);
//int UART5_IRQHandler(void);
void Deal_IR_Usart(u8 rxtemp);
void Deal_Usart_Data(void);
float Calculate_Line_error(void);
uint8_t Is_Line_Lost(void);
//u8 Check_Sum(unsigned char Count_Number,unsigned char Mode);

#endif
