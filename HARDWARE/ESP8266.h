#ifndef __ESP8266_H
#define __ESP8266_H
#include "system.h"
#define ESP8266_STK_SIZE   512 
#define ESP8266_TASK_PRIO  4

#define ESP8266_RECEIVE_LENGTH 2000

void ESP8266_Init(void);
void ESP8266_Command(char* Command_AT);
void USART3_SendChar(uint8_t ch);
void USART3_SendString(char* str);
uint8_t USART3_ReceiveChar(void);
void uart3_init(u32 bound);
void ESP8266_RxCpltCallback(uint8_t data);

#endif
