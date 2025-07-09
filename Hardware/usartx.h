#ifndef __USRATX_H
#define __USRATX_H 

#include "stdio.h"
#include "sys.h"
#include "system.h"


#define FRAME_HEADER      0X7B //Frame_header //帧头
#define FRAME_TAIL        0X7D //Frame_tail   //帧尾
#define SEND_DATA_SIZE    24
#define RECEIVE_DATA_SIZE 11


/*******The structure of the serial port sending data************/
/*******串口发送数据的结构体*************************************/


void uart2_init(u32 bound);
void usart2_send(u8 data);


#endif

