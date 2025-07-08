#ifndef __TRACKING_H
#define __TRACKING_H

#include "sys.h"
#include "system.h"

#define Package_size 100
#define IR_Num 8 //̽ͷ����
#define TRACKING_STK_SIZE  256 //�����ջ��С
#define TRACKING_TASK_PRIO  3 //�������ȼ�

#define TRACKING_SLAVE_7BITADDRESS 0x12<<1
#define TRACKING_CALIBRATION_STATUS_REG 0x01  //У׼״̬�Ĵ�����ֻд
#define TRACKING_TRACKING_DATA_REG 0x30    //ѭ�����ݼĴ�����ֻ��

// ֻ����������
extern const float IR_Weights[8]; 
extern u8 IR_Data_number[IR_Num];
extern u8 g_new_package_flag;


void Tracking_task(void);
void Tracking_Init(void);
void I2C1_Init(void);
void Tracking_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t Tracking_Read8bitRegister(uint8_t address);
void Tracking_SetCalibrationStatus(FunctionalState function);
uint8_t Tracking_ReadTrackingData(void);
void Tracking_HexToBin(uint8_t hexData);
//void USART5_SEND(uint8_t *BufferPtr, uint16_t Length);
//void usart5_send(u8 data);
//void uart5_init(u32 bound);
////int UART5_IRQHandler(void);
//void Deal_IR_Usart(u8 rxtemp);
//void Deal_Usart_Data(void);
float Calculate_Line_error(void);
uint8_t Is_Line_Lost(void);
//u8 Check_Sum(unsigned char Count_Number,unsigned char Mode);

#endif
