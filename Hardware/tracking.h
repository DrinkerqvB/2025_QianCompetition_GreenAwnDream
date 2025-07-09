#ifndef __TRACKING_H
#define __TRACKING_H

#include "sys.h"
#include "system.h"


#define IR_Num 8 //探头数量

#define TRACKING_SLAVE_7BITADDRESS 0x12<<1
#define TRACKING_CALIBRATION_STATUS_REG 0x01  //校准状态寄存器，只写
#define TRACKING_TRACKING_DATA_REG 0x30    //循迹数据寄存器，只读


void Tracking_task(void);
void Tracking_Init(void);
void I2C1_Init(void);
void Tracking_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t Tracking_Read8bitRegister(uint8_t address);
void Tracking_SetCalibrationStatus(FunctionalState function);
uint8_t Tracking_ReadTrackingData(void);
void Tracking_HexToBin(uint8_t hexData);

float Calculate_Line_error(void);
uint8_t Is_Line_Lost(void);


#endif
