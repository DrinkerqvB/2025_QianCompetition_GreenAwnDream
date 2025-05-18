#ifndef __I2C_H
#define __I2C_H

void MyI2C_Init(void);
void MyI2C_SendData(uint8_t SlaveAddress, uint8_t MemoryAddress,uint8_t data);
uint8_t MyI2C_ReceiveData(uint8_t SlaveAddress, uint8_t MemoryAddress);

#endif
