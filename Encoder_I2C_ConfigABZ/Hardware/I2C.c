#include "stm32f10x.h"                  // Device header

/**
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  I2C_EVENT: specifies the event to be checked. 
  *   This parameter can be one of the following values:
  *     @arg I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED           : EV1
  *     @arg I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED              : EV1
  *     @arg I2C_EVENT_SLAVE_TRANSMITTER_SECONDADDRESS_MATCHED     : EV1
  *     @arg I2C_EVENT_SLAVE_RECEIVER_SECONDADDRESS_MATCHED        : EV1
  *     @arg I2C_EVENT_SLAVE_GENERALCALLADDRESS_MATCHED            : EV1
  *     @arg I2C_EVENT_SLAVE_BYTE_RECEIVED                         : EV2
  *     @arg (I2C_EVENT_SLAVE_BYTE_RECEIVED | I2C_FLAG_DUALF)      : EV2
  *     @arg (I2C_EVENT_SLAVE_BYTE_RECEIVED | I2C_FLAG_GENCALL)    : EV2
  *     @arg I2C_EVENT_SLAVE_BYTE_TRANSMITTED                      : EV3
  *     @arg (I2C_EVENT_SLAVE_BYTE_TRANSMITTED | I2C_FLAG_DUALF)   : EV3
  *     @arg (I2C_EVENT_SLAVE_BYTE_TRANSMITTED | I2C_FLAG_GENCALL) : EV3
  *     @arg I2C_EVENT_SLAVE_ACK_FAILURE                           : EV3_2
  *     @arg I2C_EVENT_SLAVE_STOP_DETECTED                         : EV4
  *     @arg I2C_EVENT_MASTER_MODE_SELECT                          : EV5
  *     @arg I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED            : EV6     
  *     @arg I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED               : EV6
  *     @arg I2C_EVENT_MASTER_BYTE_RECEIVED                        : EV7
  *     @arg I2C_EVENT_MASTER_BYTE_TRANSMITTING                    : EV8
  *     @arg I2C_EVENT_MASTER_BYTE_TRANSMITTED                     : EV8_2
  *     @arg I2C_EVENT_MASTER_MODE_ADDRESS10                       : EV9
  *     
  * @note: For detailed description of Events, please refer to section 
  *    I2C_Events in stm32f10x_i2c.h file.
  *  
  */
uint8_t MyI2C_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
	for(int timer=0; timer<10000;timer++){
		if(I2C_CheckEvent(I2Cx,I2C_EVENT)==SUCCESS){
			return 1;//事件通过
		}
	}
	return 0;//超时
}



void MyI2C_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	I2C_InitTypeDef I2C_InitStructure;
	I2C_InitStructure.I2C_Ack=I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress=I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed=400000;
	I2C_InitStructure.I2C_DutyCycle=I2C_DutyCycle_2;
	I2C_InitStructure.I2C_Mode=I2C_Mode_I2C;
	I2C_InitStructure.I2C_OwnAddress1=0x00;
	I2C_Init(I2C2,&I2C_InitStructure);
	
	I2C_Cmd(I2C2,ENABLE);
}

/*
void MyI2C_SendData(uint8_t SlaveAddress, uint8_t MemoryAddress, uint8_t dataNum, uint8_t* data)
{
	I2C_GenerateSTART(I2C2,ENABLE);
	MyI2C_WaitEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C2,SlaveAddress,I2C_Direction_Transmitter);
	MyI2C_WaitEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTING);
	
	I2C_SendData(I2C2, MemoryAddress);
	MyI2C_WaitEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTING);
	
	for(uint8_t i=0;i<dataNum-1; i++){
		I2C_SendData(I2C2, data[i]);
		MyI2C_WaitEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTING);
	}
	I2C_SendData(I2C2, data[dataNum-1]);
	MyI2C_WaitEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED);

	I2C_GenerateSTOP(I2C2,ENABLE);
}
*/

void MyI2C_SendData(uint8_t SlaveAddress, uint8_t MemoryAddress,uint8_t data)
{
	I2C_GenerateSTART(I2C2,ENABLE);
	MyI2C_WaitEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C2,SlaveAddress,I2C_Direction_Transmitter);
	MyI2C_WaitEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTING);
	
	I2C_SendData(I2C2, MemoryAddress);
	MyI2C_WaitEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTING);
	
	I2C_SendData(I2C2, data);
	MyI2C_WaitEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED);

	I2C_GenerateSTOP(I2C2,ENABLE);
}

uint8_t MyI2C_ReceiveData(uint8_t SlaveAddress, uint8_t MemoryAddress)
{
	uint8_t Data;
	//0000110
	I2C_GenerateSTART(I2C2, ENABLE);										//硬件I2C生成起始条件
	MyI2C_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);					//等待EV5
	
	I2C_Send7bitAddress(I2C2, SlaveAddress, I2C_Direction_Transmitter);	//硬件I2C发送从机地址，方向为发送
	MyI2C_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);	//等待EV6
	
	I2C_SendData(I2C2, MemoryAddress);											//硬件I2C发送寄存器地址
	MyI2C_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);				//等待EV8_2
	
	I2C_GenerateSTART(I2C2, ENABLE);										//硬件I2C生成重复起始条件
	MyI2C_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);					//等待EV5
	
	I2C_Send7bitAddress(I2C2, SlaveAddress, I2C_Direction_Receiver);		//硬件I2C发送从机地址，方向为接收
	MyI2C_WaitEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);		//等待EV6
	
	I2C_AcknowledgeConfig(I2C2, DISABLE);									//在接收最后一个字节之前提前将应答失能
	I2C_GenerateSTOP(I2C2, ENABLE);											//在接收最后一个字节之前提前申请停止条件
	
	MyI2C_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);				//等待EV7
	Data = I2C_ReceiveData(I2C2);											//接收数据寄存器
	
	I2C_AcknowledgeConfig(I2C2, ENABLE);									//将应答恢复为使能，为了不影响后续可能产生的读取多字节操作
	
	return Data;
}
