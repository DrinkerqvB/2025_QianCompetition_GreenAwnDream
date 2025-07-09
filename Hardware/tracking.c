#include "tracking.h"
#include "string.h"
const float IR_Weights[8] = {-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5};//Ȩ�ؾ���

u8 IR_Data_number[IR_Num];//���������ݣ����յõ���̽�����ݣ���0λ��7λΪx1-x8
extern ControlState cmd;//�洢ƫ�ǰ���ٶȡ�ת����ٶ�
extern FlagStatus Car_StopCmd;
float MoveX_mps; //ǰ���ٶ� m/s
float error; //ƫ���IR_Data_number��IR_Weights���ڻ��ó�

/**************************************************************************
�������ܣ����񣬰�·ѭ�����ݴ���
��ڲ�������
����  ֵ����
**************************************************************************/
void Tracking_task(void)
{ 
	uint8_t data=0;
	data=Tracking_ReadTrackingData();
	Tracking_HexToBin(data);
		
		error =  Calculate_Line_error();
	//printf("%d,%d,%d,%d,%d,%d,%d,%d,%f\n",IR_Data_number[0],IR_Data_number[1],IR_Data_number[2],IR_Data_number[3],IR_Data_number[4],IR_Data_number[5],IR_Data_number[6],IR_Data_number[7],error);
			
		
		cmd.line_error = error;
	if(Car_StopCmd==RESET){
		cmd.speed_setpoint = MoveX_mps;      // �̶��ٶ�0.5m/s
		cmd.turn_gain = 20.0f*error; // KP=20
	}else{
		cmd.speed_setpoint = 0;      // �̶��ٶ�0m/s
		cmd.turn_gain = 0; // KP=0
	}
}

/**************************************************************************
�������ܣ����Ĵ�������ת��Ϊ����
��ڲ�������
����  ֵ����
**************************************************************************/
void Tracking_HexToBin(uint8_t hexData)
{
	uint8_t binData[8]={0};
	for(uint8_t i=0;i<8;i++){
		binData[i] = (hexData >> (7-i)) & 0x01;
	}
	memcpy(IR_Data_number,binData,8);
}

/*****************************************************************************************************
Functions: Calculate 
Input :data of IR_Data_number none 
Output: ƫ��ֵ
******************************************************************************************************/
float Calculate_Line_error(void){
	float weighted_sum = 0.0f;
			uint8_t sensor_count = 0;
    
	for(int i = 0; i < IR_Num; i++) {
        if(IR_Data_number[i] == 1) { //��ȡ����
            weighted_sum += IR_Weights[i];
            sensor_count++;
        }
    }
    
    if(sensor_count > 0) {
        return weighted_sum / sensor_count;
    }else{
		return 10.0f; 
	}
}
/**********************************************************************************
Function:�����⴫�����������
Input: data of IR_Data_number (none)
Output: �Ƿ���ȫ����
************************************************************************************/
uint8_t Is_Line_Lost(void)
{
    for(int i = 0; i < IR_Num; i++) {
        if(IR_Data_number[i] == 1) {
            return 0; // ������һ����������⵽��
        }
    }
    return 1; // ��ȫ����
}



/**************************************************************************
�������ܣ���·ѭ����ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void Tracking_Init(void)
{
	I2C1_Init();
	Tracking_SetCalibrationStatus(DISABLE);
	
}
/**************************************************************************
�������ܣ�I2C1��ʼ�����������·ѭ��ͨ��
��ڲ�������
����  ֵ����
**************************************************************************/
void I2C1_Init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);	//�������F1��F4�Ĳ�ͬ������
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	I2C_InitTypeDef I2C_InitStructure;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;	//�ظ�Ӧ��
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;	//��Ϊ�ӻ�ʱ���Լ��ĵ�ַλ��
	I2C_InitStructure.I2C_ClockSpeed = 100*1000;	//IIC���ʣ�0-400Khz������IIC���ʵ�ԭ��������������Ϊ�ߵ�ƽ�Ƕ�ȡ���ݣ����������ص�����ʱ���Ӱ�����ݵĶ�ȡ��
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;	//����������������100khz-400khz��������Ҫ����Ҫѡ
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;  //ѡ��IICģʽ
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;	//��Ϊ�ӻ��ĵ�ַ
	I2C_Init(I2C1, &I2C_InitStructure);

	I2C_Cmd(I2C1, ENABLE);
}


/**************************************************************************
�������ܣ�ʹ�ܻ�ʧ�ܰ�·ѭ��У׼λ
��ڲ������Ĵ�����ַ
����  ֵ����
**************************************************************************/
void Tracking_SetCalibrationStatus(FunctionalState function)
{
	if(function==ENABLE){
		Tracking_WriteReg(TRACKING_CALIBRATION_STATUS_REG,0x01);
	}else if(function==DISABLE){
		Tracking_WriteReg(TRACKING_CALIBRATION_STATUS_REG,0x00);
	}
}

/**************************************************************************
�������ܣ�����·ѭ����Ӧ����
��ڲ������Ĵ�����ַ
����  ֵ����
**************************************************************************/
uint8_t Tracking_ReadTrackingData(void)
{
	return Tracking_Read8bitRegister(TRACKING_TRACKING_DATA_REG);
}


/**************************************************************************
�������ܣ��ȴ�I2C�¼�
��ڲ������Ĵ�����ַ
����  ֵ����
**************************************************************************/
void I2C_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
	uint32_t Timeout;
	Timeout = 10000;									//������ʱ����ʱ��
	while (I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS)	//ѭ���ȴ�ָ���¼�
	{
		Timeout --;										//�ȴ�ʱ������ֵ�Լ�
		if (Timeout == 0)								//�Լ���0�󣬵ȴ���ʱ
		{
			/*��ʱ�Ĵ�������룬������ӵ��˴�*/
			break;										//�����ȴ���������
		}
	}
}

/**************************************************************************
�������ܣ�I2C1д8λ�Ĵ���
��ڲ������Ĵ�����ַ
����  ֵ����
**************************************************************************/
void Tracking_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	I2C_GenerateSTART(I2C1, ENABLE);										//Ӳ��I2C������ʼ����
	I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT);					//�ȴ�EV5
	
	I2C_Send7bitAddress(I2C1, TRACKING_SLAVE_7BITADDRESS, I2C_Direction_Transmitter);	//Ӳ��I2C���ʹӻ���ַ������Ϊ����
	I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);	//�ȴ�EV6
	
	I2C_SendData(I2C1, RegAddress);											//Ӳ��I2C���ͼĴ�����ַ
	I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING);			//�ȴ�EV8
	
	I2C_SendData(I2C1, Data);												//Ӳ��I2C��������
	I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED);				//�ȴ�EV8_2
	
	I2C_GenerateSTOP(I2C1, ENABLE);											//Ӳ��I2C������ֹ����
}


/**************************************************************************
�������ܣ�I2C1��8λ�Ĵ���
��ڲ������Ĵ�����ַ
����  ֵ����
**************************************************************************/
uint8_t Tracking_Read8bitRegister(uint8_t address)
{
	uint8_t Data;
	
	I2C_GenerateSTART(I2C1, ENABLE);										//Ӳ��I2C������ʼ����
	I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT);					//�ȴ�EV5
	
	I2C_Send7bitAddress(I2C1,TRACKING_SLAVE_7BITADDRESS , I2C_Direction_Transmitter);	//Ӳ��I2C���ʹӻ���ַ������Ϊ����
	I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);	//�ȴ�EV6
	
	I2C_SendData(I2C1, address);											//Ӳ��I2C���ͼĴ�����ַ
	I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED);				//�ȴ�EV8_2
	
	I2C_GenerateSTART(I2C1, ENABLE);										//Ӳ��I2C�����ظ���ʼ����
	I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT);					//�ȴ�EV5
	
	I2C_Send7bitAddress(I2C1, TRACKING_SLAVE_7BITADDRESS, I2C_Direction_Receiver);		//Ӳ��I2C���ʹӻ���ַ������Ϊ����
	I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);		//�ȴ�EV6
	
	I2C_AcknowledgeConfig(I2C1, DISABLE);									//�ڽ������һ���ֽ�֮ǰ��ǰ��Ӧ��ʧ��
	I2C_GenerateSTOP(I2C1, ENABLE);											//�ڽ������һ���ֽ�֮ǰ��ǰ����ֹͣ����
	
	I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED);				//�ȴ�EV7
	Data = I2C_ReceiveData(I2C1);											//�������ݼĴ���
	
	I2C_AcknowledgeConfig(I2C1, ENABLE);									//��Ӧ��ָ�Ϊʹ�ܣ�Ϊ�˲�Ӱ��������ܲ����Ķ�ȡ���ֽڲ���
	
	return Data;
}



///**************************************************************************
//Function: Serial port 5 initialization
//Input   : none
//Output  : none
//�������ܣ�����5��ʼ��
//��ڲ�������
//����  ֵ����
//**************************************************************************/
//void uart5_init(u32 bound)
//{  	 
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	
//	//PC12 TX
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	 //Enable the gpio clock  //ʹ��GPIOʱ��
//	//PD2 RX
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	 //Enable the gpio clock  //ʹ��GPIOʱ��
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); //Enable the Usart clock //ʹ��USARTʱ��

//	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5);	
//	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2 ,GPIO_AF_UART5);	 
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
//	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //���ģʽ
//	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //�������
//	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //����50MHZ
//	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
//	GPIO_Init(GPIOC, &GPIO_InitStructure);  		          //��ʼ��

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
//	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //���ģʽ
//	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //�������
//	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //����50MHZ
//	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
//	GPIO_Init(GPIOD, &GPIO_InitStructure);  		          //��ʼ��
//	
//	//UsartNVIC configuration //UsartNVIC����
//	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
//	//Preempt priority //��ռ���ȼ�
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;
//	//Sub priority //�����ȼ�
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
//	//Enable the IRQ channel //IRQͨ��ʹ��	
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
//	//Initialize the VIC register with the specified parameters 
//	//����ָ���Ĳ�����ʼ��VIC�Ĵ���		
//	NVIC_Init(&NVIC_InitStructure);
//	
//	//USART Initialization Settings ��ʼ������
//	USART_InitStructure.USART_BaudRate = bound; //Port rate //���ڲ�����
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //�ֳ�Ϊ8λ���ݸ�ʽ
//	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //һ��ֹͣ
//	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //����żУ��λ
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //��Ӳ������������
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //�շ�ģʽ
//	USART_Init(UART5, &USART_InitStructure);      //Initialize serial port 5 //��ʼ������5
//	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //�������ڽ����ж�
//	USART_Cmd(UART5, ENABLE);                     //Enable serial port 5 //ʹ�ܴ���5
//}


///**************************************************************************
//Function: Serial port 5 receives interrupted
//Input   : none
//Output  : none
//�������ܣ�����5�����ж�
//��ڲ�������
//����  ֵ����
//**************************************************************************/
//int UART5_IRQHandler(void)
//{	
//	//Check if data is received //�ж��Ƿ���յ�����
//	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET){
//		
//		
//		
//		USART_ClearITPendingBit(UART5,USART_IT_RXNE);
//	} 
//  return 0;	
//}


