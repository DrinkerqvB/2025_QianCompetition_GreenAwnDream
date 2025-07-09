#include "Modbus.h"

/**
  ******************************************************************************
  * @file    : powermeter_stdperiph.c
  * @author  : Respeke
  * @brief   : ���ܼ���ģ��Modbusͨ��ʵ�֣�����STM32F4��׼����⣩
  * @notice  : 
  *            - ʹ��USART2�ӿ�ʵ��Modbus RTUЭ��
  *            - �Զ�ѭ����ȡ��ѹ/����/���ʵȲ���
  *            - ����Ӳ����MAX485оƬ+120���ն˵���
  *            - ����STM32F407VET6оƬ
  ******************************************************************************
  */

#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "misc.h"
#include <string.h>



/* ȫ�ֱ��� ----------------------------------------------------------------*/
static uint8_t txBuffer[TX_BUF_SIZE] = {  // Modbus����֡ģ��
    0x01,       // �豸��ַ
    0x04,       // ������(������Ĵ���)
    0x00, 0x00, // �Ĵ�����ַ(��̬���)
    0x00, 0x02, // ��ȡ����(�̶�2���Ĵ���)
    0x00, 0x00  // CRC16(��̬����)
};

static uint8_t rxBuffer[RX_BUF_SIZE];    // ���ջ�����
static uint16_t dataAddr = Voltage;       // ��ǰ��ȡ�ļĴ�����ַ
static uint16_t lastAddr = Invalid;       // �ϴ�ͨ�ŵ�ַ(���ڳ�ʱ���)
PowerMeter_t powerMeter;                 // ���ܼ������ݽṹ��

/* �������� ----------------------------------------------------------------*/
static float RegToFloat(void);
static void Delay(uint32_t nCount);

/* Modbus CRC�� */
static const uint16_t crc16_table[] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    /* ������CRC��Ӧ����256����Ŀ */
};

/**
  * @brief  Modbus CRC����
  * @param  data: ����ָ��
  * @param  length: ���ݳ���
  * @retval CRC16ֵ
  */
uint16_t ModbusCRC_CheckTable(uint8_t *data, uint16_t length) {
    uint8_t temp;
    uint16_t crc = 0xFFFF;
    
    while (length--) {
        temp = *data++ ^ crc;
        crc >>= 8;
        crc ^= crc16_table[temp];
    }
    return crc;
}

/**
  * @brief  ϵͳ��ʼ��
  */
void Modbus_Init(void) {  
    /* ��ʼ������ */
    USART1_Init(9600);
	Modbus_GPIOConfig();
    
    /* ��ʼ�����ܼ����ṹ�� */
    memset(&powerMeter, 0, sizeof(PowerMeter_t));
    powerMeter.CommStatus = 1; // ��ʼ״̬Ϊͨ���쳣
	PowerMeter_StartReceive();
}

/**
  * @brief  GPIO����
  */
static void Modbus_GPIOConfig(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* ����GPIOʱ�� */
    RCC_AHB1PeriphClockCmd(RS485_IO_RCC, ENABLE);
   
    /* ����RS485����������� */
    GPIO_InitStructure.GPIO_Pin = RS485_IO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(RS485_IO_GPIO_PORT, &GPIO_InitStructure);
    
    /* ��ʼ����Ϊ����ģʽ */
    GPIO_ResetBits(RS485_IO_GPIO_PORT, RS485_IO_PIN);
}



/**
  * @brief  ����ʱ����
  * @param  nCount: ��ʱ����
  */
static void Delay(uint32_t nCount) {
    while(nCount--) {
        __NOP();
    }
}

/**
  * @brief  ����Modbus��������֡
  */
void PowerMeter_RequestData(void) {
    /* ��̬���Ĵ�����ַ */
    txBuffer[2] = (uint8_t)(dataAddr >> 8);
    txBuffer[3] = (uint8_t)dataAddr;
    
    /* ����CRC����� */
    uint16_t crc16 = ModbusCRC_CheckTable(txBuffer, TX_BUF_SIZE - 2);
    txBuffer[6] = (uint8_t)crc16;
    txBuffer[7] = (uint8_t)(crc16 >> 8);
    
    /* �л�������ģʽ */
    GPIO_SetBits(RS485_IO_GPIO_PORT, RS485_IO_PIN);
    Delay(1000);  // ȷ���շ��л��ȶ�
    
    /* �������� */
    for (uint8_t i = 0; i < TX_BUF_SIZE; i++) {
        USART_SendData(USART1, txBuffer[i]);
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    }
    
    /* �л��ؽ���ģʽ */
    Delay(1000);
    GPIO_ResetBits(RS485_IO_GPIO_PORT, RS485_IO_PIN);
    
    /* ����ͨ��״̬ */
    powerMeter.CommStatus = (lastAddr == dataAddr) ? 1 : 0;
    lastAddr = dataAddr;
}

/**
  * @brief  �������ݽ���
  */
void PowerMeter_StartReceive(void) {
    memset(rxBuffer, 0, RX_BUF_SIZE);
    /* �ڱ�׼���У��������ж����Զ����� */
}

/**
  * @brief  USART1�жϷ�����
  */
void USART1_IRQHandler(void) {
    static uint8_t rx_index = 0;
    
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        /* ��ȡ���յ������� */
        rxBuffer[rx_index++] = USART_ReceiveData(USART1);
        
        /* ����յ�����֡ */
        if(rx_index >= RX_BUF_SIZE) {
            rx_index = 0;
            PowerMeter_DataAnalyse();
        }
    }
}

/**
  * @brief  �������յ���Modbus����
  */
void PowerMeter_DataAnalyse(void) {
    /* CRCУ�� */
    uint16_t crc16Calc = ModbusCRC_CheckTable(rxBuffer, RX_BUF_SIZE - 2);
    uint16_t crc16Recv = (uint16_t)(rxBuffer[RX_BUF_SIZE-2] | (rxBuffer[RX_BUF_SIZE-1] << 8));
    
    if(crc16Calc == crc16Recv) {
        /* ���ݼĴ�����ַ���¶�Ӧ���� */
        switch(dataAddr) {
            case Voltage:
                powerMeter.Voltage = RegToFloat();
                dataAddr = Current;
                break;
            case Current:
                powerMeter.Current = RegToFloat();
                dataAddr = ActivePower;
                break;
            case ActivePower:
                powerMeter.ActivePower = RegToFloat();
                dataAddr = PowerFactor;
                break;
            case PowerFactor:
                powerMeter.PowerFactor = RegToFloat();
                dataAddr = Frequency;
                break;
            case Frequency:
                powerMeter.Frequency = RegToFloat();
                dataAddr = ActiveEnergy;
                break;
            case ActiveEnergy:
                powerMeter.ActiveEnergy = RegToFloat();
                dataAddr = Voltage;  // ѭ����ȡ
                break;
            default:
                break;
        }
        powerMeter.CommStatus = 0; // ͨ������
    } else {
        powerMeter.CommStatus = 1; // ͨ���쳣
    }
    
    PowerMeter_StartReceive(); // ������������
}

/**
  * @brief  �Ĵ�������ת������
  * @retval ת����ĸ�����ֵ
  */
static float RegToFloat(void) {
    uint32_t rawData = (uint32_t)rxBuffer[3] << 24 | 
                       (uint32_t)rxBuffer[4] << 16 |
                       (uint32_t)rxBuffer[5] << 8  |
                       rxBuffer[6];
    return *(float*)&rawData;
}

/**************************************************************************
Function: Serial port 1 initialization
Input   : none
Output  : none
�������ܣ�����1��ʼ��
��ڲ�������
�� �� ֵ����
**************************************************************************/
void USART1_Init(u32 bound)
{  	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	 //Enable the gpio clock //ʹ��GPIOʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //Enable the Usart clock //ʹ��USARTʱ��

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10 ,GPIO_AF_USART1);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //����50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOA, &GPIO_InitStructure);  		          //��ʼ��
	
  //UsartNVIC configuration //UsartNVIC����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	//Preempt priority //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
	//Subpriority //�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	//Enable the IRQ channel //IRQͨ��ʹ��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
  //Initialize the VIC register with the specified parameters 
	//����ָ���Ĳ�����ʼ��VIC�Ĵ���	
	NVIC_Init(&NVIC_InitStructure);	
	
  //USART Initialization Settings ��ʼ������
	USART_InitStructure.USART_BaudRate = bound; //Port rate //���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //�շ�ģʽ
	USART_Init(USART1, &USART_InitStructure); //Initialize serial port 1 //��ʼ������1
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //�������ڽ����ж�
	USART_Cmd(USART1, ENABLE);                     //Enable serial port 1 //ʹ�ܴ���1
}


