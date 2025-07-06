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
static void USART2_Init(void);
static void GPIO_Config(void);
static void NVIC_Config(void);
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
    /* ����ϵͳʱ��Ϊ168MHz (����ʵ����������) */
    //RCC_DeInit();
    //SystemInit();
    
    /* ��ʼ������ */
    GPIO_Config();
    USART2_Init();
    NVIC_Config();
    
    /* ��ʼ�����ܼ����ṹ�� */
    memset(&powerMeter, 0, sizeof(PowerMeter_t));
    powerMeter.CommStatus = 1; // ��ʼ״̬Ϊͨ���쳣
	PowerMeter_StartReceive();
}

/**
  * @brief  GPIO����
  */
static void GPIO_Config(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* ����GPIOʱ�� */
    RCC_AHB1PeriphClockCmd(USART2_GPIO_RCC | RS485_IO_RCC, ENABLE);
    
    /* ����USART2 TX/RX���� */
    GPIO_InitStructure.GPIO_Pin = USART2_TX_PIN | USART2_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(USART2_GPIO_PORT, &GPIO_InitStructure);
    
    /* ����USART2���ù��� */
    GPIO_PinAFConfig(USART2_GPIO_PORT, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(USART2_GPIO_PORT, GPIO_PinSource3, GPIO_AF_USART2);
    
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
  * @brief  USART2��ʼ��
  */
static void USART2_Init(void) {
    USART_InitTypeDef USART_InitStructure;
    
    /* ����USART2ʱ�� */
    RCC_APB1PeriphClockCmd(USART2_RCC, ENABLE);
    
    /* USART2���� */
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
    USART_Init(USART2_PERIPH, &USART_InitStructure);
    
    /* ����USART2 */
    USART_Cmd(USART2_PERIPH, ENABLE);
    
    /* ���ý����ж� */
    USART_ITConfig(USART2_PERIPH, USART_IT_RXNE, ENABLE);
}

/**
  * @brief  NVIC����
  */
static void NVIC_Config(void) {
    NVIC_InitTypeDef NVIC_InitStructure;
    
    /* ����USART2�ж� */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
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
        USART_SendData(USART2_PERIPH, txBuffer[i]);
        while (USART_GetFlagStatus(USART2_PERIPH, USART_FLAG_TXE) == RESET);
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
  * @brief  USART2�жϷ�����
  */
void USART2_IRQHandler(void) {
    static uint8_t rx_index = 0;
    
    if(USART_GetITStatus(USART2_PERIPH, USART_IT_RXNE) != RESET) {
        /* ��ȡ���յ������� */
        rxBuffer[rx_index++] = USART_ReceiveData(USART2_PERIPH);
        
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

/**
  * @brief  ������
  *�ǵ�Ҫ��ϵͳ��ʼ������ 
  */
//    System_Init();

void Modbus_task(void *pvParameters)
{
	 u32 lastWakeTime = getSysTickCnt();
	
   while(1)
    {	
			//The task is run at 20hz
			//��������20Hz��Ƶ������
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_20_HZ));
		
			
		PowerMeter_RequestData();
	}
}
  

//int main(void) {
//    
//    
//    /* ������һ�ν��� */
//    PowerMeter_StartReceive();
//    
//    while(1) {
//        /* ���ڷ����������� */
//        PowerMeter_RequestData();
//        
//        /* ����ʱ��ʵ��Ӧ��Ӧʹ�ö�ʱ�� */
//        for(uint32_t i = 0; i < 1000000; i++);
//    }
//}

