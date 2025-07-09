#ifndef __MODBUS_H
#define __MODBUS_H
#include "system.h"

/* Ӳ�����Ŷ��� -------------------------------------------------------------*/
#define RS485_IO_GPIO_PORT    GPIOA
#define RS485_IO_PIN          GPIO_Pin_1     // ����ʵ�ʵ�·�޸�
#define RS485_IO_RCC          RCC_AHB1Periph_GPIOA

/* �������� -----------------------------------------------------------------*/
#define RX_BUF_SIZE       9   // ���ջ�������С����2�ֽ�CRC��
#define TX_BUF_SIZE       8   // ��������֡����
#define POWER_DATA_COUNT  6   // ���ȡ�Ĳ�������

/* ���Ͷ��� -----------------------------------------------------------------*/
typedef enum {
    Voltage      = 0x0000,  // ��ѹ�Ĵ�����ַ
    Current      = 0x0008,  // �����Ĵ�����ַ
    ActivePower  = 0x0012,  // �й����ʼĴ�����ַ
    PowerFactor  = 0x002A,  // ���������Ĵ�����ַ
    Frequency    = 0x0036,  // Ƶ�ʼĴ�����ַ
    ActiveEnergy = 0x0100,  // �й����ܼĴ�����ַ
    Invalid      = 0xFFFF   // ��Ч��ַ���
} RegAddr_t;

typedef struct {
    float Voltage;
    float Current;
    float Frequency;
    float ActivePower;
    float ActiveEnergy;
    float PowerFactor; 
    uint8_t CommStatus;
    
    struct {
        uint8_t Red;    // RGB�������(0-255)
        uint8_t Green;  // �̵�����
        uint8_t Blue;   // ��������
    } RGB_LED;
    
    char OLED_Display[32]; // OLED��ʾ����
} PowerMeter_t;



uint16_t ModbusCRC_CheckTable(uint8_t *data, uint16_t length);
void Modbus_Init(void);
static void Modbus_GPIOConfig(void);
static void Delay(uint32_t nCount);
void USART1_Init(u32 bound);
void PowerMeter_RequestData(void);
void PowerMeter_StartReceive(void);
void PowerMeter_DataAnalyse(void);
static float RegToFloat(void);

#endif
