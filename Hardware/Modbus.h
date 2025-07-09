#ifndef __MODBUS_H
#define __MODBUS_H
#include "system.h"

/* 硬件引脚定义 -------------------------------------------------------------*/
#define RS485_IO_GPIO_PORT    GPIOA
#define RS485_IO_PIN          GPIO_Pin_1     // 根据实际电路修改
#define RS485_IO_RCC          RCC_AHB1Periph_GPIOA

/* 常量定义 -----------------------------------------------------------------*/
#define RX_BUF_SIZE       9   // 接收缓冲区大小（含2字节CRC）
#define TX_BUF_SIZE       8   // 发送请求帧长度
#define POWER_DATA_COUNT  6   // 需读取的参数数量

/* 类型定义 -----------------------------------------------------------------*/
typedef enum {
    Voltage      = 0x0000,  // 电压寄存器地址
    Current      = 0x0008,  // 电流寄存器地址
    ActivePower  = 0x0012,  // 有功功率寄存器地址
    PowerFactor  = 0x002A,  // 功率因数寄存器地址
    Frequency    = 0x0036,  // 频率寄存器地址
    ActiveEnergy = 0x0100,  // 有功电能寄存器地址
    Invalid      = 0xFFFF   // 无效地址标记
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
        uint8_t Red;    // RGB红灯亮度(0-255)
        uint8_t Green;  // 绿灯亮度
        uint8_t Blue;   // 蓝灯亮度
    } RGB_LED;
    
    char OLED_Display[32]; // OLED显示缓存
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
