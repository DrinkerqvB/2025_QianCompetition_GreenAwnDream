#include "Modbus.h"

/**
  ******************************************************************************
  * @file    : powermeter_stdperiph.c
  * @author  : Respeke
  * @brief   : 电能计量模块Modbus通信实现（基于STM32F4标准外设库）
  * @notice  : 
  *            - 使用USART2接口实现Modbus RTU协议
  *            - 自动循环读取电压/电流/功率等参数
  *            - 依赖硬件：MAX485芯片+120Ω终端电阻
  *            - 基于STM32F407VET6芯片
  ******************************************************************************
  */

#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "misc.h"
#include <string.h>



/* 全局变量 ----------------------------------------------------------------*/
static uint8_t txBuffer[TX_BUF_SIZE] = {  // Modbus请求帧模板
    0x01,       // 设备地址
    0x04,       // 功能码(读输入寄存器)
    0x00, 0x00, // 寄存器地址(动态填充)
    0x00, 0x02, // 读取长度(固定2个寄存器)
    0x00, 0x00  // CRC16(动态计算)
};

static uint8_t rxBuffer[RX_BUF_SIZE];    // 接收缓冲区
static uint16_t dataAddr = Voltage;       // 当前读取的寄存器地址
static uint16_t lastAddr = Invalid;       // 上次通信地址(用于超时检测)
PowerMeter_t powerMeter;                 // 电能计量数据结构体

/* 函数声明 ----------------------------------------------------------------*/
static float RegToFloat(void);
static void Delay(uint32_t nCount);

/* Modbus CRC表 */
static const uint16_t crc16_table[] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    /* 完整的CRC表应包含256个条目 */
};

/**
  * @brief  Modbus CRC计算
  * @param  data: 数据指针
  * @param  length: 数据长度
  * @retval CRC16值
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
  * @brief  系统初始化
  */
void Modbus_Init(void) {  
    /* 初始化外设 */
    USART1_Init(9600);
	Modbus_GPIOConfig();
    
    /* 初始化电能计量结构体 */
    memset(&powerMeter, 0, sizeof(PowerMeter_t));
    powerMeter.CommStatus = 1; // 初始状态为通信异常
	PowerMeter_StartReceive();
}

/**
  * @brief  GPIO配置
  */
static void Modbus_GPIOConfig(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* 启用GPIO时钟 */
    RCC_AHB1PeriphClockCmd(RS485_IO_RCC, ENABLE);
   
    /* 配置RS485方向控制引脚 */
    GPIO_InitStructure.GPIO_Pin = RS485_IO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(RS485_IO_GPIO_PORT, &GPIO_InitStructure);
    
    /* 初始设置为接收模式 */
    GPIO_ResetBits(RS485_IO_GPIO_PORT, RS485_IO_PIN);
}



/**
  * @brief  简单延时函数
  * @param  nCount: 延时计数
  */
static void Delay(uint32_t nCount) {
    while(nCount--) {
        __NOP();
    }
}

/**
  * @brief  发送Modbus数据请求帧
  */
void PowerMeter_RequestData(void) {
    /* 动态填充寄存器地址 */
    txBuffer[2] = (uint8_t)(dataAddr >> 8);
    txBuffer[3] = (uint8_t)dataAddr;
    
    /* 计算CRC并填充 */
    uint16_t crc16 = ModbusCRC_CheckTable(txBuffer, TX_BUF_SIZE - 2);
    txBuffer[6] = (uint8_t)crc16;
    txBuffer[7] = (uint8_t)(crc16 >> 8);
    
    /* 切换至发送模式 */
    GPIO_SetBits(RS485_IO_GPIO_PORT, RS485_IO_PIN);
    Delay(1000);  // 确保收发切换稳定
    
    /* 发送数据 */
    for (uint8_t i = 0; i < TX_BUF_SIZE; i++) {
        USART_SendData(USART1, txBuffer[i]);
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    }
    
    /* 切换回接收模式 */
    Delay(1000);
    GPIO_ResetBits(RS485_IO_GPIO_PORT, RS485_IO_PIN);
    
    /* 更新通信状态 */
    powerMeter.CommStatus = (lastAddr == dataAddr) ? 1 : 0;
    lastAddr = dataAddr;
}

/**
  * @brief  启动数据接收
  */
void PowerMeter_StartReceive(void) {
    memset(rxBuffer, 0, RX_BUF_SIZE);
    /* 在标准库中，接收在中断中自动处理 */
}

/**
  * @brief  USART1中断服务函数
  */
void USART1_IRQHandler(void) {
    static uint8_t rx_index = 0;
    
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        /* 读取接收到的数据 */
        rxBuffer[rx_index++] = USART_ReceiveData(USART1);
        
        /* 如果收到完整帧 */
        if(rx_index >= RX_BUF_SIZE) {
            rx_index = 0;
            PowerMeter_DataAnalyse();
        }
    }
}

/**
  * @brief  解析接收到的Modbus数据
  */
void PowerMeter_DataAnalyse(void) {
    /* CRC校验 */
    uint16_t crc16Calc = ModbusCRC_CheckTable(rxBuffer, RX_BUF_SIZE - 2);
    uint16_t crc16Recv = (uint16_t)(rxBuffer[RX_BUF_SIZE-2] | (rxBuffer[RX_BUF_SIZE-1] << 8));
    
    if(crc16Calc == crc16Recv) {
        /* 根据寄存器地址更新对应参数 */
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
                dataAddr = Voltage;  // 循环读取
                break;
            default:
                break;
        }
        powerMeter.CommStatus = 0; // 通信正常
    } else {
        powerMeter.CommStatus = 1; // 通信异常
    }
    
    PowerMeter_StartReceive(); // 重新启动接收
}

/**
  * @brief  寄存器数据转浮点数
  * @retval 转换后的浮点数值
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
函数功能：串口1初始化
入口参数：无
返 回 值：无
**************************************************************************/
void USART1_Init(u32 bound)
{  	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	 //Enable the gpio clock //使能GPIO时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //Enable the Usart clock //使能USART时钟

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10 ,GPIO_AF_USART1);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //高速50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);  		          //初始化
	
  //UsartNVIC configuration //UsartNVIC配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	//Preempt priority //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
	//Subpriority //子优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	//Enable the IRQ channel //IRQ通道使能
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
  //Initialize the VIC register with the specified parameters 
	//根据指定的参数初始化VIC寄存器	
	NVIC_Init(&NVIC_InitStructure);	
	
  //USART Initialization Settings 初始化设置
	USART_InitStructure.USART_BaudRate = bound; //Port rate //串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //收发模式
	USART_Init(USART1, &USART_InitStructure); //Initialize serial port 1 //初始化串口1
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //开启串口接受中断
	USART_Cmd(USART1, ENABLE);                     //Enable serial port 1 //使能串口1
}


