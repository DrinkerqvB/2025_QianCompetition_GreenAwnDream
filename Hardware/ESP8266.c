#include "ESP8266.h"
//发送到串口上后清空数组！！

uint8_t ESP8266_ReceiveBuf2[ESP8266_RECEIVE_LENGTH];//接收暂存帧
uint8_t ESP8266_ReceiveCmd[ESP8266_RECEIVE_LENGTH];//接收
//uint16_t pt_w2=0;
uint8_t aRxBuffer;			//接收中断缓冲

char RST_Start[]="AT+RST\r\n";
char MODE_Start[]="AT+CWMODE=1\r\n";
char WIFI_Start[]="AT+CWJAP=\"2\",\"12345678\"\r\n";
char Connected_Start[]="AT+CIPMUX=0\r\n";
char TCP_Start[]="AT+CIPSTART=\"TCP\",\"192.168.214.20\",8234\r\n";
char SeriaNet_Start[]="AT+CIPMODE=1\r\n";
char SeriaNet_Exit[]="+++";
char TX_Start[]="AT+CIPSEND\r\n";
char Type[]="Type";

Modbus_Typedef Modbus_Type=Modbus_Type_A;

/**************************************************************************
Function: Usartx3, Usartx1,Usartx5 and CAN send data task 
Input   : none
Output  : none
函数功能：串口3、串口1、串口5、CAN发送数据任务
入口参数：无
返回  值：无
**************************************************************************/
void ESP8266_task(void *pvParameters)
{
	 u32 lastWakeTime = getSysTickCnt();
	
   while(1)
    {	
			//The task is run at 20hz
			//此任务以20Hz的频率运行
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_20_HZ));
			
		
	}
}



void ESP8266_Init(void)
{
	uart3_init(115200);
	ESP8266_Command(RST_Start);
	ESP8266_Command(MODE_Start);
	ESP8266_Command(WIFI_Start);
	ESP8266_Command(Connected_Start);
	ESP8266_Command(TCP_Start);
	ESP8266_Command(SeriaNet_Start);
	ESP8266_Command(TX_Start);		//发送指令
}


void ESP8266_Command(char* Command_AT)
{
	int Lenth=strlen(Command_AT);
	
	USART3_SendString(Command_AT);
	
	delay_ms(2000);
	printf("%s",Command_AT);
	//pt_w2=0;
}

/* 发送单个字符 */
void USART3_SendChar(uint8_t ch)
{
    while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
    USART_SendData(USART3, ch);
}

/* 发送字符串 */
void USART3_SendString(char* str)
{
    while(*str) {
        USART3_SendChar(*str++);
    }
}

/* 接收字符（阻塞式）*/
uint8_t USART3_ReceiveChar(void)
{
    while(USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == RESET);
    return USART_ReceiveData(USART3);
}

/**************************************************************************
Function: Serial port 3 initialization
Input   : none
Output  : none
函数功能：串口3初始化
入口参数：无
返回  值：无
**************************************************************************/
void uart3_init(u32 bound)
{  	 
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	 //Enable the gpio clock  //使能GPIO时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //Enable the Usart clock //使能USART时钟
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3);	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //高速50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
	GPIO_Init(GPIOD, &GPIO_InitStructure);  		          //初始化
	
  //UsartNVIC configuration //UsartNVIC配置
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	//Preempt priority //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;
	//Preempt priority //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	//Enable the IRQ channel //IRQ通道使能	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
  //Initialize the VIC register with the specified parameters 
	//根据指定的参数初始化VIC寄存器		
	NVIC_Init(&NVIC_InitStructure);
	
  //USART Initialization Settings 初始化设置
	USART_InitStructure.USART_BaudRate = bound; //Port rate //串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //一个停止
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //收发模式
  USART_Init(USART3, &USART_InitStructure);      //Initialize serial port 3 //初始化串口3
	
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //开启串口接受中断
  USART_Cmd(USART3, ENABLE);                     //Enable serial port 3 //使能串口3 
}




/**************************************************************************
Function: Serial port 3 receives interrupted
Input   : none
Output  : none
函数功能：串口3接收中断
入口参数：无
返回  值：无
**************************************************************************/
int USART3_IRQHandler(void)
{	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
        uint8_t aRxBuffer = USART_ReceiveData(USART3);
        // 处理接收到的数据
		ESP8266_RxCpltCallback(aRxBuffer);
        USART_SendData(USART3, aRxBuffer); // 回传测试
    }
}

void ESP8266_RxCpltCallback(uint8_t data)
{
	static uint16_t pReceiveBuf2=0;
	ESP8266_ReceiveBuf2[pReceiveBuf2++]=data;
	
	if((ESP8266_ReceiveBuf2[pReceiveBuf2-1] == 0x0A)&&(ESP8266_ReceiveBuf2[pReceiveBuf2-2] == 0x0D)) //判断结束位 /r/n
	{
		//printf("%s",ESP8266_ReceiveBuf2);
		//while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);//检测USART2发送结束
		pReceiveBuf2=0;
		memcpy(ESP8266_ReceiveCmd,ESP8266_ReceiveBuf2,ESP8266_RECEIVE_LENGTH);
		memset(ESP8266_ReceiveBuf2,0x00,sizeof(ESP8266_ReceiveBuf2)); //清空数组
	}
	
}


