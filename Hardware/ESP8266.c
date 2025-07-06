#include "ESP8266.h"
//���͵������Ϻ�������飡��

uint8_t ESP8266_ReceiveBuf2[ESP8266_RECEIVE_LENGTH];//�����ݴ�֡
uint8_t ESP8266_ReceiveCmd[ESP8266_RECEIVE_LENGTH];//����
//uint16_t pt_w2=0;
uint8_t aRxBuffer;			//�����жϻ���

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
�������ܣ�����3������1������5��CAN������������
��ڲ�������
����  ֵ����
**************************************************************************/
void ESP8266_task(void *pvParameters)
{
	 u32 lastWakeTime = getSysTickCnt();
	
   while(1)
    {	
			//The task is run at 20hz
			//��������20Hz��Ƶ������
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
	ESP8266_Command(TX_Start);		//����ָ��
}


void ESP8266_Command(char* Command_AT)
{
	int Lenth=strlen(Command_AT);
	
	USART3_SendString(Command_AT);
	
	delay_ms(2000);
	printf("%s",Command_AT);
	//pt_w2=0;
}

/* ���͵����ַ� */
void USART3_SendChar(uint8_t ch)
{
    while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
    USART_SendData(USART3, ch);
}

/* �����ַ��� */
void USART3_SendString(char* str)
{
    while(*str) {
        USART3_SendChar(*str++);
    }
}

/* �����ַ�������ʽ��*/
uint8_t USART3_ReceiveChar(void)
{
    while(USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == RESET);
    return USART_ReceiveData(USART3);
}

/**************************************************************************
Function: Serial port 3 initialization
Input   : none
Output  : none
�������ܣ�����3��ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void uart3_init(u32 bound)
{  	 
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	 //Enable the gpio clock  //ʹ��GPIOʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //Enable the Usart clock //ʹ��USARTʱ��
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3);	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //����50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOD, &GPIO_InitStructure);  		          //��ʼ��
	
  //UsartNVIC configuration //UsartNVIC����
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	//Preempt priority //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;
	//Preempt priority //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	//Enable the IRQ channel //IRQͨ��ʹ��	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
  //Initialize the VIC register with the specified parameters 
	//����ָ���Ĳ�����ʼ��VIC�Ĵ���		
	NVIC_Init(&NVIC_InitStructure);
	
  //USART Initialization Settings ��ʼ������
	USART_InitStructure.USART_BaudRate = bound; //Port rate //���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //һ��ֹͣ
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //�շ�ģʽ
  USART_Init(USART3, &USART_InitStructure);      //Initialize serial port 3 //��ʼ������3
	
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //�������ڽ����ж�
  USART_Cmd(USART3, ENABLE);                     //Enable serial port 3 //ʹ�ܴ���3 
}




/**************************************************************************
Function: Serial port 3 receives interrupted
Input   : none
Output  : none
�������ܣ�����3�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
int USART3_IRQHandler(void)
{	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
        uint8_t aRxBuffer = USART_ReceiveData(USART3);
        // ������յ�������
		ESP8266_RxCpltCallback(aRxBuffer);
        USART_SendData(USART3, aRxBuffer); // �ش�����
    }
}

void ESP8266_RxCpltCallback(uint8_t data)
{
	static uint16_t pReceiveBuf2=0;
	ESP8266_ReceiveBuf2[pReceiveBuf2++]=data;
	
	if((ESP8266_ReceiveBuf2[pReceiveBuf2-1] == 0x0A)&&(ESP8266_ReceiveBuf2[pReceiveBuf2-2] == 0x0D)) //�жϽ���λ /r/n
	{
		//printf("%s",ESP8266_ReceiveBuf2);
		//while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);//���USART2���ͽ���
		pReceiveBuf2=0;
		memcpy(ESP8266_ReceiveCmd,ESP8266_ReceiveBuf2,ESP8266_RECEIVE_LENGTH);
		memset(ESP8266_ReceiveBuf2,0x00,sizeof(ESP8266_ReceiveBuf2)); //�������
	}
	
}


