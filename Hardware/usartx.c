#include "usartx.h"


/**************************************************************************
Function: Serial port 2 initialization
Input   : none
Output  : none
�������ܣ�����2��ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void uart2_init(u32 bound)
{  	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	 //Enable the gpio clock  //ʹ��GPIOʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //Enable the Usart clock //ʹ��USARTʱ��
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2);	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6 ,GPIO_AF_USART2);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //����50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOD, &GPIO_InitStructure);  		          //��ʼ��
	
	//UsartNVIC configuration //UsartNVIC����
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
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
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //һ��ֹͣ
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //�շ�ģʽ
	USART_Init(USART2, &USART_InitStructure);      //Initialize serial port 2 //��ʼ������2
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //�������ڽ����ж�
	USART_Cmd(USART2, ENABLE);                     //Enable serial port 2 //ʹ�ܴ���2 
}




/**************************************************************************
Function: Serial port 2 sends data
Input   : The data to send
Output  : none
�������ܣ�����2��������
��ڲ�����Ҫ���͵�����
����  ֵ����
**************************************************************************/
void usart2_send(u8 data)
{
	USART2->DR = data;
	while((USART2->SR&0x40)==0);	
}



