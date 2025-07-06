#include "tracking.h"
#include "string.h"
const float IR_Weights[8] = {-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5};
extern int Time_count;
QueueHandle_t xControlQueue = NULL;//����һ�����еĻ���ָ��***��������Ҫ��ȥ��
u8 rx_buff[Package_size];//����ֱ�ӽ��մ�������
u8 new_package[Package_size];//����ת��Ϊ����
u8 g_new_package_flag = 0;//���յ��µ�һ�����ݱ�־

u8 IR_Data_number[IR_Num];//���������ݣ����յõ���̽�����ݣ���0λ��7λΪx1-x8

/**************************************************************************
�������ܣ�FreeRTOS���񣬰�·ѭ�����ݴ���
��ڲ�������
����  ֵ����
**************************************************************************/
void Tracking_task(void *pvParameters)
{ 
	
		Deal_Usart_Data();
		float error =  Calculate_Line_error();
			
		ControlState cmd;
		cmd.line_error = error;
		cmd.speed_setpoint = 0.5f;      // �̶��ٶ�0.5m/s
		cmd.turn_gain = 20.0f * error; // KP=20
		
		 
//		xQueueOverwrite(xControlQueue, &cmd);
//		vTaskDelay(pdMS_TO_TICKS(5));

	
}
/**************************************************************************
�������ܣ���·ѭ����ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void Tracking_Init(void)
{
	uart5_init(115200);
	u8 send_buf[8] = "$0,0,1#";

	USART5_SEND(send_buf,strlen((char*)send_buf));
	
}


/**************************************************************************
Function: Serial port 5 sends data
Input   : none
Output  : none
�������ܣ�����5�����ַ�����
��ڲ�������
����  ֵ����
**************************************************************************/
void USART5_SEND(uint8_t *BufferPtr, uint16_t Length)
{
	uint16_t i = 0;	
	for(i=0; i<Length; i++)
	{
		usart5_send(BufferPtr[i]);
	}	 
}

/**************************************************************************
Function: Serial port 5 sends data
Input   : The data to send
Output  : none
�������ܣ�����5��������
��ڲ�����Ҫ���͵�����
����  ֵ����
**************************************************************************/
void usart5_send(u8 data)
{
	UART5->DR = data;
	while((UART5->SR&0x40)==0);	
}

/**************************************************************************
Function: Serial port 5 initialization
Input   : none
Output  : none
�������ܣ�����5��ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void uart5_init(u32 bound)
{  	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//PC12 TX
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	 //Enable the gpio clock  //ʹ��GPIOʱ��
	//PD2 RX
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	 //Enable the gpio clock  //ʹ��GPIOʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); //Enable the Usart clock //ʹ��USARTʱ��

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5);	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2 ,GPIO_AF_UART5);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //����50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOC, &GPIO_InitStructure);  		          //��ʼ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //����50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOD, &GPIO_InitStructure);  		          //��ʼ��
	
	//UsartNVIC configuration //UsartNVIC����
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	//Preempt priority //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;
	//Sub priority //�����ȼ�
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
	USART_Init(UART5, &USART_InitStructure);      //Initialize serial port 5 //��ʼ������5
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //�������ڽ����ж�
	USART_Cmd(UART5, ENABLE);                     //Enable serial port 5 //ʹ�ܴ���5
}


/**************************************************************************
Function: Serial port 5 receives interrupted
Input   : none
Output  : none
�������ܣ�����5�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
int UART5_IRQHandler(void)
{	
	u8 Usart_Receive;

	//Check if data is received //�ж��Ƿ���յ�����
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET){
		if(Time_count<CONTROL_DELAY){
			//����10��ǰ����������
			return 0;	
		}
		
		Usart_Receive=USART_ReceiveData(UART5); //��ȡ����
		//��������
		Deal_IR_Usart(Usart_Receive);
		
		USART_ClearITPendingBit(UART5,USART_IT_RXNE);
	} 
  return 0;	
}

/**************************************************************************
�������ܣ�����5�����ֽ�����Ϊ���ݰ�
��ڲ��������ܵ��ֽ�
����  ֵ����
**************************************************************************/
void Deal_IR_Usart(u8 rxtemp)
{
	static u8 g_start = 0;
	static u8 step = 0;
	if(rxtemp == '$'){  
		g_start = 1;//��ʼ���ձ�־
		rx_buff[step] = rxtemp;
		step++;
	}else{
		if(g_start == 0){
			return;
		}else{
			rx_buff[step] = rxtemp;
			step ++;
			if(rxtemp == '#')//����
			{
				g_start = 0;
				step = 0;
				memcpy(new_package,rx_buff,Package_size);//��ֵ
				g_new_package_flag = 1;
				memset(rx_buff,0,Package_size);//�������

			}
      
			if(step >= Package_size)//�����쳣
			{
				g_start = 0;
				step = 0;
				memset(rx_buff,0,Package_size);//�������
			}
		}
  }
}

/**************************************************************************
�������ܣ����ݰ�������charת��Ϊ����
��ڲ�������
����  ֵ����
**************************************************************************/
//�������ӣ�$D,x1:0,x2:0,x3:0,x4:0,x5:0,x6:0,x7:0,x8:0#
void Deal_Usart_Data(void) //��������������
{
	for(u8 i = 0;i<IR_Num;i++)
	{
		IR_Data_number[i] = (new_package[6+i*5]-'0');//���ַ�ת������ //6 11 16 21 26 ...
	}
	g_new_package_flag = 0;
	memset(new_package,0,Package_size);//���������
  
}




/**************************************************************************
Function: Calculates the check bits of data to be sent/received
Input   : Count_Number: The first few digits of a check; Mode: 0-Verify the received data, 1-Validate the sent data
Output  : Check result
�������ܣ�����Ҫ����/���յ�����У����
��ڲ�����Count_Number��У���ǰ��λ����Mode��0-�Խ������ݽ���У�飬1-�Է������ݽ���У��
����  ֵ��У����
**************************************************************************/
/*
u8 Check_Sum(unsigned char Count_Number,unsigned char Mode)
{
	unsigned char check_sum=0,k;
	
	//Validate the data to be sent
	//��Ҫ���͵����ݽ���У��
	if(Mode==1)
	for(k=0;k<Count_Number;k++)
	{
	check_sum=check_sum^Send_Data.buffer[k];
	}
	
	//Verify the data received
	//�Խ��յ������ݽ���У��
	if(Mode==0)
	for(k=0;k<Count_Number;k++)
	{
	check_sum=check_sum^Receive_Data.buffer[k];
	}
	return check_sum;
}

*/
/*
****************************************************************************************************
Functions: Calculate 
Input :data of IR_Data_number none 
Output: ƫ��ֵ
********************************************************************************************************
*/
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
/*
*********************************************************************************
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



