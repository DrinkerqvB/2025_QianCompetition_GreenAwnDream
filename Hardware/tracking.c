#include "tracking.h"
#include "string.h"
const float IR_Weights[8] = {-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5};
extern int Time_count;
QueueHandle_t xControlQueue = NULL;//给了一个队列的基本指针***出现问题要回去看
u8 rx_buff[Package_size];//用于直接接收串口数据
u8 new_package[Package_size];//用于转换为数字
u8 g_new_package_flag = 0;//接收到新的一包数据标志

u8 IR_Data_number[IR_Num];//（所需数据）最终得到的探测数据，从0位到7位为x1-x8

/**************************************************************************
函数功能：FreeRTOS任务，八路循迹数据处理
入口参数：无
返回  值：无
**************************************************************************/
void Tracking_task(void *pvParameters)
{ 
	
		Deal_Usart_Data();
		float error =  Calculate_Line_error();
			
		ControlState cmd;
		cmd.line_error = error;
		cmd.speed_setpoint = 0.5f;      // 固定速度0.5m/s
		cmd.turn_gain = 20.0f * error; // KP=20
		
		 
//		xQueueOverwrite(xControlQueue, &cmd);
//		vTaskDelay(pdMS_TO_TICKS(5));

	
}
/**************************************************************************
函数功能：八路循迹初始化
入口参数：无
返回  值：无
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
函数功能：串口5发送字符数组
入口参数：无
返回  值：无
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
函数功能：串口5发送数据
入口参数：要发送的数据
返回  值：无
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
函数功能：串口5初始化
入口参数：无
返回  值：无
**************************************************************************/
void uart5_init(u32 bound)
{  	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//PC12 TX
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	 //Enable the gpio clock  //使能GPIO时钟
	//PD2 RX
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	 //Enable the gpio clock  //使能GPIO时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); //Enable the Usart clock //使能USART时钟

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5);	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2 ,GPIO_AF_UART5);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //高速50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);  		          //初始化

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //高速50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
	GPIO_Init(GPIOD, &GPIO_InitStructure);  		          //初始化
	
	//UsartNVIC configuration //UsartNVIC配置
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	//Preempt priority //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;
	//Sub priority //子优先级
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
	USART_Init(UART5, &USART_InitStructure);      //Initialize serial port 5 //初始化串口5
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //开启串口接受中断
	USART_Cmd(UART5, ENABLE);                     //Enable serial port 5 //使能串口5
}


/**************************************************************************
Function: Serial port 5 receives interrupted
Input   : none
Output  : none
函数功能：串口5接收中断
入口参数：无
返回  值：无
**************************************************************************/
int UART5_IRQHandler(void)
{	
	u8 Usart_Receive;

	//Check if data is received //判断是否接收到数据
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET){
		if(Time_count<CONTROL_DELAY){
			//开机10秒前不处理数据
			return 0;	
		}
		
		Usart_Receive=USART_ReceiveData(UART5); //读取数据
		//数据整合
		Deal_IR_Usart(Usart_Receive);
		
		USART_ClearITPendingBit(UART5,USART_IT_RXNE);
	} 
  return 0;	
}

/**************************************************************************
函数功能：串口5接收字节整合为数据包
入口参数：接受的字节
返回  值：无
**************************************************************************/
void Deal_IR_Usart(u8 rxtemp)
{
	static u8 g_start = 0;
	static u8 step = 0;
	if(rxtemp == '$'){  
		g_start = 1;//开始接收标志
		rx_buff[step] = rxtemp;
		step++;
	}else{
		if(g_start == 0){
			return;
		}else{
			rx_buff[step] = rxtemp;
			step ++;
			if(rxtemp == '#')//结束
			{
				g_start = 0;
				step = 0;
				memcpy(new_package,rx_buff,Package_size);//赋值
				g_new_package_flag = 1;
				memset(rx_buff,0,Package_size);//清空数据

			}
      
			if(step >= Package_size)//数据异常
			{
				g_start = 0;
				step = 0;
				memset(rx_buff,0,Package_size);//清空数据
			}
		}
  }
}

/**************************************************************************
函数功能：数据包内容由char转换为数字
入口参数：无
返回  值：无
**************************************************************************/
//数据例子：$D,x1:0,x2:0,x3:0,x4:0,x5:0,x6:0,x7:0,x8:0#
void Deal_Usart_Data(void) //处理数字型数据
{
	for(u8 i = 0;i<IR_Num;i++)
	{
		IR_Data_number[i] = (new_package[6+i*5]-'0');//把字符转成数字 //6 11 16 21 26 ...
	}
	g_new_package_flag = 0;
	memset(new_package,0,Package_size);//清除旧数据
  
}




/**************************************************************************
Function: Calculates the check bits of data to be sent/received
Input   : Count_Number: The first few digits of a check; Mode: 0-Verify the received data, 1-Validate the sent data
Output  : Check result
函数功能：计算要发送/接收的数据校验结果
入口参数：Count_Number：校验的前几位数；Mode：0-对接收数据进行校验，1-对发送数据进行校验
返回  值：校验结果
**************************************************************************/
/*
u8 Check_Sum(unsigned char Count_Number,unsigned char Mode)
{
	unsigned char check_sum=0,k;
	
	//Validate the data to be sent
	//对要发送的数据进行校验
	if(Mode==1)
	for(k=0;k<Count_Number;k++)
	{
	check_sum=check_sum^Send_Data.buffer[k];
	}
	
	//Verify the data received
	//对接收到的数据进行校验
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
Output: 偏差值
********************************************************************************************************
*/
float Calculate_Line_error(void){
	float weighted_sum = 0.0f;
			uint8_t sensor_count = 0;
    
	for(int i = 0; i < IR_Num; i++) {
        if(IR_Data_number[i] == 1) { //读取黑线
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
Function:检查红外传感器脱线情况
Input: data of IR_Data_number (none)
Output: 是否完全脱线
************************************************************************************/
uint8_t Is_Line_Lost(void)
{
    for(int i = 0; i < IR_Num; i++) {
        if(IR_Data_number[i] == 1) {
            return 0; // 至少有一个传感器检测到线
        }
    }
    return 1; // 完全脱线
}



