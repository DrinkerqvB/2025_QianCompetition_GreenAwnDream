#include "tracking.h"
#include "string.h"
const float IR_Weights[8] = {-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5};//权重矩阵

u8 IR_Data_number[IR_Num];//（所需数据）最终得到的探测数据，从0位到7位为x1-x8
extern ControlState cmd;//存储偏差、前进速度、转向角速度
extern FlagStatus Car_StopCmd;
float MoveX_mps; //前进速度 m/s
float error; //偏差，由IR_Data_number与IR_Weights做内积得出

/**************************************************************************
函数功能：任务，八路循迹数据处理
入口参数：无
返回  值：无
**************************************************************************/
void Tracking_task(void)
{ 
	uint8_t data=0;
	data=Tracking_ReadTrackingData();
	Tracking_HexToBin(data);
		
		error =  Calculate_Line_error();
	//printf("%d,%d,%d,%d,%d,%d,%d,%d,%f\n",IR_Data_number[0],IR_Data_number[1],IR_Data_number[2],IR_Data_number[3],IR_Data_number[4],IR_Data_number[5],IR_Data_number[6],IR_Data_number[7],error);
			
		
		cmd.line_error = error;
	if(Car_StopCmd==RESET){
		cmd.speed_setpoint = MoveX_mps;      // 固定速度0.5m/s
		cmd.turn_gain = 20.0f*error; // KP=20
	}else{
		cmd.speed_setpoint = 0;      // 固定速度0m/s
		cmd.turn_gain = 0; // KP=0
	}
}

/**************************************************************************
函数功能：将寄存器内容转化为数字
入口参数：无
返回  值：无
**************************************************************************/
void Tracking_HexToBin(uint8_t hexData)
{
	uint8_t binData[8]={0};
	for(uint8_t i=0;i<8;i++){
		binData[i] = (hexData >> (7-i)) & 0x01;
	}
	memcpy(IR_Data_number,binData,8);
}

/*****************************************************************************************************
Functions: Calculate 
Input :data of IR_Data_number none 
Output: 偏差值
******************************************************************************************************/
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
/**********************************************************************************
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



/**************************************************************************
函数功能：八路循迹初始化
入口参数：无
返回  值：无
**************************************************************************/
void Tracking_Init(void)
{
	I2C1_Init();
	Tracking_SetCalibrationStatus(DISABLE);
	
}
/**************************************************************************
函数功能：I2C1初始化，用于与八路循迹通信
入口参数：无
返回  值：无
**************************************************************************/
void I2C1_Init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);	//这里就是F1和F4的不同！！！
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	I2C_InitTypeDef I2C_InitStructure;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;	//回复应答
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;	//作为从机时，自己的地址位数
	I2C_InitStructure.I2C_ClockSpeed = 100*1000;	//IIC速率：0-400Khz（限制IIC速率的原因是弱上拉，因为高电平是读取数据，当弱上拉回调不及时则会影响数据的读取）
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;	//如果上面参数速率在100khz-400khz，则这里要必须要选
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;  //选择IIC模式
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;	//作为从机的地址
	I2C_Init(I2C1, &I2C_InitStructure);

	I2C_Cmd(I2C1, ENABLE);
}


/**************************************************************************
函数功能：使能或失能八路循迹校准位
入口参数：寄存器地址
返回  值：无
**************************************************************************/
void Tracking_SetCalibrationStatus(FunctionalState function)
{
	if(function==ENABLE){
		Tracking_WriteReg(TRACKING_CALIBRATION_STATUS_REG,0x01);
	}else if(function==DISABLE){
		Tracking_WriteReg(TRACKING_CALIBRATION_STATUS_REG,0x00);
	}
}

/**************************************************************************
函数功能：读八路循迹感应数据
入口参数：寄存器地址
返回  值：无
**************************************************************************/
uint8_t Tracking_ReadTrackingData(void)
{
	return Tracking_Read8bitRegister(TRACKING_TRACKING_DATA_REG);
}


/**************************************************************************
函数功能：等待I2C事件
入口参数：寄存器地址
返回  值：无
**************************************************************************/
void I2C_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
	uint32_t Timeout;
	Timeout = 10000;									//给定超时计数时间
	while (I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS)	//循环等待指定事件
	{
		Timeout --;										//等待时，计数值自减
		if (Timeout == 0)								//自减到0后，等待超时
		{
			/*超时的错误处理代码，可以添加到此处*/
			break;										//跳出等待，不等了
		}
	}
}

/**************************************************************************
函数功能：I2C1写8位寄存器
入口参数：寄存器地址
返回  值：无
**************************************************************************/
void Tracking_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	I2C_GenerateSTART(I2C1, ENABLE);										//硬件I2C生成起始条件
	I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT);					//等待EV5
	
	I2C_Send7bitAddress(I2C1, TRACKING_SLAVE_7BITADDRESS, I2C_Direction_Transmitter);	//硬件I2C发送从机地址，方向为发送
	I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);	//等待EV6
	
	I2C_SendData(I2C1, RegAddress);											//硬件I2C发送寄存器地址
	I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING);			//等待EV8
	
	I2C_SendData(I2C1, Data);												//硬件I2C发送数据
	I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED);				//等待EV8_2
	
	I2C_GenerateSTOP(I2C1, ENABLE);											//硬件I2C生成终止条件
}


/**************************************************************************
函数功能：I2C1读8位寄存器
入口参数：寄存器地址
返回  值：无
**************************************************************************/
uint8_t Tracking_Read8bitRegister(uint8_t address)
{
	uint8_t Data;
	
	I2C_GenerateSTART(I2C1, ENABLE);										//硬件I2C生成起始条件
	I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT);					//等待EV5
	
	I2C_Send7bitAddress(I2C1,TRACKING_SLAVE_7BITADDRESS , I2C_Direction_Transmitter);	//硬件I2C发送从机地址，方向为发送
	I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);	//等待EV6
	
	I2C_SendData(I2C1, address);											//硬件I2C发送寄存器地址
	I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED);				//等待EV8_2
	
	I2C_GenerateSTART(I2C1, ENABLE);										//硬件I2C生成重复起始条件
	I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT);					//等待EV5
	
	I2C_Send7bitAddress(I2C1, TRACKING_SLAVE_7BITADDRESS, I2C_Direction_Receiver);		//硬件I2C发送从机地址，方向为接收
	I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);		//等待EV6
	
	I2C_AcknowledgeConfig(I2C1, DISABLE);									//在接收最后一个字节之前提前将应答失能
	I2C_GenerateSTOP(I2C1, ENABLE);											//在接收最后一个字节之前提前申请停止条件
	
	I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED);				//等待EV7
	Data = I2C_ReceiveData(I2C1);											//接收数据寄存器
	
	I2C_AcknowledgeConfig(I2C1, ENABLE);									//将应答恢复为使能，为了不影响后续可能产生的读取多字节操作
	
	return Data;
}



///**************************************************************************
//Function: Serial port 5 initialization
//Input   : none
//Output  : none
//函数功能：串口5初始化
//入口参数：无
//返回  值：无
//**************************************************************************/
//void uart5_init(u32 bound)
//{  	 
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	
//	//PC12 TX
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	 //Enable the gpio clock  //使能GPIO时钟
//	//PD2 RX
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	 //Enable the gpio clock  //使能GPIO时钟
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); //Enable the Usart clock //使能USART时钟

//	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5);	
//	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2 ,GPIO_AF_UART5);	 
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
//	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //输出模式
//	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //推挽输出
//	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //高速50MHZ
//	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
//	GPIO_Init(GPIOC, &GPIO_InitStructure);  		          //初始化

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
//	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //输出模式
//	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //推挽输出
//	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //高速50MHZ
//	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
//	GPIO_Init(GPIOD, &GPIO_InitStructure);  		          //初始化
//	
//	//UsartNVIC configuration //UsartNVIC配置
//	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
//	//Preempt priority //抢占优先级
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;
//	//Sub priority //子优先级
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
//	//Enable the IRQ channel //IRQ通道使能	
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
//	//Initialize the VIC register with the specified parameters 
//	//根据指定的参数初始化VIC寄存器		
//	NVIC_Init(&NVIC_InitStructure);
//	
//	//USART Initialization Settings 初始化设置
//	USART_InitStructure.USART_BaudRate = bound; //Port rate //串口波特率
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //字长为8位数据格式
//	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //一个停止
//	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //无奇偶校验位
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //无硬件数据流控制
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //收发模式
//	USART_Init(UART5, &USART_InitStructure);      //Initialize serial port 5 //初始化串口5
//	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //开启串口接受中断
//	USART_Cmd(UART5, ENABLE);                     //Enable serial port 5 //使能串口5
//}


///**************************************************************************
//Function: Serial port 5 receives interrupted
//Input   : none
//Output  : none
//函数功能：串口5接收中断
//入口参数：无
//返回  值：无
//**************************************************************************/
//int UART5_IRQHandler(void)
//{	
//	//Check if data is received //判断是否接收到数据
//	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET){
//		
//		
//		
//		USART_ClearITPendingBit(UART5,USART_IT_RXNE);
//	} 
//  return 0;	
//}


