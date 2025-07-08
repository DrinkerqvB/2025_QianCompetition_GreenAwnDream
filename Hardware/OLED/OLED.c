#include "OLED.h"


uint8_t OLED_DispalyBuf[8][128] = {0}; //初始化为0


void OLED_ManualClear(void)
{
	OLED_Printf(1,1,"                ");
	OLED_Printf(2,1,"                ");
	OLED_Printf(3,1,"                ");
	OLED_Printf(4,1,"                ");
	
}



void IIC_init(void)//PB10-->SCL || PB11-->SDA
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

	GPIO_InitTypeDef IIC1_IO;
	IIC1_IO.GPIO_Mode = GPIO_Mode_AF;
	IIC1_IO.GPIO_PuPd = GPIO_PuPd_NOPULL; 
	IIC1_IO.GPIO_OType = GPIO_OType_OD;
	IIC1_IO.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	IIC1_IO.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &IIC1_IO);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);	//这里就是F1和F4的不同！！！
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);
	
	I2C_InitTypeDef IIC1;
	IIC1.I2C_Ack = I2C_Ack_Enable;	//回复应答
	IIC1.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;	//作为从机时，自己的地址位数
	IIC1.I2C_ClockSpeed = 100*1000;	//IIC速率：0-400Khz（限制IIC速率的原因是弱上拉，因为高电平是读取数据，当弱上拉回调不及时则会影响数据的读取）
	IIC1.I2C_DutyCycle = I2C_DutyCycle_2;	//如果上面参数速率在100khz-400khz，则这里要必须要选
	IIC1.I2C_Mode = I2C_Mode_I2C;  //选择IIC模式
	IIC1.I2C_OwnAddress1 = 0x00;	//作为从机的地址
	I2C_Init(I2C2, &IIC1);

	I2C_Cmd(I2C2, ENABLE);
}


void OLED_WriteCommand(uint8_t cmmand)  //在控制位选项上，选择不连续模式，并且片选为命令（DC为0）
{
//	IIC_Start();
//	IIC_send_Byte(0x78);  //通讯地址
//	IIC_rec_ACK();
//	IIC_send_Byte(0x00);  //控制位
//	IIC_rec_ACK();
//	IIC_send_Byte(cmmand);  //数据
//	IIC_rec_ACK();
//	IIC_Stop();
	while(SET == I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));	//是否忙碌
	
	I2C_GenerateSTART(I2C2, ENABLE);	//起始位
	while(I2C_CheckEvent(I2C2 , I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS);
	
	
	I2C_Send7bitAddress(I2C2, 0x78, I2C_Direction_Transmitter); //发送地址
	while(I2C_CheckEvent(I2C2 , I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS);
	
	
	I2C_SendData(I2C2, 0x00);	//发送控制位
	while(I2C_CheckEvent(I2C2 , I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS);
	

	I2C_SendData(I2C2, cmmand);	//发送数据
	while(I2C_CheckEvent(I2C2 , I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS);
	
	
	I2C_GenerateSTOP(I2C2 , ENABLE);
}


void OLED_WriteData(uint8_t Data)  //在控制位选项上，选择不连续模式，并且片选为数据（DC为1）
{
//	IIC_Start();
//	IIC_send_Byte(0x78);  //通讯地址
//	IIC_rec_ACK();
//	IIC_send_Byte(0x40);  //控制位
//	IIC_rec_ACK();
//	IIC_send_Byte(Data);  //数据
//	IIC_rec_ACK();
//	IIC_Stop();
	while(SET == I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));	//是否忙碌
	I2C_GenerateSTART(I2C2, ENABLE);	//起始位
	while(I2C_CheckEvent(I2C2 , I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS);
	
	
	I2C_Send7bitAddress(I2C2, 0x78, I2C_Direction_Transmitter); //发送地址
	while(I2C_CheckEvent(I2C2 , I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS);
	
	
	I2C_SendData(I2C2, 0x40);	//发送控制位
	while(I2C_CheckEvent(I2C2 , I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS);
	

	I2C_SendData(I2C2, Data);	//发送数据
	while(I2C_CheckEvent(I2C2 , I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS);
	
	
	I2C_GenerateSTOP(I2C2 , ENABLE);

}


void OLED_init(void)
{
	IIC_init();
	
	delay_ms(100);
/*具体的初始化命令根据手册来*/
	OLED_WriteCommand(0xAE);
	
	OLED_WriteCommand(0xD5);
	OLED_WriteCommand(0x80);
		
	OLED_WriteCommand(0xA8);
	OLED_WriteCommand(0x3F);
		
	OLED_WriteCommand(0xD3);
	OLED_WriteCommand(0x00);
	
	OLED_WriteCommand(0x40);
	
	OLED_WriteCommand(0xA1);
	
	OLED_WriteCommand(0xC8);

	OLED_WriteCommand(0xDA);
	OLED_WriteCommand(0x12);
	
	OLED_WriteCommand(0x81);
	OLED_WriteCommand(0xCF);

	OLED_WriteCommand(0xD9);
	OLED_WriteCommand(0xF1);

	OLED_WriteCommand(0xDB);
	OLED_WriteCommand(0x30);

	OLED_WriteCommand(0xA4);

	OLED_WriteCommand(0xA6);

	OLED_WriteCommand(0x8D);
	OLED_WriteCommand(0x14);

	OLED_WriteCommand(0xAF);

	delay_ms(100);
	
}

/*
	x:横坐标(0-128) page:指定页（纵坐标0-7）
模块说明:1：每一个字节向下展开
		 2：自动向右寻址，但是不会换行
*/
void OLED_SetCursor(uint8_t x , uint8_t page)
{
	OLED_WriteCommand(0x0f & x); //写入x的低4位
	OLED_WriteCommand(0x10 | (0x0f & (x>>4))); //写入x的高4位,且要求第5为必须为1
	OLED_WriteCommand(0xb0 | page);
}

void OLED_Clear(void)
{
	for(uint8_t i = 0 ; i < 8 ; i++)
	{
		OLED_SetCursor(0 , i);
		for(uint8_t j = 0 ; j < 128 ; j++)
		{
			OLED_WriteData(0x00);
		}
	}
}


void OLED_Update(void)
{
	/*for(uint8_t i=0 ; i < 8 ; i++)
	{
		OLED_SetCursor(0, i);
		IIC_Start();
		IIC_send_Byte(0x78);  //通讯地址
		IIC_rec_ACK();
		IIC_send_Byte(0x40);  //控制位
		IIC_rec_ACK();
		for(uint8_t j=0 ; j < 128 ; j++)
		{
			IIC_send_Byte(OLED_DispalyBuf[i][j]);  //数据 
			IIC_rec_ACK();
		}
		IIC_Stop();
	}*/

	for(uint8_t i=0 ; i < 8 ; i++)
	{
		OLED_SetCursor(0, i);
		I2C_GenerateSTART(I2C2, ENABLE);	//起始位
		while(I2C_CheckEvent(I2C2 , I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS);
		
		I2C_Send7bitAddress(I2C2, 0x78, I2C_Direction_Transmitter); //发送地址
		while(I2C_CheckEvent(I2C2 , I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS);
		
		I2C_SendData(I2C2, 0x40);	//发送控制位
		while(I2C_CheckEvent(I2C2 , I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS);
		
		for(uint8_t j=0 ; j < 127 ; j++)
		{
			I2C_SendData(I2C2, OLED_DispalyBuf[i][j]); //发送数据
			while(I2C_CheckEvent(I2C2 , I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS);
			
		}
		/*最后一位由于标识符不同，所以发送需要拿出来*/
		I2C_SendData(I2C2, OLED_DispalyBuf[i][127]); //发送数据
		while(I2C_CheckEvent(I2C2 , I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS);
		
		I2C_GenerateSTOP(I2C2 , ENABLE);
	}
}

void OLED_ShowChar(int16_t X, int16_t Y, uint32_t Number)
{
	uint8_t Page = Y / 8;
	uint8_t Shift = Y % 8;
	for(uint8_t i = 0 ; i < 16 ; i++)
	{
		if(i/8 == 0) //上半部分
		{
			OLED_DispalyBuf[Page][X+(i%8)] = OLED_F8x16[Number][i]<<Shift;   // 第(Y/8)行的处理
			OLED_DispalyBuf[Page+1][X+(i%8)] = OLED_F8x16[Number][i]>>(8-Shift);   // 第(Y/8)+1行的处理
		}
		else if(i/8 == 1) //下半部分
		{
			OLED_DispalyBuf[Page+1][X+(i%8)] = (OLED_F8x16[Number][i]<<Shift) | (OLED_DispalyBuf[Page+1][X+(i%8)] & (0xff>>(8-Shift)));   // 第(Y/8)行的处理
			OLED_DispalyBuf[Page+2][X+(i%8)] = OLED_F8x16[Number][i]>>(8-Shift);   // 第(Y/8)+1行的处理
		}
	}
	
}


void OLED_ShowString(int16_t X, int16_t Y, char *String) //8*16-->宽*高
{
	char *P = String;
	while(*P != '\0')
	{
		OLED_ShowChar(X , Y , *P-32);
		P++;
		X += 8;
	}
}


void OLED_Printf(int16_t X, int16_t Y,const char *format, ...)
{
	char str[256] = {0};
	va_list arg;  //定义可变参数列表类型变量
	va_start(arg, format); //开始接收参数列表
	vsprintf(str , format , arg);  //将格式化字符串存储在str中
	va_end(arg); //清除arg
	OLED_ShowString(X , Y , str);
}


void OLED_AreaClear(int16_t X , int16_t Y , int16_t X1 , int16_t Y1)
{
	int16_t M1 = -1;
	int16_t M2 = -1;
	/*得到比左上角Y值大并且最近的8的倍数*/
	uint8_t i = 0;
	for(; i < 10 ; i++)
	{
		if(i*8 >= Y)
		{
			M1 = 8*i - Y;
			break;
		}
	}
	/*得到比右下角Y值大并且最近的8的倍数*/
	uint8_t j = 0;
	for(; j < 10 ; j++)
	{
		if(j*8 >= Y1)
		{
			M2 = 8-(8*j - Y1);
			break;
		}
	}
	//OLED_DispalyBuf[8][128]
	/*清除中间整页的像素点,分情况*/
	if(Y1%8 != 0)
	{
		for(uint8_t m = i ; m < j-1 ; m++)
		{
			for(uint8_t n = X ; n < X1 ; n++)
			{
				OLED_DispalyBuf[m][n] = 0x00;
			}
		}
	}
	else
	{
		for(uint8_t m = i ; m < j ; m++)
		{
			for(uint8_t n = X ; n < X1 ; n++)
			{
				OLED_DispalyBuf[m][n] = 0x00;
			}
		}
	}
	/*清除两边的像素点*/
	if(Y%8)  //上边
	{
		for(uint8_t m = X ; m < X1 ; m++)
		{
			OLED_DispalyBuf[i-1][m] = (~(0xff<<(8-M1))) & OLED_DispalyBuf[i-1][m];
		}
	}
	if(Y1%8) //下边
	{
		for(uint8_t m = X ; m < X1 ; m++)
		{
			uint8_t T = (uint8_t)(0xff<<M2);
			OLED_DispalyBuf[j-1][m] = T & OLED_DispalyBuf[j-1][m];
		}
	}
}



void OLED_AreaReversal(int16_t X , int16_t Y , int16_t X1 , int16_t Y1)  //局部反色显示
{
	int16_t M1 = -1;
	int16_t M2 = -1;
	/*得到比左上角Y值大并且最近的8的倍数*/
	uint8_t i = 0;
	for(; i < 10 ; i++)
	{
		if(i*8 >= Y)
		{
			M1 = 8*i - Y;
			break;
		}
	}
	/*得到比右下角Y值大并且最近的8的倍数*/
	uint8_t j = 0;
	for(; j < 10 ; j++)
	{
		if(j*8 >= Y1)
		{
			M2 = 8-(8*j - Y1);
			break;
		}
	}
	//OLED_DispalyBuf[8][128]
	/*清除中间整页的像素点,分情况*/
	if(Y1%8 != 0)
	{
		for(uint8_t m = i ; m < j-1 ; m++)
		{
			for(uint8_t n = X ; n < X1 ; n++)
			{
				OLED_DispalyBuf[m][n] = ~OLED_DispalyBuf[m][n];
			}
		}
	}
	else
	{
		for(uint8_t m = i ; m < j ; m++)
		{
			for(uint8_t n = X ; n < X1 ; n++)
			{
				OLED_DispalyBuf[m][n] = ~OLED_DispalyBuf[m][n];
			}
		}
	}
	/*清除两边的像素点*/
	if(Y%8)  //上边
	{
		for(uint8_t m = X ; m < X1 ; m++)
		{
			OLED_DispalyBuf[i-1][m] = (0xff<<(8-M1)) ^ OLED_DispalyBuf[i-1][m];
		}
	}
	if(Y1%8) //下边
	{
		for(uint8_t m = X ; m < X1 ; m++)
		{
			uint8_t T = (uint8_t)(0xff<<M2);
			OLED_DispalyBuf[j-1][m] = (~T) ^ OLED_DispalyBuf[j-1][m];
		}
	}
}





void OLED_AreaRefresh(int16_t X , int16_t Y , int16_t X1 , int16_t Y1)  //指定区域刷新显示,Y轴只能是8的倍数
{
	if(!(Y%8) && !(Y1%8))  //判断都是8的倍数
	{
		for(uint8_t i = Y/8 ; i < Y1/8 ; i++)
		{
			OLED_SetCursor(X , i);
			/*IIC_Start();
			IIC_send_Byte(0x78);  //通讯地址
			IIC_rec_ACK();
			IIC_send_Byte(0x40);  //控制位
			IIC_rec_ACK();
			for(uint8_t j=X ; j < X1 ; j++)
			{
				IIC_send_Byte(OLED_DispalyBuf[i][j]);  //数据 
				IIC_rec_ACK();
			}
			IIC_Stop();*/
			I2C_GenerateSTART(I2C2, ENABLE);	//起始位
			while(I2C_CheckEvent(I2C2 , I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS);
			
			I2C_Send7bitAddress(I2C2, 0x78, I2C_Direction_Transmitter); //发送地址
			while(I2C_CheckEvent(I2C2 , I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS);
			
			I2C_SendData(I2C2, 0x40);	//发送控制位
			while(I2C_CheckEvent(I2C2 , I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS);
			
			for(uint8_t j=X ; j < X1-1 ; j++)
			{
				I2C_SendData(I2C2, OLED_DispalyBuf[i][j]); //发送数据
				while(I2C_CheckEvent(I2C2 , I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS);
				
			}
			I2C_SendData(I2C2, OLED_DispalyBuf[i][X1-1]); //发送数据
			while(I2C_CheckEvent(I2C2 , I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS);
			
			I2C_GenerateSTOP(I2C2 , ENABLE);
		}
	}
	else
	{
		OLED_AreaReversal(0,0,127,63);
		OLED_Update();
		for(uint16_t i=0 ; i<9000 ; i++)
		{
			for(uint16_t j = 0 ; j < 900 ; j++)
			{
				__NOP();
			}
		}
		OLED_AreaReversal(0,0,127,63);
		OLED_Update();
	}
}

//OLED_DispalyBuf[8][128]
/*
显示图片，长宽只能为8的倍数，纵坐标也只能为8的倍数
*/
void OLED_ShowImage(uint8_t X , uint8_t Y , uint8_t Lenth , uint8_t With , char Str[])
{
	if(!(Lenth%8) && !(With%8) && !(Y%8)) //确保是8的倍数
	{
		for(uint8_t i = Y/8 ; i < (Y+With)/8 ; i++)
		{
			for(uint8_t j = X ; j < (X+Lenth) ; j++)
			{
				OLED_DispalyBuf[i][j] = Str[(j-X)+(i-Y/8)*Lenth];
			}
		}
	}
	else
	{
		OLED_AreaReversal(0,0,127,63);
		OLED_Update();
		for(uint16_t i=0 ; i<9000 ; i++)
		{
			for(uint16_t j = 0 ; j < 900 ; j++)
			{
				__NOP();
			}
		}
		OLED_AreaReversal(0,0,127,63);
		OLED_Update();
	}
}


