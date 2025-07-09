#include "system.h"

//Vehicle three-axis target moving speed, unit: m/s
//小车三轴目标运动速度，单位：m/s
float Move_X, Move_Y, Move_Z;   

//PID parameters of Speed control
//速度控制PID参数
float Velocity_KP=1.5f;
float Velocity_KI=0.06f;
float Velocity_KD=0.2; 

//Smooth control of intermediate variables, dedicated to omni-directional moving cars
//平滑控制中间变量
Smooth_Control smooth_control;  

//The parameter structure of the motor
//电机的参数结构体
BrushlessMotor Motor_Left, Motor_Right;


/************ 小车型号相关变量 **************************/
/************ Variables related to car model ************/
//Encoder accuracy
//编码器精度
float Encoder_precision; 
//Wheel circumference, unit: m
//轮子周长，单位：m
float Wheel_perimeter; 
//Drive wheel base, unit: m
//主动轮轮距，单位：m
float Wheel_spacing; 
//The wheelbase of the front and rear axles of the trolley, unit: m
//小车前后轴的轴距，单位：m
float Axle_spacing; 
//All-directional wheel turning radius, unit: m
//全向轮转弯半径，单位：m
float Omni_turn_radiaus; 


void systemInit(void)
{       
	
	//中断优先级分组设置
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	//RGB三色灯控制引脚初始化
	RGB_Init();
	
	//KEY0 按钮初始化，用于ESP8266重连
	Key0_Init();
	
	//串口1初始化，通信波特率115200，可用于Modbus通信
	Modbus_Init();
	
	uart2_init(9600);  
	
	
	//串口3初始化，通信波特率115200
	//ESP8266 联网初始化 ，使用USART3
	uart3_init(115200);
//	ESP8266_Init();

	//初始化与OLED显示屏连接的硬件接口	
	OLED_init();
	
	
	//八路循迹初始化，使用I2C1
	Tracking_Init();
	
	
  //进行小车对应的参数初始化	
	Robot_Select();                 
	
	 //Encoder A is initialized to read the real time speed of motor C  
  //左轮编码器初始化，用于读取Motor_Left的实时速度	
	 Encoder_Init_TIM2();
	//Encoder B is initialized to read the real time speed of motor D
  //右轮编码器初始化，用于读取Motor_Right的实时速度	
	  Encoder_Init_TIM3();   
	
	//生成正弦表
	 FOC_Init();
	 //初始化电机所需定时器
	 TIM7_Init();  //FOC更新定时器
	 //输出信号波形所需定时器
	 TIM4_Init();
	 TIM8_Init();
						
}
