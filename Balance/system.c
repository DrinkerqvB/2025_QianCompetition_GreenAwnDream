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
//平滑控制中间变量，全向移动小车专用
Smooth_Control smooth_control;  

//The parameter structure of the motor
//电机的参数结构体
BrushlessMotor Motor_Left, Motor_Right;
//BrushlessMotor MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D;  

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
/************ 小车型号相关变量 **************************/
/************ Variables related to car model ************/

//PS2 controller, Bluetooth APP, aircraft model controller, CAN communication, serial port 1, serial port 5 communication control flag bit.
//These 6 flag bits are all 0 by default, representing the serial port 3 control mode
//PS2手柄、蓝牙APP、航模手柄、CAN通信、串口1、串口5通信控制标志位。这6个标志位默认都为0，代表串口3控制模式
u8 APP_ON_Flag=0, Usart1_ON_Flag, Usart5_ON_Flag; 



//舵机计数值
int servo_direction[6] = {0};
int servo_pwm_count = 500;

int check_time_count_motor_forward=300,check_time_count_motor_retreat=500;
int POT_val;

int Servo_Count[6] = {500, 500, 500, 500, 500, 500};

u8 uart3_receive_message[50];
u8 uart3_send_flag;
u8 message_count=0;

u8 uart2_receive_message[50];
u8 uart2_send_flag=5;
u8 app_count=0;

int Full_rotation = 16799;

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
