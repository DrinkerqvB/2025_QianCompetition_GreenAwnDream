#ifndef __SYSTEM_H
#define __SYSTEM_H

// Refer to all header files you need
//引用所有需要用到的头文件
#include "stm32f4xx.h"
//The associated header file for the peripheral 
//外设的相关头文件

typedef enum{
	Modbus_Type_A,
	Modbus_Type_B,
	Modbus_Other_Error
}Modbus_Typedef;//两种通讯协议

typedef struct{
	float line_error;      // 寻迹偏差（-3.5=最左, +3.5=最右）
    float speed_setpoint;  // 目标速度（m/s）
    float turn_gain;       // 转向系数（左正右负）
}ControlState;

typedef struct  
{
	float U1,U2,U3;//三个电压
	uint16_t angle;//转子电角度
	uint8_t RotorRegion;//所在扇区
	uint16_t dutyA,dutyB,dutyC;//三个占空比，即CCR寄存器值
	
	float FOC_freq; //FOC电频率
	 uint8_t dir;//0=CW, 1=CCW（方向控制）
	float Encoder;     //Read the real time speed of the motor by encoder //编码器数值，读取电机实时速度
	
	float Target;      //Control the target speed of the motor //电机目标速度值，控制电机目标速度
}BrushlessMotor;


#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "balance.h"
#include "usart.h"
#include "usartx.h"
#include "encoder.h"
#include "robot_select_init.h"
#include "tracking.h"
#include "brushlessMotor.h"
#include "ESP8266.h"
#include "RGB.h"
#include "Timer.h"
#include "OLED_Data.h"
#include "OLED.h"
#include "Key.h"
#include "Modbus.h"

// Enumeration of car types
//小车型号的枚举定义
typedef enum 
{
	Mec_Car = 0, 
	Omni_Car, 
	Akm_Car, 
	Diff_Car, 
	FourWheel_Car, 
	Tank_Car
} CarMode;


//Smoothed the speed of the three axes
//平滑处理后的三轴速度
typedef struct  
{
	float VX;
	float VY;
	float VZ;
}Smooth_Control;


/****** external variable definition. When system.h is referenced in other C files, 
        other C files can also use the variable defined by system.c           ******/
/****** 外部变量定义，当其它c文件引用system.h时，也可以使用system.c定义的变量 ******/

extern float Move_X, Move_Y, Move_Z; 
extern float Velocity_KP, Velocity_KI, Velocity_KD;	
extern Smooth_Control smooth_control;

extern BrushlessMotor Motor_Left,Motor_Right;
extern float Encoder_precision;
extern float Wheel_perimeter;
extern float Wheel_spacing; 
extern float Axle_spacing; 
extern float Omni_turn_radiaus; 

extern u8 uart2_receive_message[50];
extern u8 uart2_send_flag;

void systemInit(void);

/***Macros define***/ /***宏定义***/
//After starting the car (1000/100Hz =10) for seconds, it is allowed to control the car to move
//开机(1000/100hz=10)秒后才允许控制小车进行运动
#define CONTROL_DELAY		1000
//The number of robot types to determine the value of Divisor_Mode. There are currently 6 car types
//机器人型号数量，决定Divisor_Mode的值，目前有6种小车类型
#define CAR_NUMBER    6      
#define RATE_1_HZ		  1
#define RATE_5_HZ		  5
#define RATE_10_HZ		10
#define RATE_20_HZ		20
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000
/***Macros define***/ /***宏定义***/

//C library function related header file
//C库函数的相关头文件
#include <stdio.h> 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stdarg.h"
#endif 
