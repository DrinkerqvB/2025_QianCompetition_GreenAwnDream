#ifndef __SYSTEM_H
#define __SYSTEM_H

// Refer to all header files you need
//����������Ҫ�õ���ͷ�ļ�
#include "stm32f4xx.h"
//The associated header file for the peripheral 
//��������ͷ�ļ�

typedef enum{
	Modbus_Type_A,
	Modbus_Type_B,
	Modbus_Other_Error
}Modbus_Typedef;//����ͨѶЭ��

typedef struct{
	float line_error;      // Ѱ��ƫ�-3.5=����, +3.5=���ң�
    float speed_setpoint;  // Ŀ���ٶȣ�m/s��
    float turn_gain;       // ת��ϵ���������Ҹ���
}ControlState;

typedef struct  
{
	float U1,U2,U3;//������ѹ
	uint16_t angle;//ת�ӵ�Ƕ�
	uint8_t RotorRegion;//��������
	uint16_t dutyA,dutyB,dutyC;//����ռ�ձȣ���CCR�Ĵ���ֵ
	
	float FOC_freq; //FOC��Ƶ��
	 uint8_t dir;//0=CW, 1=CCW��������ƣ�
	float Encoder;     //Read the real time speed of the motor by encoder //��������ֵ����ȡ���ʵʱ�ٶ�
	
	float Target;      //Control the target speed of the motor //���Ŀ���ٶ�ֵ�����Ƶ��Ŀ���ٶ�
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
//С���ͺŵ�ö�ٶ���
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
//ƽ�������������ٶ�
typedef struct  
{
	float VX;
	float VY;
	float VZ;
}Smooth_Control;


/****** external variable definition. When system.h is referenced in other C files, 
        other C files can also use the variable defined by system.c           ******/
/****** �ⲿ�������壬������c�ļ�����system.hʱ��Ҳ����ʹ��system.c����ı��� ******/

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

/***Macros define***/ /***�궨��***/
//After starting the car (1000/100Hz =10) for seconds, it is allowed to control the car to move
//����(1000/100hz=10)�����������С�������˶�
#define CONTROL_DELAY		1000
//The number of robot types to determine the value of Divisor_Mode. There are currently 6 car types
//�������ͺ�����������Divisor_Mode��ֵ��Ŀǰ��6��С������
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
/***Macros define***/ /***�궨��***/

//C library function related header file
//C�⺯�������ͷ�ļ�
#include <stdio.h> 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stdarg.h"
#endif 
