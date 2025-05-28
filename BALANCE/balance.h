#ifndef __BALANCE_H
#define __BALANCE_H			  	 
#include "sys.h"
#include "system.h"

#define BALANCE_TASK_PRIO		4     //Task priority //�������ȼ�
#define BALANCE_STK_SIZE 		512   //Task stack size //�����ջ��С

//Parameter of kinematics analysis of omnidirectional trolley
//ȫ����С���˶�ѧ��������
#define X_PARAMETER    (sqrt(3)/2.f)               
#define Y_PARAMETER    (0.5f)    
#define L_PARAMETER    (1.0f)

typedef struct  
{
	float U1,U2,U3;//������ѹ
	uint16_t angle;//ת�ӵ�Ƕ�
	uint8_t RotorRegion;//��������
	uint16_t dutyA,dutyB,dutyC;//����ռ�ձȣ���CCR�Ĵ���ֵ
	
	float FOC_freq; //FOC��Ƶ��
	
	float Encoder;     //Read the real time speed of the motor by encoder //��������ֵ����ȡ���ʵʱ�ٶ�
	
	
//	float Motor_Pwm;   //Motor PWM value, control the real-time speed of the motor //���PWM��ֵ�����Ƶ��ʵʱ�ٶ�
	float Target;      //Control the target speed of the motor //���Ŀ���ٶ�ֵ�����Ƶ��Ŀ���ٶ�
	float Velocity_KP; //Speed control PID parameters //�ٶȿ���PID����
	float	Velocity_KI; //Speed control PID parameters //�ٶȿ���PID����
}BrushlessMotor;

BrushlessMotor Motor_Left;
BrushlessMotor Motor_Right;

extern short test_num;
extern int robot_mode_check_flag;
extern u8 command_lost_count; //���ڡ�CAN�������ʧʱ���������ʧ1���ֹͣ����
void Balance_task(void *pvParameters);

//void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo);
void Set_Pwm(void);
void FOC_Init(void);
void FOC_duty_Update(BrushlessMotor* motor,float freq);


void Limit_Pwm(int amplitude);
float target_limit_float(float insert,float low,float high);
int target_limit_int(int insert,int low,int high);
u8 Turn_Off( int voltage);
u32 myabs(long int a);
int Incremental_PI_A (float Encoder,float Target);
int Incremental_PI_B (float Encoder,float Target);
int Incremental_PI_C (float Encoder,float Target);
int Incremental_PI_D (float Encoder,float Target);
void Get_RC(void);
//void Remote_Control(void);
void Drive_Motor(float Vx,float Vy,float Vz);
void Key(void);
void Get_Velocity_Form_Encoder(void);
void Smooth_control(float vx,float vy,float vz);
//void PS2_control(void);
float float_abs(float insert);
void robot_mode_check(void);

#endif  

