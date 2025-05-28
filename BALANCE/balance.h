#ifndef __BALANCE_H
#define __BALANCE_H			  	 
#include "sys.h"
#include "system.h"

#define BALANCE_TASK_PRIO		4     //Task priority //任务优先级
#define BALANCE_STK_SIZE 		512   //Task stack size //任务堆栈大小

//Parameter of kinematics analysis of omnidirectional trolley
//全向轮小车运动学分析参数
#define X_PARAMETER    (sqrt(3)/2.f)               
#define Y_PARAMETER    (0.5f)    
#define L_PARAMETER    (1.0f)

typedef struct  
{
	float U1,U2,U3;//三个电压
	uint16_t angle;//转子电角度
	uint8_t RotorRegion;//所在扇区
	uint16_t dutyA,dutyB,dutyC;//三个占空比，即CCR寄存器值
	
	float FOC_freq; //FOC电频率
	
	float Encoder;     //Read the real time speed of the motor by encoder //编码器数值，读取电机实时速度
	
	
//	float Motor_Pwm;   //Motor PWM value, control the real-time speed of the motor //电机PWM数值，控制电机实时速度
	float Target;      //Control the target speed of the motor //电机目标速度值，控制电机目标速度
	float Velocity_KP; //Speed control PID parameters //速度控制PID参数
	float	Velocity_KI; //Speed control PID parameters //速度控制PID参数
}BrushlessMotor;

BrushlessMotor Motor_Left;
BrushlessMotor Motor_Right;

extern short test_num;
extern int robot_mode_check_flag;
extern u8 command_lost_count; //串口、CAN控制命令丢失时间计数，丢失1秒后停止控制
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

