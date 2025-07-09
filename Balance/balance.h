#ifndef __BALANCE_H
#define __BALANCE_H			  	 
#include "sys.h"
#include "system.h"

#define X_PARAMETER    (sqrt(3)/2.f)               
#define Y_PARAMETER    (0.5f)    
#define L_PARAMETER    (1.0f)
typedef enum {
   CW = 0,  // À≥ ±’Î
   CCW = 1  // ƒÊ ±’Î
} MotorDirection;


void Balance_task(void);

void FOCLoop_task(void);
void Set_Pwm(void);
void FOC_Init(void);
void FOC_duty_Update(BrushlessMotor* motor,float freq);


void Limit_Freq(int amplitude);
float target_limit_float(float insert,float low,float high);
int target_limit_int(int insert,int low,int high);

u32 myabs(long int a);
int Incremental_PID_Left (float Encoder,float Target);
int Incremental_PID_Right (float Encoder,float Target);

void Drive_Motor(float Vx,float Vy,float Vz);

void Get_Velocity_Form_Encoder(void);
void Smooth_control(float vx,float vy,float vz);

float float_abs(float insert);


#endif  

