#include "balance.h"

int Time_count=0; //Time variable //��ʱ����
u32 Buzzer_count1 = 0;
// Robot mode is wrong to detect flag bits
//������ģʽ�Ƿ�������־λ
int robot_mode_check_flag=0; 

short test_num;

// ���ұ�Ԥ����360�㣬��ֵ0-1��
static float SinTable[360];



ControlState cmd;
Encoder OriginalEncoder; //Encoder raw data //������ԭʼ����

u8 command_lost_count=0; //���ڡ�CAN�������ʧʱ���������ʧ1���ֹͣ����
/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
�������ܣ��˶�ѧ��⣬��������Ŀ���ٶȼ��������Ŀ��ת��
��ڲ�����X��Y��Z�᷽���Ŀ���˶��ٶ�
����  ֵ����
**************************************************************************/
void Drive_Motor(float Vx,float Vy,float Vz)
{
		float amplitude=6.0f; //Wheel target speed limit //����Ŀ���ٶ��޷�
	
			//Inverse kinematics //�˶�ѧ���
			Motor_Left.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //��������ֵ�Ŀ���ٶ�
			Motor_Right.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //��������ֵ�Ŀ���ٶ�

					
			//Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
			Motor_Left.Target=target_limit_float( Motor_Left.Target,-amplitude,amplitude); 
			Motor_Right.Target=target_limit_float( Motor_Right.Target,-amplitude,amplitude); 
		

}
/**************************************************************************
Function: FreerTOS task, core motion control task
Input   : none
Output  : none
�������ܣ�FreeRTOS���񣬺����˶���������
��ڲ�������
����  ֵ����
**************************************************************************/
void Balance_task(void)
{ 	
	  
	//ControlState ctrl;//�Ѻ��⴫�������ճ��
	//float last_error = 0;//ƫ��


		//Get_Velocity_Form_Encoder();
			
				Move_X = cmd.speed_setpoint;
				Move_Z = cmd.turn_gain;
                
		Drive_Motor(Move_X, 0, Move_Z); // ֻʹ��X(�ٶ�)��Z(ת��)
 
//			Motor_Left.FOC_freq=Incremental_PID_Left(Motor_Left.Encoder, Motor_Left.Target);
//			Motor_Right.FOC_freq=Incremental_PID_Right(Motor_Right.Encoder, Motor_Right.Target);
		Motor_Left.FOC_freq=Motor_Left.Target / Wheel_perimeter * 7;
		Motor_Right.FOC_freq=Motor_Right.Target / Wheel_perimeter * 7;
	
//			Motor_Left.FOC_freq=Incremental_PID_left(Motor_Left.Encoder, 0.5f);
//			Motor_Right.FOC_freq=Incremental_PID_Right(Motor_Right.Encoder, 0.5f);
	
//	printf("%f,%f,%f,%f\n",Motor_Left.Encoder,Motor_Left.Target,Motor_Right.Encoder,Motor_Right.Target);
	
//	Motor_Right.FOC_freq=Incremental_PID_Right(Motor_Right.Encoder, 0.5);
//#define TEST_FOCFREQ 25.0f
//		Motor_Left.FOC_freq=TEST_FOCFREQ;
//		Motor_Right.FOC_freq=TEST_FOCFREQ;
		
		printf("%f,%f,%f,%f,%f\n",Motor_Left.Encoder,Motor_Left.Target,Motor_Right.Encoder,Motor_Right.Target , cmd.turn_gain);
				Limit_Freq(50);
	 
}


/**************************************************************************
�������ܣ�����FOCѭ����FreeRTOS����
��ڲ�����
����  ֵ����
**************************************************************************/
void FOCLoop_task(void)
{
	
			// This task runs at a frequency of 100Hz (10ms control once)
			//��������1000Hz��Ƶ�����У�1ms����һ�Σ�
		
			FOC_duty_Update(&Motor_Left, Motor_Left.FOC_freq);
			FOC_duty_Update(&Motor_Right, Motor_Right.FOC_freq);
			Set_Pwm();

}

/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and direction
Input   : PWM
Output  : none
�������ܣ���ֵ��PWM�Ĵ��������Ƴ���ת���뷽��
��ڲ�����PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(void)
{
	TIM_SetCompare1(TIM4, Motor_Left.dutyA); // CH1 = U
    TIM_SetCompare2(TIM4, Motor_Left.dutyB); // CH2 = V
    TIM_SetCompare4(TIM8, Motor_Left.dutyC); // CH3 = W
	
	TIM_SetCompare1(TIM8, Motor_Right.dutyA); // CH1 = U
    TIM_SetCompare2(TIM8, Motor_Right.dutyB); // CH2 = V
    TIM_SetCompare3(TIM8, Motor_Right.dutyC); // CH3 = W
}




/**************************************************************************
�������ܣ�FOC�����㷨
��ڲ�����
����  ֵ��
**************************************************************************/
void FOC_Init(void) {
    // �������ұ�
    for (int i = 0; i < 360; i++) {
        SinTable[i] = sinf(i * 3.1415926f / 180.0f);
    }
}

// ���¿���FOC�����freq: ��Ƶ��Hz��
void FOC_duty_Update(BrushlessMotor* motor,float freq) {
	
	// �������ұ��С
#define SIN_TABLE_SIZE 360


    // ÿ1ms���õ������ڼ�����ת�Ƕ�
    static float Phase_Left = 0.0f;
    static float Phase_Right = 0.0f;
    
    // �жϵ������ (��תʱfreq>0����תʱfreq<0)
    motor->dir = (freq >= 0.0f) ? CW : CCW;
    
    // ������λ������ÿ1msִ��1�Σ�
    float phase_inc = 360.0f / SWITCHFREQ * fabsf(freq); // 0.36 = 360��/1000ms
    
    // ��ȡ��ǰ�������λָ��
    float *phase_ptr = (motor == &Motor_Left) ? &Phase_Left : &Phase_Right;
    
    // ������λֵ����������ת��
    if (motor->dir == CW) {
        *phase_ptr += phase_inc;  // ��ת����λ����
    } else {
        *phase_ptr -= phase_inc;  // ��ת����λ����
    }
    
    // ��λ��һ����0~360��
    *phase_ptr = fmodf(*phase_ptr, 360.0f);
    if (*phase_ptr < 0.0f) {
        *phase_ptr += 360.0f;
    }
    
    // ��תʱ���ɷ������Ҳ�����ȷ�ķ�ʽ��
    float theta = *phase_ptr;
    float inversion_factor = (motor->dir == CW) ? 1.0f : -1.0f;
    
    // �����������Ҳ���ʹ�ò�ֵ��߾��ȣ�
    float theta1 = fmodf(theta, SIN_TABLE_SIZE);
    int idx0 = (int)theta1;
    int idx1 = (idx0 + 1) % SIN_TABLE_SIZE;
    float frac = theta1 - idx0;
    
    float Ua = (SinTable[idx0] + frac * (SinTable[idx1] - SinTable[idx0])) * inversion_factor;
    
    idx0 = (int)fmodf(theta + 120.0f, SIN_TABLE_SIZE);
    idx1 = (idx0 + 1) % SIN_TABLE_SIZE;
    frac = (theta + 120.0f) - idx0;
    
    float Ub = (SinTable[idx0] + frac * (SinTable[idx1] - SinTable[idx0])) * inversion_factor;
    
    idx0 = (int)fmodf(theta + 240.0f, SIN_TABLE_SIZE);
    idx1 = (idx0 + 1) % SIN_TABLE_SIZE;
    frac = (theta + 240.0f) - idx0;
    
    float Uc = (SinTable[idx0] + frac * (SinTable[idx1] - SinTable[idx0])) * inversion_factor;
    
//    // ת����PWMռ�ձȣ���ѹ���ĵ�ƫ�ƣ�
//    float V_offset = 0.5f;
    
    // �޷�����
    Ua = fmaxf(-1.0f, fminf(Ua, 1.0f));
    Ub = fmaxf(-1.0f, fminf(Ub, 1.0f));
    Uc = fmaxf(-1.0f, fminf(Uc, 1.0f));
    
    // ת��ΪPWMռ�ձȣ���Χ��0��PWM_PERIOD��
    uint16_t DutyA = (uint16_t)((Ua * 0.5f + 0.5f) * PWM_PERIOD);
    uint16_t DutyB = (uint16_t)((Ub * 0.5f + 0.5f) * PWM_PERIOD);
    uint16_t DutyC = (uint16_t)((Uc * 0.5f + 0.5f) * PWM_PERIOD);
    
    // д�뵽������ƽṹ
    motor->dutyA = DutyA;
    motor->dutyB = DutyB;
    motor->dutyC = DutyC;
}
	








/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
�������ܣ����Ƶ�Ƶ��ֵ 
��ڲ�������ֵ
����  ֵ����
**************************************************************************/
void Limit_Freq(int amplitude)
{	
	Motor_Left.FOC_freq=target_limit_float(Motor_Left.FOC_freq,-amplitude,amplitude);
	
	Motor_Right.FOC_freq=target_limit_float(Motor_Right.FOC_freq,-amplitude,amplitude);
  
}	    
/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
�������ܣ��޷�����
��ڲ�������ֵ
����  ֵ����
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}

/**************************************************************************
Function: Calculate absolute value
Input   : long int
Output  : unsigned int
�������ܣ������ֵ
��ڲ�����long int
����  ֵ��unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
Function: Incremental PI controller
Input   : Encoder measured value (actual speed), target speed
Output  : Motor PWM
According to the incremental discrete PID formula
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)

�������ܣ�����ʽPI������
��ڲ���������������ֵ(ʵ���ٶ�)��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PID_A (float Encoder,float Target)
{ 	
	 static float Bias,ElecFreq=0,Last_bias;
	 Bias=(Target-Encoder)*7/Wheel_perimeter; //Calculate the deviation //����ƫ��
	
	 ElecFreq+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias+Velocity_KD*(Bias-Last_bias); 
	
//	 if(ElecFreq>200)ElecFreq=200;
//	 if(ElecFreq<-200)ElecFreq=-200;
	
	 Last_bias=Bias; //Save the last deviation //������һ��ƫ�� 
	 return ElecFreq;    
}
int Incremental_PID_B (float Encoder,float Target)
{  
	 static float Bias,ElecFreq=0,Last_bias;
	
	 Bias=(Target-Encoder)*7/Wheel_perimeter; //Calculate the deviation //����ƫ��
	
	 ElecFreq+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias+Velocity_KD*(Bias-Last_bias);  
	
//	 if(ElecFreq>200)ElecFreq=200;
//	 if(ElecFreq<-200)ElecFreq=-200;
	
	 Last_bias=Bias; //Save the last deviation //������һ��ƫ�� 
	 return ElecFreq;
}
//int Incremental_PI_C (float Encoder,float Target)
//{  
//	 static float Bias,Pwm,Last_bias;
//	 Bias=Target-Encoder; //Calculate the deviation //����ƫ��
//	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
//	 if(Pwm>16700)Pwm=16700;
//	 if(Pwm<-16700)Pwm=-16700;
//	 Last_bias=Bias; //Save the last deviation //������һ��ƫ�� 
//	 return Pwm; 
//}
//int Incremental_PI_D (float Encoder,float Target)
//{  
//	 static float Bias,Pwm,Last_bias;
//	 Bias=Target-Encoder; //Calculate the deviation //����ƫ��
//	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
//	 if(Pwm>16700)Pwm=16700;
//	 if(Pwm<-16700)Pwm=-16700;
//	 Last_bias=Bias; //Save the last deviation //������һ��ƫ�� 
//	 return Pwm; 
//}


/**************************************************************************
Function: Read the encoder value and calculate the wheel speed, unit m/s
Input   : none
Output  : none
�������ܣ���ȡ��������ֵ�����㳵���ٶȣ���λm/s
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_Velocity_Form_Encoder(void)
{
//	static uint16_t last_time=0,now_time=0;
//	uint8_t bias=(((TIM7->ARR)+1)*CONTROL_FREQUENCY*((TIM7->PSC)+1))/84000000;
//	now_time =TIM7->CNT;
//	if(now_time-last_time<bias && now_time>last_time){
//		return;
//	}
//	last_time = now_time;
	
	  //Retrieves the original data of the encoder
	  //��ȡ��������ԭʼ����
		float Encoder_A_pr,Encoder_B_pr; 
//		float Encoder_C_pr,Encoder_D_pr;
		OriginalEncoder.A=Read_Encoder(2);	
		OriginalEncoder.B=Read_Encoder(3);	
		

		Encoder_A_pr= OriginalEncoder.A - ENCODER_TIM_PERIOD/2; 
		Encoder_B_pr= OriginalEncoder.B - ENCODER_TIM_PERIOD/2; 

		
		//The encoder converts the raw data to wheel speed in m/s
		//������ԭʼ����ת��Ϊ�����ٶȣ���λm/s
		Motor_Left.Encoder= Encoder_A_pr*SWITCHFREQ*Wheel_perimeter/Encoder_precision;  
		Motor_Right.Encoder= Encoder_B_pr*SWITCHFREQ*Wheel_perimeter/Encoder_precision;  

		//�����ã���ֵΪ��Ƶ��
//		Motor_Left.Encoder= Encoder_A_pr*SWITCHFREQ/Encoder_precision*7;  
//		Motor_Right.Encoder= Encoder_B_pr*SWITCHFREQ/Encoder_precision*7;  

}











/**************************************************************************
Function: Smoothing the three axis target velocity
Input   : Three-axis target velocity
Output  : none
�������ܣ�������Ŀ���ٶ���ƽ������
��ڲ���������Ŀ���ٶ�
����  ֵ����
**************************************************************************/
void Smooth_control(float vx,float vy,float vz)
{
	float step=0.01;

	if	   (vx>0) 	smooth_control.VX+=step;
	else if(vx<0)		smooth_control.VX-=step;
	else if(vx==0)	smooth_control.VX=smooth_control.VX*0.9f;
	
	if	   (vy>0)   smooth_control.VY+=step;
	else if(vy<0)		smooth_control.VY-=step;
	else if(vy==0)	smooth_control.VY=smooth_control.VY*0.9f;
	
	if	   (vz>0) 	smooth_control.VZ+=step;
	else if(vz<0)		smooth_control.VZ-=step;
	else if(vz==0)	smooth_control.VZ=smooth_control.VZ*0.9f;
	
	smooth_control.VX=target_limit_float(smooth_control.VX,-float_abs(vx),float_abs(vx));
	smooth_control.VY=target_limit_float(smooth_control.VY,-float_abs(vy),float_abs(vy));
	smooth_control.VZ=target_limit_float(smooth_control.VZ,-float_abs(vz),float_abs(vz));
}
/**************************************************************************
Function: Floating-point data calculates the absolute value
Input   : float
Output  : The absolute value of the input number
�������ܣ����������ݼ������ֵ
��ڲ�����������
����  ֵ���������ľ���ֵ
**************************************************************************/
float float_abs(float insert)
{
	if(insert>=0) return insert;
	else return -insert;
}

