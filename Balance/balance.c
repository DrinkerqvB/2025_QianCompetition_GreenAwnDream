#include "balance.h"

int Time_count=0; //Time variable //计时变量
u32 Buzzer_count1 = 0;
// Robot mode is wrong to detect flag bits
//机器人模式是否出错检测标志位
int robot_mode_check_flag=0; 

short test_num;

// 正弦表（预生成360点，幅值0-1）
static float SinTable[360];



ControlState cmd;
Encoder OriginalEncoder; //Encoder raw data //编码器原始数据

u8 command_lost_count=0; //串口、CAN控制命令丢失时间计数，丢失1秒后停止控制
/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
函数功能：运动学逆解，根据三轴目标速度计算各车轮目标转速
入口参数：X和Y、Z轴方向的目标运动速度
返回  值：无
**************************************************************************/
void Drive_Motor(float Vx,float Vy,float Vz)
{
		float amplitude=6.0f; //Wheel target speed limit //车轮目标速度限幅
	
			//Inverse kinematics //运动学逆解
			Motor_Left.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出左轮的目标速度
			Motor_Right.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出左轮的目标速度

					
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
			Motor_Left.Target=target_limit_float( Motor_Left.Target,-amplitude,amplitude); 
			Motor_Right.Target=target_limit_float( Motor_Right.Target,-amplitude,amplitude); 
		

}
/**************************************************************************
Function: FreerTOS task, core motion control task
Input   : none
Output  : none
函数功能：FreeRTOS任务，核心运动控制任务
入口参数：无
返回  值：无
**************************************************************************/
void Balance_task(void)
{ 	
	  
	//ControlState ctrl;//把红外传输的数据粘贴
	//float last_error = 0;//偏差


		//Get_Velocity_Form_Encoder();
			
				Move_X = cmd.speed_setpoint;
				Move_Z = cmd.turn_gain;
                
		Drive_Motor(Move_X, 0, Move_Z); // 只使用X(速度)和Z(转向)
 
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
函数功能：用于FOC循环的FreeRTOS任务
入口参数：
返回  值：无
**************************************************************************/
void FOCLoop_task(void)
{
	
			// This task runs at a frequency of 100Hz (10ms control once)
			//此任务以1000Hz的频率运行（1ms控制一次）
		
			FOC_duty_Update(&Motor_Left, Motor_Left.FOC_freq);
			FOC_duty_Update(&Motor_Right, Motor_Right.FOC_freq);
			Set_Pwm();

}

/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and direction
Input   : PWM
Output  : none
函数功能：赋值给PWM寄存器，控制车轮转速与方向
入口参数：PWM
返回  值：无
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
函数功能：FOC核心算法
入口参数：
返回  值：
**************************************************************************/
void FOC_Init(void) {
    // 生成正弦表
    for (int i = 0; i < 360; i++) {
        SinTable[i] = sinf(i * 3.1415926f / 180.0f);
    }
}

// 更新开环FOC输出（freq: 电频率Hz）
void FOC_duty_Update(BrushlessMotor* motor,float freq) {
	
	// 定义正弦表大小
#define SIN_TABLE_SIZE 360


    // 每1ms调用的周期内计算旋转角度
    static float Phase_Left = 0.0f;
    static float Phase_Right = 0.0f;
    
    // 判断电机方向 (正转时freq>0，反转时freq<0)
    motor->dir = (freq >= 0.0f) ? CW : CCW;
    
    // 计算相位增量（每1ms执行1次）
    float phase_inc = 360.0f / SWITCHFREQ * fabsf(freq); // 0.36 = 360°/1000ms
    
    // 获取当前电机的相位指针
    float *phase_ptr = (motor == &Motor_Left) ? &Phase_Left : &Phase_Right;
    
    // 更新相位值（区分正反转）
    if (motor->dir == CW) {
        *phase_ptr += phase_inc;  // 正转：相位增加
    } else {
        *phase_ptr -= phase_inc;  // 反转：相位减少
    }
    
    // 相位归一化到0~360°
    *phase_ptr = fmodf(*phase_ptr, 360.0f);
    if (*phase_ptr < 0.0f) {
        *phase_ptr += 360.0f;
    }
    
    // 反转时生成反相正弦波（正确的方式）
    float theta = *phase_ptr;
    float inversion_factor = (motor->dir == CW) ? 1.0f : -1.0f;
    
    // 生成三相正弦波（使用插值提高精度）
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
    
//    // 转换到PWM占空比（电压中心点偏移）
//    float V_offset = 0.5f;
    
    // 限幅保护
    Ua = fmaxf(-1.0f, fminf(Ua, 1.0f));
    Ub = fmaxf(-1.0f, fminf(Ub, 1.0f));
    Uc = fmaxf(-1.0f, fminf(Uc, 1.0f));
    
    // 转换为PWM占空比（范围：0到PWM_PERIOD）
    uint16_t DutyA = (uint16_t)((Ua * 0.5f + 0.5f) * PWM_PERIOD);
    uint16_t DutyB = (uint16_t)((Ub * 0.5f + 0.5f) * PWM_PERIOD);
    uint16_t DutyC = (uint16_t)((Uc * 0.5f + 0.5f) * PWM_PERIOD);
    
    // 写入到电机控制结构
    motor->dutyA = DutyA;
    motor->dutyB = DutyB;
    motor->dutyC = DutyC;
}
	








/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
函数功能：限制电频率值 
入口参数：幅值
返回  值：无
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
函数功能：限幅函数
入口参数：幅值
返回  值：无
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
函数功能：求绝对值
入口参数：long int
返回  值：unsigned int
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
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)

函数功能：增量式PI控制器
入口参数：编码器测量值(实际速度)，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PID_A (float Encoder,float Target)
{ 	
	 static float Bias,ElecFreq=0,Last_bias;
	 Bias=(Target-Encoder)*7/Wheel_perimeter; //Calculate the deviation //计算偏差
	
	 ElecFreq+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias+Velocity_KD*(Bias-Last_bias); 
	
//	 if(ElecFreq>200)ElecFreq=200;
//	 if(ElecFreq<-200)ElecFreq=-200;
	
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return ElecFreq;    
}
int Incremental_PID_B (float Encoder,float Target)
{  
	 static float Bias,ElecFreq=0,Last_bias;
	
	 Bias=(Target-Encoder)*7/Wheel_perimeter; //Calculate the deviation //计算偏差
	
	 ElecFreq+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias+Velocity_KD*(Bias-Last_bias);  
	
//	 if(ElecFreq>200)ElecFreq=200;
//	 if(ElecFreq<-200)ElecFreq=-200;
	
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return ElecFreq;
}
//int Incremental_PI_C (float Encoder,float Target)
//{  
//	 static float Bias,Pwm,Last_bias;
//	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
//	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
//	 if(Pwm>16700)Pwm=16700;
//	 if(Pwm<-16700)Pwm=-16700;
//	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
//	 return Pwm; 
//}
//int Incremental_PI_D (float Encoder,float Target)
//{  
//	 static float Bias,Pwm,Last_bias;
//	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
//	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
//	 if(Pwm>16700)Pwm=16700;
//	 if(Pwm<-16700)Pwm=-16700;
//	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
//	 return Pwm; 
//}


/**************************************************************************
Function: Read the encoder value and calculate the wheel speed, unit m/s
Input   : none
Output  : none
函数功能：读取编码器数值并计算车轮速度，单位m/s
入口参数：无
返回  值：无
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
	  //获取编码器的原始数据
		float Encoder_A_pr,Encoder_B_pr; 
//		float Encoder_C_pr,Encoder_D_pr;
		OriginalEncoder.A=Read_Encoder(2);	
		OriginalEncoder.B=Read_Encoder(3);	
		

		Encoder_A_pr= OriginalEncoder.A - ENCODER_TIM_PERIOD/2; 
		Encoder_B_pr= OriginalEncoder.B - ENCODER_TIM_PERIOD/2; 

		
		//The encoder converts the raw data to wheel speed in m/s
		//编码器原始数据转换为车轮速度，单位m/s
		Motor_Left.Encoder= Encoder_A_pr*SWITCHFREQ*Wheel_perimeter/Encoder_precision;  
		Motor_Right.Encoder= Encoder_B_pr*SWITCHFREQ*Wheel_perimeter/Encoder_precision;  

		//调试用，此值为电频率
//		Motor_Left.Encoder= Encoder_A_pr*SWITCHFREQ/Encoder_precision*7;  
//		Motor_Right.Encoder= Encoder_B_pr*SWITCHFREQ/Encoder_precision*7;  

}











/**************************************************************************
Function: Smoothing the three axis target velocity
Input   : Three-axis target velocity
Output  : none
函数功能：对三轴目标速度做平滑处理
入口参数：三轴目标速度
返回  值：无
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
函数功能：浮点型数据计算绝对值
入口参数：浮点数
返回  值：输入数的绝对值
**************************************************************************/
float float_abs(float insert)
{
	if(insert>=0) return insert;
	else return -insert;
}

