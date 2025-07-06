#include "balance.h"

int Time_count=0; //Time variable //计时变量
u32 Buzzer_count1 = 0;
// Robot mode is wrong to detect flag bits
//机器人模式是否出错检测标志位
int robot_mode_check_flag=0; 

short test_num;

// 正弦表（预生成360点，幅值0-1）
static float SinTable[360];




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
		float amplitude=3.5; //Wheel target speed limit //车轮目标速度限幅
		
	
		if(Car_Mode==FourWheel_Car) 
		{	
			//Inverse kinematics //运动学逆解
			Motor_Left.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出左轮的目标速度
			Motor_Right.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出左轮的目标速度
//			MOTOR_C.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出右轮的目标速度
//			MOTOR_D.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出右轮的目标速度
					
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
			Motor_Left.Target=target_limit_float( Motor_Left.Target,-amplitude,amplitude); 
			Motor_Right.Target=target_limit_float( Motor_Right.Target,-amplitude,amplitude); 
//			MOTOR_C.Target=target_limit_float( MOTOR_C.Target,-amplitude,amplitude); 
//			MOTOR_D.Target=target_limit_float( MOTOR_D.Target,-amplitude,amplitude); 	
		}
		

}
/**************************************************************************
Function: FreerTOS task, core motion control task
Input   : none
Output  : none
函数功能：FreeRTOS任务，核心运动控制任务
入口参数：无
返回  值：无
**************************************************************************/
void Balance_task(void *pvParameters)
{ 	
	  
	ControlState ctrl;//把红外传输的数据粘贴
	u32 lastWakeTime = getSysTickCnt();
	float last_error = 0;//偏差

    while(1)
    {	
			
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ)); 
			
		
//			if(Time_count<3000)Time_count++;
//			Buzzer_count1++;
			
			Get_Velocity_Form_Encoder();   
		
//				Key();
			if(Check==0) 
			{
				if(xQueueReceive(xControlQueue, &ctrl, 0) == pdPASS) {
                    Move_X = ctrl.speed_setpoint;
                    Move_Z = ctrl.turn_gain;
                }
                Drive_Motor(Move_X, 0, Move_Z); // 只使用X(速度)和Z(转向)
            }
								

				//如果电池电压不存在异常，而且使能开关在ON档位，而且软件失能标志位为0
				
				 			
          
					Motor_Left.FOC_freq=Incremental_PID_A(Motor_Left.Encoder, Motor_Left.Target);
					Motor_Right.FOC_freq=Incremental_PID_B(Motor_Right.Encoder, Motor_Right.Target);
//				 	MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
//					MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);
						 
					 Limit_Pwm(16700);

		 }  
		 
}


/**************************************************************************
函数功能：用于FOC循环的FreeRTOS任务
入口参数：
返回  值：无
**************************************************************************/
void FOCLoop_task(void *pvParameters)
{
	u32 lastWakeTime = getSysTickCnt();
			
    while(1)
    {	
			// This task runs at a frequency of 100Hz (10ms control once)
			//此任务以1000Hz的频率运行（1ms控制一次）
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_1000_HZ)); 
	
			FOC_duty_Update(&Motor_Left, Motor_Left.FOC_freq);
			FOC_duty_Update(&Motor_Right, Motor_Right.FOC_freq);
			Set_Pwm();
		
	}

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
	 static float Phase_Left = 0, Phase_Right = 0; // 分离左右电机相位
		
	
	
	motor->dir = (freq >= 0) ? CW : CCW;//判断正反转
	
    float abs_freq = fabsf(freq);
    float *phase = (motor == &Motor_Left) ? &Phase_Left : &Phase_Right;
	
    // 计算相位增量（每1ms更新一次）
	* phase += 0.36f * freq; // 0.36 = 360° / 1000ms
	if (* phase >= 360.0f) * phase -= 360.0f;
	if (* phase <=0.0f) * phase += 360.0f;
        
    
    float theta = (motor->dir == CW) ? *phase : (359-*phase);
	
	
	
    // 生成三相正弦波
    float Ua = SinTable[(int)theta % 360];
    float Ub = SinTable[(int)(theta + 120) % 360];
    float Uc = SinTable[(int)(theta + 240) % 360];
    
    // 转换为PWM占空比（幅值设为50%）
    uint16_t DutyA = (uint16_t)((Ua + 1.0f) * PWM_PERIOD / 2);
    uint16_t DutyB = (uint16_t)((Ub + 1.0f) * PWM_PERIOD / 2);
    uint16_t DutyC = (uint16_t)((Uc + 1.0f) * PWM_PERIOD / 2);
    
	
	motor->dutyA=DutyA;
	motor->dutyB=DutyB;
	motor->dutyC=DutyC;
	
	
	
//	TIM1->CCR1 = (uint16_t)((Ua + 1.0f) * 0.4f * PWM_PERIOD + 0.1f * PWM_PERIOD);
//    TIM1->CCR2 = (uint16_t)((Ub + 1.0f) * 0.4f * PWM_PERIOD + 0.1f * PWM_PERIOD);
//    TIM1->CCR3 = (uint16_t)((Uc + 1.0f) * 0.4f * PWM_PERIOD + 0.1f * PWM_PERIOD);
	
	
	
    // 更新PWM寄存器
//    TIM1->CCR1 = DutyA;
//    TIM1->CCR2 = DutyB;
//    TIM1->CCR3 = DutyC;
}







/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
函数功能：限制电频率值 
入口参数：幅值
返回  值：无
**************************************************************************/
void Limit_Pwm(int amplitude)
{	
	Motor_Left.dutyA=target_limit_float(Motor_Left.dutyA,-amplitude,amplitude);
	Motor_Left.dutyB=target_limit_float(Motor_Left.dutyB,-amplitude,amplitude);
	Motor_Left.dutyC=target_limit_float(Motor_Left.dutyC,-amplitude,amplitude);
	
	Motor_Right.dutyA=target_limit_float(Motor_Left.dutyA,-amplitude,amplitude);
	Motor_Right.dutyB=target_limit_float(Motor_Left.dutyB,-amplitude,amplitude);
	Motor_Right.dutyC=target_limit_float(Motor_Left.dutyC,-amplitude,amplitude);
	    
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
Function: Check the battery voltage, enable switch status, software failure flag status
Input   : Voltage
Output  : Whether control is allowed, 1: not allowed, 0 allowed
函数功能：检查电池电压、使能开关状态、软件失能标志位状态
入口参数：电压
返回  值：是否允许控制，1：不允许，0允许
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<10||Flag_Stop==1)
			{	                                                
				temp=1;    
					TIM4->CCR1=0;
				TIM4->CCR2=0;
				TIM8->CCR1=0;
				TIM8->CCR2=0;
				TIM8->CCR3=0;
				TIM8->CCR4=0;
							
      }
			else
			temp=0;
			return temp;			
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
	 static float Bias,ElecFreq,Last_bias;
	 Bias=(Target-Encoder)*7/Wheel_perimeter; //Calculate the deviation //计算偏差
	
	 ElecFreq+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias+Velocity_KD*(Bias-Last_bias); 
	
	 if(ElecFreq>233)ElecFreq=233;
	 if(ElecFreq<-233)ElecFreq=-233;
	
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return ElecFreq;    
}
int Incremental_PID_B (float Encoder,float Target)
{  
	 static float Bias,ElecFreq,Last_bias;
	 Bias=(Target-Encoder)*7/Wheel_perimeter; //Calculate the deviation //计算偏差
	
	 ElecFreq+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias+Velocity_KD*(Bias-Last_bias);  
	
	 if(ElecFreq>233)ElecFreq=233;
	 if(ElecFreq<-233)ElecFreq=-233;
	
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return ElecFreq;
}
int Incremental_PI_C (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm; 
}
int Incremental_PI_D (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm; 
}
/**************************************************************************
Function: Processes the command sent by APP through usart 2
Input   : none
Output  : none
函数功能：对APP通过串口2发送过来的命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Get_RC(void)
{
	u8 Flag_Move=1;
	if(Car_Mode==Mec_Car||Car_Mode==Omni_Car) //The omnidirectional wheel moving trolley can move laterally //全向轮运动小车可以进行横向移动
	{
	 switch(Flag_Direction)  //Handle direction control commands //处理方向控制命令
	 { 
			case 1:      Move_X=RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 2:      Move_X=RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 3:      Move_X=0;      		     Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 4:      Move_X=-RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1;    break;
			case 5:      Move_X=-RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 6:      Move_X=-RC_Velocity;  	 Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 7:      Move_X=0;     	 		     Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 8:      Move_X=RC_Velocity; 	   Move_Y=RC_Velocity;   Flag_Move=1;    break; 
			default:     Move_X=0;               Move_Y=0;             Flag_Move=0;    break;
	 }
	 if(Flag_Move==0)		
	 {	
		 //If no direction control instruction is available, check the steering control status
		 //如果无方向控制指令，检查转向控制状态
		 if     (Flag_Left ==1)  Move_Z= PI/2*(RC_Velocity/500); //left rotation  //左自转  
		 else if(Flag_Right==1)  Move_Z=-PI/2*(RC_Velocity/500); //right rotation //右自转
		 else 		               Move_Z=0;                       //stop           //停止
	 }
	}	
	else //Non-omnidirectional moving trolley //非全向移动小车
	{
	 switch(Flag_Direction) //Handle direction control commands //处理方向控制命令
	 { 
			case 1:      Move_X=+RC_Velocity;  	 Move_Z=0;         break;
			case 2:      Move_X=+RC_Velocity;  	 Move_Z=-PI/2;   	 break;
			case 3:      Move_X=0;      				 Move_Z=-PI/2;   	 break;	 
			case 4:      Move_X=-RC_Velocity;  	 Move_Z=-PI/2;     break;		 
			case 5:      Move_X=-RC_Velocity;  	 Move_Z=0;         break;	 
			case 6:      Move_X=-RC_Velocity;  	 Move_Z=+PI/2;     break;	 
			case 7:      Move_X=0;     	 			 	 Move_Z=+PI/2;     break;
			case 8:      Move_X=+RC_Velocity; 	 Move_Z=+PI/2;     break; 
			default:     Move_X=0;               Move_Z=0;         break;
	 }
	 if     (Flag_Left ==1)  Move_Z= PI/2; //left rotation  //左自转 
	 else if(Flag_Right==1)  Move_Z=-PI/2; //right rotation //右自转	
	}
	
	//Z-axis data conversion //Z轴数据转化
	if(Car_Mode==Akm_Car)
	{
		//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		//阿克曼结构小车转换为前轮转向角度
		Move_Z=Move_Z*2/9; 
	}
	else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
	{
	  if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //差速控制原理系列需要此处理
		Move_Z=Move_Z*RC_Velocity/500;
	}		
	
	//Unit conversion, mm/s -> m/s
  //单位转换，mm/s -> m/s	
	Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z;
	
	//Control target value is obtained and kinematics analysis is performed
	//得到控制目标值，进行运动学分析
	Drive_Motor(Move_X,Move_Y,Move_Z);
}










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
	  //Retrieves the original data of the encoder
	  //获取编码器的原始数据
		float Encoder_A_pr,Encoder_B_pr; 
//		float Encoder_C_pr,Encoder_D_pr;
		OriginalEncoder.A=Read_Encoder(2);	
		OriginalEncoder.B=Read_Encoder(3);	
//		OriginalEncoder.C=Read_Encoder(4);	
//		OriginalEncoder.D=Read_Encoder(5);	

	//test_num=OriginalEncoder.B;

		Encoder_A_pr= OriginalEncoder.A; 
		Encoder_B_pr= OriginalEncoder.B; 
//	Encoder_C_pr=-OriginalEncoder.C;  
//	Encoder_D_pr=-OriginalEncoder.D; 
//				break; 
//			case Tank_Car:      Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; 
//		}
		
		//The encoder converts the raw data to wheel speed in m/s
		//编码器原始数据转换为车轮速度，单位m/s
		Motor_Left.Encoder= Encoder_A_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		Motor_Right.Encoder= Encoder_B_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
//		MOTOR_C.Encoder= Encoder_C_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
//		MOTOR_D.Encoder= Encoder_D_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
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

