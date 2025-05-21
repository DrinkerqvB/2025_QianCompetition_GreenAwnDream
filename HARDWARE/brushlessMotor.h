
#ifndef __BRUSHLESSMOTOR_H
#define __BRUSHLESSMOTOR_H

#include "system.h"

#define ABZ_Resolution 1024

// 电机参数
//#define POLE_PAIRS        7       // 电机极对数
#define PWM_FREQ          20000   // PWM频率(Hz)
#define PWM_PERIOD        (168000000/PWM_FREQ) // 定时器周期值(APB2=84MHz, TIMxCLK=168MHz)

// 数学常量
#define SQRT3             1.73205080757f
#define _2PI              6.28318530718f
#define _3PI_2            4.71238898038f

typedef enum{
	LeftWheel=0,
	RightWheel=1
}ChooseWheel; //选择左轮还是右轮

typedef enum{
	Region_1=1,
	Region_2=2,
	Region_3=3,
	Region_4=4,
	Region_5=5,
	Region_6=6
}Rotor_Region;

// FOC控制结构体
typedef struct {
//    // 电流值
//    float Ialpha, Ibeta;
//    float Id, Iq;
//    float Ia, Ib, Ic;
    
    // 电压值
    float Valpha, Vbeta;
    float Vd, Vq;
    
    // 角度与速度
    float electrical_angle;
    float mechanical_angle;
    float shaft_velocity;
    
    // 控制目标
    float target_current;
} FOC_HandleTypeDef;

// PID控制器
typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    float output;
    float max_output;
} PID_HandleTypeDef;




/*--------Motor_A control pins--------*/

/*------------------------------------*/


/*--------Motor_B control pins--------*/

/*------------------------------------*/


/*--------Motor_C control pins--------*/

/*------------------------------------*/


/*--------Motor_D control pins--------*/

/*------------------------------------*/
			
void Enable_Pin(void);
void TIM4_Init(void);
void TIM8_Init(void);


#endif
