
#ifndef __BRUSHLESSMOTOR_H
#define __BRUSHLESSMOTOR_H

#include "system.h"

#define ABZ_Resolution 1024

// �������
//#define POLE_PAIRS        7       // ���������
#define PWM_FREQ          20000   // PWMƵ��(Hz)
#define PWM_PERIOD        (168000000/PWM_FREQ) // ��ʱ������ֵ(APB2=84MHz, TIMxCLK=168MHz)

// ��ѧ����
#define SQRT3             1.73205080757f
#define _2PI              6.28318530718f
#define _3PI_2            4.71238898038f

typedef enum{
	LeftWheel=0,
	RightWheel=1
}ChooseWheel; //ѡ�����ֻ�������

typedef enum{
	Region_1=1,
	Region_2=2,
	Region_3=3,
	Region_4=4,
	Region_5=5,
	Region_6=6
}Rotor_Region;

// FOC���ƽṹ��
typedef struct {
//    // ����ֵ
//    float Ialpha, Ibeta;
//    float Id, Iq;
//    float Ia, Ib, Ic;
    
    // ��ѹֵ
    float Valpha, Vbeta;
    float Vd, Vq;
    
    // �Ƕ����ٶ�
    float electrical_angle;
    float mechanical_angle;
    float shaft_velocity;
    
    // ����Ŀ��
    float target_current;
} FOC_HandleTypeDef;

// PID������
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
