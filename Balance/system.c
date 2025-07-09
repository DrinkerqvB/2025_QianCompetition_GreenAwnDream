#include "system.h"

//Vehicle three-axis target moving speed, unit: m/s
//С������Ŀ���˶��ٶȣ���λ��m/s
float Move_X, Move_Y, Move_Z;   

//PID parameters of Speed control
//�ٶȿ���PID����
float Velocity_KP=1.5f;
float Velocity_KI=0.06f;
float Velocity_KD=0.2; 

//Smooth control of intermediate variables, dedicated to omni-directional moving cars
//ƽ�������м����
Smooth_Control smooth_control;  

//The parameter structure of the motor
//����Ĳ����ṹ��
BrushlessMotor Motor_Left, Motor_Right;


/************ С���ͺ���ر��� **************************/
/************ Variables related to car model ************/
//Encoder accuracy
//����������
float Encoder_precision; 
//Wheel circumference, unit: m
//�����ܳ�����λ��m
float Wheel_perimeter; 
//Drive wheel base, unit: m
//�������־࣬��λ��m
float Wheel_spacing; 
//The wheelbase of the front and rear axles of the trolley, unit: m
//С��ǰ�������࣬��λ��m
float Axle_spacing; 
//All-directional wheel turning radius, unit: m
//ȫ����ת��뾶����λ��m
float Omni_turn_radiaus; 


void systemInit(void)
{       
	
	//�ж����ȼ���������
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	//RGB��ɫ�ƿ������ų�ʼ��
	RGB_Init();
	
	//KEY0 ��ť��ʼ��������ESP8266����
	Key0_Init();
	
	//����1��ʼ����ͨ�Ų�����115200��������Modbusͨ��
	Modbus_Init();
	
	uart2_init(9600);  
	
	
	//����3��ʼ����ͨ�Ų�����115200
	//ESP8266 ������ʼ�� ��ʹ��USART3
	uart3_init(115200);
//	ESP8266_Init();

	//��ʼ����OLED��ʾ�����ӵ�Ӳ���ӿ�	
	OLED_init();
	
	
	//��·ѭ����ʼ����ʹ��I2C1
	Tracking_Init();
	
	
  //����С����Ӧ�Ĳ�����ʼ��	
	Robot_Select();                 
	
	 //Encoder A is initialized to read the real time speed of motor C  
  //���ֱ�������ʼ�������ڶ�ȡMotor_Left��ʵʱ�ٶ�	
	 Encoder_Init_TIM2();
	//Encoder B is initialized to read the real time speed of motor D
  //���ֱ�������ʼ�������ڶ�ȡMotor_Right��ʵʱ�ٶ�	
	  Encoder_Init_TIM3();   
	
	//�������ұ�
	 FOC_Init();
	 //��ʼ��������趨ʱ��
	 TIM7_Init();  //FOC���¶�ʱ��
	 //����źŲ������趨ʱ��
	 TIM4_Init();
	 TIM8_Init();
						
}
