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
//ƽ�������м������ȫ���ƶ�С��ר��
Smooth_Control smooth_control;  

//The parameter structure of the motor
//����Ĳ����ṹ��
BrushlessMotor Motor_Left, Motor_Right;
//BrushlessMotor MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D;  

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
/************ С���ͺ���ر��� **************************/
/************ Variables related to car model ************/

//PS2 controller, Bluetooth APP, aircraft model controller, CAN communication, serial port 1, serial port 5 communication control flag bit.
//These 6 flag bits are all 0 by default, representing the serial port 3 control mode
//PS2�ֱ�������APP����ģ�ֱ���CANͨ�š�����1������5ͨ�ſ��Ʊ�־λ����6����־λĬ�϶�Ϊ0��������3����ģʽ
u8 APP_ON_Flag=0, Usart1_ON_Flag, Usart5_ON_Flag; 



//�������ֵ
int servo_direction[6] = {0};
int servo_pwm_count = 500;

int check_time_count_motor_forward=300,check_time_count_motor_retreat=500;
int POT_val;

int Servo_Count[6] = {500, 500, 500, 500, 500, 500};

u8 uart3_receive_message[50];
u8 uart3_send_flag;
u8 message_count=0;

u8 uart2_receive_message[50];
u8 uart2_send_flag=5;
u8 app_count=0;

int Full_rotation = 16799;

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
