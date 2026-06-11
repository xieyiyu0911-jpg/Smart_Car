///*********************************************************************************************************************
// * COPYRIGHT NOTICE
// * Copyright (c) 2020, ��ɿƼ�
// * All rights reserved.
// * ����֧�� QQ Ⱥ��һȺ 179029047(����)  ��Ⱥ 244861897(����)  ��Ⱥ 824575535
// *
// * �������Ȩ����ɿƼ����У�δ�����ɲ���������ҵ��;��
// * ��ӭ��ѧϰ�;�����ʹ�ã��޸Ĵ���ʱ�뱣������Ȩ������
// *
// * @file           isr
// * @company        �ɶ���ɿƼ����޹�˾
// * @author         ��ɿƼ�(QQ790875685)
// * @version        �鿴 doc Ŀ¼�� version �ļ��еİ汾˵��
// * @Software       MDK FOR C251 V5.60
// * @Target core    STC32G12K128
// * @Taobao         https://seekfree.taobao.com/
// * @date           2020-4-14
// ********************************************************************************************************************/
#include "headfile.h"



#define Servo_angle_max  180// ���ת�����
#define Servo_angle_min  -180 // ��Сת�����


#define ANGLE_PID 0
#define ANGLE_SPEED_PID 1

float xdata Second_distance = 0;
float xdata Second_encoder_ave = 0;

volatile float xdata SpeedMeasure_L = 0, xdata SpeedTarget_L = 0;
volatile float xdata SpeedMeasure_R = 0, xdata Target_Right1 = 0;
volatile int16 motor_speed_L = 0;
volatile int16 motor_speed_R = 0;
volatile uint16 Result_L = 0, Result_Middle_M_L = 0,Result_Middle_M_R = 0,Result_R = 0, Result_Middle_M = 0;// ���ֵ

volatile float  xdata Angle_Speed_error1 = 0;
float  xdata Angle_Speed_Output1 = 0;
volatile float xdata Turn_Output = 0;

 extern float xdata error1;
 extern float xdata Turn_Cmd1;
 extern float xdata Motor_L_output;
 extern float xdata Motor_output_R;

 extern uint32 xdata Left_Round_Config2,xdata Right_Round_Config2,xdata Left_Round_Config1,xdata Right_Round_Config1;

 extern float error1;
 extern float xdata Yaw_Angular_Speed;

 extern void uart_isr_call_back(uint8 dat);
 extern uint32 xdata Round_Config1,xdata Round_Config2;
 
float xdata Source = 0;
volatile uint8 Timer_Config = 0;

 extern uint8 xdata LCD_Config;

 extern uint8 xdata read_buff1[8];
 extern uint8 xdata read_buff2[4];
 
volatile float xdata Yaw_Angle;
volatile float xdata Pitch_Angle;

volatile float xdata Angle_time1 = 0;
volatile float xdata turn_cmd = 0;
volatile uint16 xdata conservation = 0;
volatile int  xdata array1[7] ,xdata array2[7] , xdata array3[7] ,xdata array4[7] ,xdata array5[7];
int xdata i = 0;
volatile int xdata Motor_Speed_Left[7],Motor_Speed_Right[7];


volatile uint8 xdata Source_Config = 0;

volatile int16 xdata Source_Start = 0;


uint8 xdata Tiaocan_Config = 0;

float xdata Speed_L_P = 0;
float xdata Speed_L_I = 0;
float xdata Speed_L_D = 0;
float xdata Speed_R_P = 0;
float xdata Speed_R_I = 0;
float xdata Speed_R_D = 0;

// ========== ���������������� ==========
  extern Round_State_TypeDef Round_State;
  extern uint8 Round_Direction;
  extern float xdata Round_Pre_Distance;
  extern float xdata Round_Exit_Distance;
  extern const Round_Config_TypeDef Round_Params;


// UART1 �ж�
void UART1_Isr() interrupt 4
{
    uint8 res;
	static uint8 dwon_count;
    if(UART1_GET_TX_FLAG)
    {
        UART1_CLEAR_TX_FLAG;
        busy[1] = 0;
    }
    if(UART1_GET_RX_FLAG)
    {
        UART1_CLEAR_RX_FLAG;
        res = SBUF;
        // ��������ģʽ
        if(res == 0x7F)
        {
            if(dwon_count++ > 20)
                IAP_CONTR = 0x60;
        }
        else
        {
            dwon_count = 0;
        }
    }
}

// UART2 �ж�
void UART2_Isr() interrupt 8
{
    if(UART2_GET_TX_FLAG)
	{
        UART2_CLEAR_TX_FLAG;
		busy[2] = 0;
	}
    if(UART2_GET_RX_FLAG)
	{
        UART2_CLEAR_RX_FLAG;
		uart_isr_call_back(S2BUF);
	}
}


// UART3 �ж�
void UART3_Isr() interrupt 17
{
    if(UART3_GET_TX_FLAG)
	{
        UART3_CLEAR_TX_FLAG;
		busy[3] = 0;
	}
    if(UART3_GET_RX_FLAG)
	{
        UART3_CLEAR_RX_FLAG;
		// Ԥ������ 3 ���մ���

	}
}


// UART4 �ж�
void UART4_Isr() interrupt 18
{
	uint8 res;
	static uint8 dwon_count4;
	
    if(UART4_GET_TX_FLAG)
	{
    UART4_CLEAR_TX_FLAG;
		busy[4] = 0;
	}
    if(UART4_GET_RX_FLAG)
	{
        UART4_CLEAR_RX_FLAG;
		    res = S4BUF;
        // ��������ģʽ
        if(res == 0x7F)
        {
            if(dwon_count4++ > 20)
                IAP_CONTR = 0x60;
        }
        else
        {
            dwon_count4 = 0;
        }
				
		// ��������ģ�鴮�ڽ������� S4BUF
		if(wireless_module_uart_handler != NULL)
		{
			// ������λ�����ڽ��ջص�
			// ����ǰ�ֽڽ�������ģ�鴦������
			wireless_module_uart_handler(S4BUF);
		}
		
		uart_isr_call_back(res);
	}
}


void INT0_Isr() interrupt 0
{
	
}
void INT1_Isr() interrupt 2
{

}
void INT2_Isr() interrupt 10
{
	INT2_CLEAR_FLAG;  // ����ⲿ�ж� 2 ��־
			
			
}
void INT3_Isr() interrupt 11
{
	INT3_CLEAR_FLAG;  // ����ⲿ�ж� 3 ��־
}

void INT4_Isr() interrupt 16
{
	INT4_CLEAR_FLAG;  // ����ⲿ�ж� 4 ��־
}

void TM0_Isr() interrupt 1
{

}
void TM1_Isr() interrupt 3
{

}
void car_control_timer_handler(void)
{
		for(i=0 ; i<7 ; i++)
    {	
			Motor_Speed_Left[i] = ctimer_count_read(CTIM3_P04);
			Motor_Speed_Right[i] = ctimer_count_read(CTIM0_P34);
			
			array1[i] = adc_once(ADC_P11,ADC_8BIT); // �����
			array2[i] = adc_once(ADC_P00,ADC_8BIT);// ���е��
			array3[i] = adc_once(ADC_P05,ADC_8BIT);// ���е��
			array4[i] = adc_once(ADC_P06,ADC_8BIT);// �Ҳ���
			array5[i] = adc_once(ADC_P01,ADC_8BIT);// �м���
    }
			
		
			motor_speed_L = (int16) Servo_Measure(Motor_Speed_Left,Times);// ����������ȥ��ֵƽ�����������ɷ�����ж�
			motor_speed_R = (int16) Servo_Measure(Motor_Speed_Right,Times);
			
				SpeedTarget_L = 400 ;//+ seekfree_assistant_parameter[6];//650
				
//				if(SpeedTarget_L >= 350)//1000
//					SpeedTarget_L = 350;
//				else if(SpeedTarget_L <= 100)//300
//					SpeedTarget_L = 100;
				
				Target_Right1 = SpeedTarget_L;
			
			
//				if(P35 == 1)// ������������
//			{
//				motor_speed_L = - motor_speed_L;
//			}
//			
//			if(P53 == 1)// �ұ����������
//			{
//				motor_speed_R = - motor_speed_R;
//			}
			
			SpeedMeasure_L = motor_speed_L;
			SpeedMeasure_R = motor_speed_R;
			
			Result_L = Servo_Measure(array1, Times);  // ���ֵ�˲�
			Result_Middle_M_L = Servo_Measure(array2, Times);
			Result_Middle_M_R = Servo_Measure(array3, Times);
			Result_R = Servo_Measure(array4, Times);
			Result_Middle_M = Servo_Measure(array5, Times);
			
//			Result_L = Servo[0];
//			Result_Middle_M_L = Servo[1];
//			Result_Middle_M_R = Servo[2];
//		  	Result_R = Servo[3];
//			Result_Middle_M = Servo[4];// �м���ֵ
			
			Result_L = (uint16)((Result_L/ (Max1 * 1.00) ) *100);
			Result_Middle_M_L = (uint16)((Result_Middle_M_L/ (Max2 * 1.00)) *100);
			Result_Middle_M_R = (uint16)((Result_Middle_M_R/ (Max4 * 1.00)) *100);
			Result_Middle_M = (uint16)((Result_Middle_M/ (Max3 * 1.00)) *100);
			Result_R = (uint16)((Result_R/ (Max5 * 1.00) ) *100);  
	




		// ========== �������׶νǶȻ�������̼��� ==========
		  imu660ra_get_gyro();
		  Yaw_Angular_Speed = - imu660ra_gyro_transition((float)imu660ra_gyro_z - imu_data.gyro_z);
		  
		  quaternion_update();
		  Pitch_Angle = (euler.pitch * 90) / 40.0f;
			Yaw_Angle = (euler.yaw * 90) / 40.0f;
//		  Yaw_Angle += (imu660ra_gyro_transition((float)imu660ra_gyro_z - imu_data.gyro_z)) * 0.01;

//		  // Ԥ�����׶Σ������ǶȻ��֣���������ѭ��
//		//  if(Round_State == ROUND_NONE)
//		//  {
//		//      // ����ѭ����������
//		//  }
//		  // �뻷�׶� 0��~60��
//			  // ========== ������̼������� ==========
//		  // Ԥ�����׶���̼���
//		  if(Round_State == ROUND_PRE)
//		  {
//			  Second_distance_calculate();
//			  Round_Pre_Distance += Second_encoder_ave;
//			  if(Round_Pre_Distance >= Round_Params.pre_distance_thres)
//			  {
//				  // ��̴ﵽ��ֵ������ƫ���ǶȻ��֣������뻷�׶�
//				  Round_State = ROUND_ENTRY;
//				  Round_Pre_Distance = 0;
//				  Yaw_Angle = 0;
//				  Buzzer_On();
//			  }
//		  }
//		  else if(Round_State == ROUND_ENTRY)
//		  {
//			  Yaw_Angle += (imu660ra_gyro_transition((float)imu660ra_gyro_z - imu_data.gyro_z)) * 0.01;
//			  if(fabs(Yaw_Angle) >= Round_Params.entry_angle_end)
//			  {
//				  Round_State = ROUND_INSIDE;
//				  if(Round_Direction == 0)//�󻷵�
//				  {
//					  Yaw_Angle = -Round_Params.entry_angle_end;
//				  }
//				  else//�һ���
//				  {
//					  Yaw_Angle = Round_Params.entry_angle_end;
//				  }
//			  }
//		  }
//		  // ���ڽ׶� 60��~270��
//		  else if(Round_State == ROUND_INSIDE)
//		  {
//			  Yaw_Angle += (imu660ra_gyro_transition((float)imu660ra_gyro_z - imu_data.gyro_z)) * 0.01;
//			  if(fabs(Yaw_Angle) >= Round_Params.inside_angle_end)
//			  {
//				  Round_State = ROUND_EXIT;
//				  if(Round_Direction == 0)//�󻷵�
//				  {
//					  Yaw_Angle = -Round_Params.inside_angle_end;
//				  }
//				  else//�һ���
//				  {
//					  Yaw_Angle = Round_Params.inside_angle_end;
//				  }
//			  }
//		  }
//		  // �����׶� 270��~330��
//		  else if(Round_State == ROUND_EXIT)
//		  {
//			  Yaw_Angle += (imu660ra_gyro_transition((float)imu660ra_gyro_z - imu_data.gyro_z)) * 0.01;
//			  if(fabs(Yaw_Angle) >= Round_Params.exit_angle_end)
//			  {
//				  Round_State = ROUND_EXIT_AFTER;
//				  Yaw_Angle = 0;
//				  Round_Exit_Distance = 0; 
//			  }
//		  }

//		  // ��������̼���
//		  else if(Round_State == ROUND_EXIT_AFTER)
//		  {
//			  Second_distance_calculate();
//			  Round_Exit_Distance += Second_encoder_ave;
//			  if(Round_Exit_Distance >= Round_Params.exit_distance_thres)
//			  {
//				  // ��̳���10cm�������˳�����״̬
//				  Round_State = ROUND_NONE;
//				  Round_Exit_Distance = 0;
//				  Buzzer_Off();
//			  }
//		  }
			
			turn_cmd = Turn_Control_PID(Result_L,Result_Middle_M_L,Result_Middle_M_R,Result_R,Result_Middle_M);
			Turn_Output = turn_cmd;		
						
			Differential_Speed_Control(Turn_Output);// ���ٷ���
				
//			Speed_L_P = seekfree_assistant_parameter[0];
//			Speed_L_I = seekfree_assistant_parameter[1];
//			Speed_L_D = seekfree_assistant_parameter[2];
//			Speed_R_P = seekfree_assistant_parameter[3];
//			Speed_R_I = seekfree_assistant_parameter[4];
//			Speed_R_D = seekfree_assistant_parameter[5];

			Motor_PID(SpeedTarget_L,motor_speed_L,1.6,3.65,1.5,Left,Turn_Output);// �����ٶȻ�PID 
			Motor_PID(Target_Right1,motor_speed_R,2.3,3.8,0.7,Right,Turn_Output);// �����ٶȻ�PID 
				//L:1.6,3.65,1.5
				//R:2.3,3.8,0.7			
				conservation = PID_Conservation(Result_L,Result_Middle_M_L,Result_Middle_M_R,Result_R);// �����ж�


//				Motor_PID(SpeedTarget_L,motor_speed_L,seekfree_assistant_parameter[0],seekfree_assistant_parameter[1],seekfree_assistant_parameter[2],Left,Turn_Output);// �����ٶȻ�PID //5,2.5,1.25/1.4,0.8,0/3,1.5,0.3/1.4/0.8
//				Motor_PID(Target_Right1,motor_speed_R,seekfree_assistant_parameter[3],seekfree_assistant_parameter[4],seekfree_assistant_parameter[5],Right,Turn_Output);// �����ٶȻ�PID //4.8,2.4,1.25/1.7,0.6,0/2.5,0.8,0.2/1.7��0.8


			
			
			if(conservation == 0 && LCD_Config == 0)
			{
				Motor_PWM_set_L();// �������PWM
				Motor_PWM_set_R();// �������PWM
			}//PID	

			

			
			ctimer_count_clean(CTIM0_P34);
			ctimer_count_clean(CTIM3_P04);
			
			Timer_Config = 1;
}


void TM2_Isr() interrupt 12
{
	TIM2_CLEAR_FLAG;  // UART2 ʹ�� TIM2 �������ʷ��������������ٿ��� TIM2 �����ж�
}

void TM3_Isr() interrupt 19
{
	TIM3_CLEAR_FLAG; // �����ʱ�� 3 �жϱ�־

}


void TM4_Isr() interrupt 20
{
	TIM4_CLEAR_FLAG; // �����ʱ�� 4 �жϱ�־
	car_control_timer_handler();

}

//void  INT0_Isr()  interrupt 0;
//void  TM0_Isr()   interrupt 1;
//void  INT1_Isr()  interrupt 2;
//void  TM1_Isr()   interrupt 3;
//void  UART1_Isr() interrupt 4;
//void  ADC_Isr()   interrupt 5;
//void  LVD_Isr()   interrupt 6;
//void  PCA_Isr()   interrupt 7;
//void  UART2_Isr() interrupt 8;
//void  SPI_Isr()   interrupt 9;
//void  INT2_Isr()  interrupt 10;
//void  INT3_Isr()  interrupt 11;
//void  TM2_Isr()   interrupt 12;
//void  INT4_Isr()  interrupt 16;
//void  UART3_Isr() interrupt 17;
//void  UART4_Isr() interrupt 18;
//void  TM3_Isr()   interrupt 19;
//void  TM4_Isr()   interrupt 20;
//void  CMP_Isr()   interrupt 21;
//void  I2C_Isr()   interrupt 24;
//void  USB_Isr()   interrupt 25;
//void  PWM1_Isr()  interrupt 26;
//void  PWM2_Isr()  interrupt 27;