/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020, ��ɿƼ�
 * All rights reserved.
 * ����֧�� QQ Ⱥ��һȺ 179029047(����)  ��Ⱥ 244861897(����)  ��Ⱥ 824575535
 *
 * �������Ȩ����ɿƼ����У�δ�����ɲ���������ҵ��;��
 * ��ӭ��ѧϰ�;�����ʹ�ã��޸Ĵ���ʱ�뱣������Ȩ������
 *
 * @file           main
 * @company        �ɶ���ɿƼ����޹�˾
 * @author         ��ɿƼ�(QQ790875685)
 * @version        �鿴 doc Ŀ¼�� version �ļ��еİ汾˵��
 * @Software       MDK FOR C251 V5.60
 * @Target core    STC32G12K128
 * @Taobao         https://seekfree.taobao.com/
 * @date           2020-12-18
 *********************************************************************************************************************/

#include "headfile.h"

#define TEMP_BUFFER_SIZE  	64
static  fifo_struct     	temp_uart_fifo;
static  uint8            temp_uart_buffer[TEMP_BUFFER_SIZE];  // ������ʱ���ջ�����

#pragma float64
/*
 * ϵͳƵ�ʿ��� board.h ��ͨ�� FOSC �궨���޸ġ�
 * �� board.h �е� FOSC ����Ϊ 0 ʱ��������Զ�����ϵͳƵ��Ϊ 33.1776MHz��
 * �� board_init() �У��Ѿ��� P54 Ĭ������Ϊ��λ�š�
 * �����Ҫ�� P54 ��Ϊ��ͨ IO ʹ�ã����� board.c �� board_init() ��ɾ�� SET_P54_RESET ���á�
 */
 
 extern float xdata error1;
 extern float xdata Turn_Cmd1;
 extern volatile int16 motor_speed_L ,motor_speed_R;
 extern float xdata Motor_L_output;
 extern float xdata Motor_output_R;
 extern float xdata SpeedMeasure_L;
 extern float xdata SpeedMeasure_R;// ����ʵ���ٶ�
 extern float xdata SpeedTarget_L;
 extern float xdata Target_Right1;
 extern uint16 Result_Middle_M;
 extern uint8 Timer_Config;
 extern float xdata Source;
 extern float xdata Yaw_Angular_Speed;
 extern uint16 Result_L, Result_Middle_M_L, Result_Middle_M_R, Result_R;
 extern int  xdata array1[7] ,xdata array2[7] , xdata array3[7] ,xdata array4[7] , xdata array5[7];
 
 extern float xdata error_change;
 extern float xdata error;
 
 extern volatile int xdata Motor_Speed_Left[7];    // ���ֱ�����ԭʼ����
 extern volatile int xdata Motor_Speed_Right[7];   // ���ֱ�����ԭʼ����
 extern volatile float xdata Yaw_Angle;            // ����ƫ���ǻ���
 extern volatile float xdata Turn_Output;          // ת���������ʾ�ã�
 extern volatile uint16 xdata conservation;        // ������־
 extern Round_State_TypeDef Round_State;           // ����״̬
 extern uint8 Round_Direction;                     // ��������
 extern float xdata Round_Pre_Distance;                  // Ԥ�������
 extern float xdata Round_Exit_Distance;                 // ���������
 
 extern volatile float xdata Pitch_Angle;


// �ú����� isr.c �� UART1_Isr() �жϷ������б��ص�
void uart_isr_call_back(uint8 dat)
{
	fifo_write_buffer(&temp_uart_fifo, &dat, 1);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��λ�����ջص�����
// ����˵��     *buff           ���ڽ������ݵĻ�������ַ
// ����˵��     length          �������յ����ݳ���
// ���ز���     uint32          ʵ�ʽ��յ������ݳ���
//-------------------------------------------------------------------------------------------------------------------
uint32 seekfree_assistant_receive_callback   (uint8 *buff, uint32 length)
{
	fifo_read_buffer(&temp_uart_fifo, buff, &length, FIFO_READ_AND_CLEAN);
	return length;
}
//-------------------------------------------------------------------------------------------------------------------
// �������     ��λ�����ͻص�����
// ����˵��     *buff           ��Ҫ���͵����ݻ�������ַ
// ����˵��     length          ��Ҫ���͵����ݳ���
// ���ز���     uint32          ʣ��δ���͵����ݳ���
//-------------------------------------------------------------------------------------------------------------------
uint32 seekfree_assistant_transfer_callback   (const uint8 *buff, uint32 length)
{
	uart_putbuff(DEBUG_UART, buff, length);
	return 0;
}

int16 xdata Change_Config_Number = 0;
int16 xdata Change_Config_Number1 = 0;
uint8 xdata Key_Config[4];
uint8 xdata j = 0;
uint8 xdata LCD_Config = 0;
float xdata diff_ratio;

uint8 xdata read_buff1[8] = {0,0,0,0,0,0,0,0};
uint8 xdata read_buff2[4] = {0,0,0,0};// ���ܲ���

uint8 xdata	write_buff1[8] = {0,0,0,0,0,0,0,0};// ��������ֵ
uint8 xdata	write_buff2[4] = {0,0,0,0};// ��������ֵ

uint8 xdata Change_Config_Add[6] = {0,0,0,0,0,0};
uint8 xdata Change_Config_Reduce[6] = {0,0,0,0,0,0};

uint8 xdata k = 0;
static uint8 xdata uart_debug_cnt = 0;


volatile extern uint8 xdata circle_config;

volatile extern float xdata turn_cmd;


extern uint8 xdata Fuzzy_Config;


extern uint8 xdata Tiaocan_Config;

extern float expect_gyro;



static void send_inductor_to_assistant(void)
{
//	if(Tiaocan_Config == 1 || Tiaocan_Config == 0)
//	{
//    seekfree_assistant_oscilloscope_data.dat[0] = SpeedTarget_L;
//    seekfree_assistant_oscilloscope_data.dat[1] = SpeedMeasure_L;
//    seekfree_assistant_oscilloscope_data.dat[2] = Target_Right1;
//    seekfree_assistant_oscilloscope_data.dat[3] = SpeedMeasure_R;
//    seekfree_assistant_oscilloscope_data.channel_num = 4;
//    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
//	}
	
//	if(Tiaocan_Config == 2)
//	{
	  seekfree_assistant_oscilloscope_data.dat[0] = Yaw_Angular_Speed;
    seekfree_assistant_oscilloscope_data.dat[1] = expect_gyro;
    seekfree_assistant_oscilloscope_data.dat[2] = Result_Middle_M;
    seekfree_assistant_oscilloscope_data.dat[3] = SpeedTarget_L;
		seekfree_assistant_oscilloscope_data.dat[4] = SpeedMeasure_L;
		seekfree_assistant_oscilloscope_data.dat[5] = Target_Right1;
    seekfree_assistant_oscilloscope_data.dat[6] = SpeedMeasure_R;
    seekfree_assistant_oscilloscope_data.dat[7] = 0;
    seekfree_assistant_oscilloscope_data.channel_num = 8;
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
//	}
	
}


void main()
{
	board_init();			// ��ʼ���ײ�Ӳ�������������������ɾ��
	
//	IP3 |= 0x02;
//	IP3H |= 0x02;
//	Uarts_Init();
	wireless_uart_init();

//	iap_init();				// ��ʼ�� EEPROM
	Buzzer_Init();
//	Button_Init();
//	OLED_LCD_Init();
//	Sw_Init();
	ADCs_Init();// ADC ������ʼ��
	Pwms_Init();// ��� PWM ��ʼ��
	imu660ra_init();// IMU ��ʼ��
	quaternion_init();// ��Ԫ����ʼ��
	gyro_calibrate(200);
//	OLED_LCD_Show();
	// ����
	//fan_init();
	
//		// ע����λ�����ͻص�
//	seekfree_assistant_transfer = seekfree_assistant_transfer_callback;// �ص�����
//	
//		// ע����λ�����ջص�
//	seekfree_assistant_receive = seekfree_assistant_receive_callback;// �ص�����

		//����ת����
		// ע����λ�����ͻص�
	seekfree_assistant_transfer = wireless_uart_send_buff;// �ص�����
	
		// ע����λ�����ջص�
	seekfree_assistant_receive = wireless_uart_read_buff;// �ص�����
	
	seekfree_assistant_init();
	
//	// ��ʼ�� FIFO
//	fifo_init(&temp_uart_fifo, FIFO_DATA_8BIT, temp_uart_buffer, TEMP_BUFFER_SIZE);

	
	Time_Pulse_Init();// ��������������ʼ��
	
//	pwm_init(PWMB_CH1_P20, 50, 600);
	
   
	    // ��ѭ����������λ������ʾ�Ϳ���״̬
    while(1)
    {
           
		    uint8 xdata i = 0;			
			uint8 xdata Menu_Config = 0;				
					
				if(Timer_Config == 1)
			{
				Timer_Config = 0;
				LCD_Config = 0;
				


				
				
				
				

				if(LCD_Config == 0)
			{											 // ������λ���յ�������
				seekfree_assistant_data_analysis();
			
				for(j = 0; j < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; j++)
				{
                // �������±�־
                if(seekfree_assistant_parameter_update_flag[j])
				{
                seekfree_assistant_parameter_update_flag[j] = 0;

                // ��ӡ������Ϣ
//                printf("receive data channel : %d ", j);
//                printf("data : %f ", seekfree_assistant_parameter[j]);
//                printf("");
                }//��λ��������λ���Ĳ��������������κ���
		        }
				// �������º�ͨ�����ڴ�ӡ��ǰֵ
				send_inductor_to_assistant();
				
				
//				OLED_LCD_Show();    
			}
			}
	}
}