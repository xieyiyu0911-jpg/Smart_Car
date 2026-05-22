/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020, 逐飞科技
 * All rights reserved.
 * 技术支持 QQ 群：一群 179029047(已满)  二群 244861897(已满)  三群 824575535
 *
 * 本代码版权归逐飞科技所有，未经许可不得用于商业用途。
 * 欢迎在学习和竞赛中使用，修改代码时请保留本版权声明。
 *
 * @file           main
 * @company        成都逐飞科技有限公司
 * @author         逐飞科技(QQ790875685)
 * @version        查看 doc 目录下 version 文件中的版本说明
 * @Software       MDK FOR C251 V5.60
 * @Target core    STC32G12K128
 * @Taobao         https://seekfree.taobao.com/
 * @date           2020-12-18
 *********************************************************************************************************************/

#include "headfile.h"

#define TEMP_BUFFER_SIZE  	64
static  fifo_struct     	temp_uart_fifo;
static  uint8            temp_uart_buffer[TEMP_BUFFER_SIZE];  // 串口临时接收缓冲区

#pragma float64
/*
 * 系统频率可在 board.h 中通过 FOSC 宏定义修改。
 * 当 board.h 中的 FOSC 设置为 0 时，程序会自动配置系统频率为 33.1776MHz。
 * 在 board_init() 中，已经将 P54 默认配置为复位脚。
 * 如果需要将 P54 作为普通 IO 使用，请在 board.c 的 board_init() 中删除 SET_P54_RESET 配置。
 */
 
 extern float xdata Yaw_Angle;
 extern float xdata error1;
 extern float xdata Turn_Cmd1;
 extern volatile int16 motor_speed_L ,motor_speed_R;
 extern float Motor_L_output;
 extern float Motor_output_R;
 extern float SpeedMeasure_L;
 extern float SpeedMeasure_R;// 右轮实际速度
 extern float SpeedTarget_L;
 extern float Target_Right1;
 extern uint16 Result_Middle_M;
 extern uint8 Timer_Config;
 extern float xdata Source;
 extern float Yaw_Angular_Speed;
 extern uint16 Result_L, Result_Middle_M_L, Result_Middle_M_R, Result_R;
 extern int  xdata array1[7] ,xdata array2[7] , xdata array3[7] ,xdata array4[7] ; //xdata array5[7]
 
 extern float error_change;
 extern float error;


// 该函数在 isr.c 的 UART1_Isr() 中断服务函数中被回调
void uart_isr_call_back(uint8 dat)
{
	fifo_write_buffer(&temp_uart_fifo, &dat, 1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     上位机接收回调函数
// 参数说明     *buff           用于接收数据的缓冲区地址
// 参数说明     length          期望接收的数据长度
// 返回参数     uint32          实际接收到的数据长度
//-------------------------------------------------------------------------------------------------------------------
uint32 seekfree_assistant_receive_callback   (uint8 *buff, uint32 length)
{
	fifo_read_buffer(&temp_uart_fifo, buff, &length, FIFO_READ_AND_CLEAN);
	return length;
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     上位机发送回调函数
// 参数说明     *buff           需要发送的数据缓冲区地址
// 参数说明     length          需要发送的数据长度
// 返回参数     uint32          剩余未发送的数据长度
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
float diff_ratio;

uint8 xdata read_buff1[8] = {0,0,0,0,0,0,0,0};
uint8 xdata read_buff2[4] = {0,0,0,0};// 起跑参数

uint8 xdata	write_buff1[8] = {0,0,0,0,0,0,0,0};// 参数备份值
uint8 xdata	write_buff2[4] = {0,0,0,0};// 参数备份值

uint8 xdata Change_Config_Add[6] = {0,0,0,0,0,0};
uint8 xdata Change_Config_Reduce[6] = {0,0,0,0,0,0};

uint8 xdata k = 0;
static uint8 xdata uart_debug_cnt = 0;


volatile extern uint8 xdata circle_config;

volatile extern float xdata Second_distance;






static void send_inductor_to_assistant(void)
{
    seekfree_assistant_oscilloscope_data.dat[0] = Result_L;
    seekfree_assistant_oscilloscope_data.dat[1] = Result_Middle_M_L;
    seekfree_assistant_oscilloscope_data.dat[2] = Result_Middle_M_R;
    seekfree_assistant_oscilloscope_data.dat[3] = Result_R;
	seekfree_assistant_oscilloscope_data.dat[4] = Result_Middle_M;
    seekfree_assistant_oscilloscope_data.dat[5] = Yaw_Angular_Speed;
	seekfree_assistant_oscilloscope_data.dat[6] = error;
	seekfree_assistant_oscilloscope_data.dat[7] = error_change;
    seekfree_assistant_oscilloscope_data.channel_num = 8;
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
}


void main()
{
	board_init();			// 初始化底层硬件，相关配置请勿随意删除
	
//	IP3 |= 0x02;
//	IP3H |= 0x02;
	Uarts_Init();

//	iap_init();				// 初始化 EEPROM
	Buzzer_Init();
//	Button_Init();
//	OLED_LCD_Init();
//	Sw_Init();
	ADCs_Init();// ADC 采样初始化
	Pwms_Init();// 电机 PWM 初始化
	imu660ra_init();// IMU 初始化
	quaternion_init();// 四元数初始化
	gyro_calibrate(200);
//	OLED_LCD_Show();
	// 风扇
	//fan_init();
	
		// 注册上位机发送回调
	seekfree_assistant_transfer = seekfree_assistant_transfer_callback;// 回调函数
	
		// 注册上位机接收回调
	seekfree_assistant_receive = seekfree_assistant_receive_callback;// 回调函数
	
	seekfree_assistant_init();
	
	// 初始化 FIFO
	fifo_init(&temp_uart_fifo, FIFO_DATA_8BIT, temp_uart_buffer, TEMP_BUFFER_SIZE);

//	// 设置默认 PID 参数
//	seekfree_assistant_parameter[0] = 2.3f;    // 左轮P2.3
//	seekfree_assistant_parameter[1] = 0.28f;    // 左轮I0.28
//	seekfree_assistant_parameter[2] = 0.03f;   // 左轮D0.03
//	seekfree_assistant_parameter[3] = 1.5f;    // 右轮P1.5
//	seekfree_assistant_parameter[4] = 0.24f;    // 右轮I0.24
//	seekfree_assistant_parameter[5] = 0.08;   // 右轮D0.08
//	seekfree_assistant_parameter[6] = 0.0f;    // 额暂时还没用到
//	seekfree_assistant_parameter[7] = 0.0f;    // 停车保护，1就停车
	
	Time_Pulse_Init();// 编码器计数器初始化
	
   
	    // 主循环负责处理上位机、显示和控制状态
    while(1)
    {
           
		    uint8 xdata i = 0;			
			uint8 xdata Menu_Config = 0;				
					
				if(Timer_Config == 1)
			{
				Timer_Config = 0;
				LCD_Config = 0;
//				if(++uart_debug_cnt >= 50)
//				{
//					uart_debug_cnt = 0;
//					//uart_putstr(UART_2, "UART2 Hello,world");
//				}
//				update_fan_control();

				if(LCD_Config == 0)
			{											 // 解析上位机收到的数据
				seekfree_assistant_data_analysis();
			
				for(j = 0; j < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; j++)
				{
                // 参数更新标志
                if(seekfree_assistant_parameter_update_flag[j])
				{
                seekfree_assistant_parameter_update_flag[j] = 0;

                // 打印调试信息
//                printf("receive data channel : %d ", j);
//                printf("data : %f ", seekfree_assistant_parameter[j]);
//                printf("");
                }//下位机接收上位机的参数函数，即传参函数
		        }
				// 参数更新后通过串口打印当前值
				send_inductor_to_assistant();
//				OLED_LCD_Show();    
			}
			}
	}
}