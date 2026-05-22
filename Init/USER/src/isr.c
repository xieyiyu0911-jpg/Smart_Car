///*********************************************************************************************************************
// * COPYRIGHT NOTICE
// * Copyright (c) 2020, 逐飞科技
// * All rights reserved.
// * 技术支持 QQ 群：一群 179029047(已满)  二群 244861897(已满)  三群 824575535
// *
// * 本代码版权归逐飞科技所有，未经许可不得用于商业用途。
// * 欢迎在学习和竞赛中使用，修改代码时请保留本版权声明。
// *
// * @file           isr
// * @company        成都逐飞科技有限公司
// * @author         逐飞科技(QQ790875685)
// * @version        查看 doc 目录下 version 文件中的版本说明
// * @Software       MDK FOR C251 V5.60
// * @Target core    STC32G12K128
// * @Taobao         https://seekfree.taobao.com/
// * @date           2020-4-14
// ********************************************************************************************************************/
#include "headfile.h"

#define Max1 66
#define Max2 120
#define Max3 226
#define Max4 79
#define Max5 137
#define Times 7 // 电磁采样次数
#define factor3 0.8
#define Servo_angle_max  180// 最大转向输出
#define Servo_angle_min  -180 // 最小转向输出

#define Left 1
#define Right 0
#define ANGLE_PID 0
#define ANGLE_SPEED_PID 1

volatile float xdata Second_distance = 0;
volatile float xdata Second_encoder_ave = 0;

volatile float SpeedMeasure_L = 0,SpeedTarget_L = 0;
volatile float SpeedMeasure_R = 0,Target_Right1 = 0;
volatile int16 motor_speed_L = 0;
volatile int16 motor_speed_R = 0;
volatile uint16 Result_L = 0, Result_Middle_M_L = 0,Result_Middle_M_R = 0,Result_R = 0, Result_Middle_M = 0;// 电磁值

volatile float  Angle_Speed_error1 = 0;
volatile float  Angle_Speed_Output1 = 0;
volatile float  Turn_Output = 0;

 extern float xdata error1;
 extern float xdata Turn_Cmd1;
 extern float Motor_L_output;
 extern float Motor_output_R;

 extern uint32 xdata Left_Round_Config2,xdata Right_Round_Config2,xdata Left_Round_Config1,xdata Right_Round_Config1;

 extern float error1;
 extern float Yaw_Angular_Speed;

 extern void uart_isr_call_back(uint8 dat);
 extern uint32 xdata Round_Config1,xdata Round_Config2;
 
float xdata Source = 0;
volatile uint8 Timer_Config = 0;

 extern uint8 xdata LCD_Config;

 extern uint8 xdata read_buff1[8];
 extern uint8 xdata read_buff2[4];
 
volatile float xdata Yaw_Angle;

volatile float xdata Angle_time1 = 0;
volatile float turn_cmd = 0;
volatile uint16 xdata conservation = 0;
volatile int  xdata array1[7] ,xdata array2[7] , xdata array3[7] ,xdata array4[7] ,xdata array5[7];
int xdata i = 0;
volatile int xdata Motor_Speed_Left[7],Motor_Speed_Right[7];


volatile uint8 xdata Source_Config = 0;

volatile int16 xdata Source_Start = 0;



// ========== 环岛处理变量声明 ==========
  extern Round_State_TypeDef Round_State;
  extern uint8 Round_Direction;
  extern float Round_Pre_Distance;
  extern float Round_Exit_Distance;
  extern const Round_Config_TypeDef Round_Params;


// UART1 中断
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
        // 进入下载模式
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

// UART2 中断
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


// UART3 中断
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
		// 预留串口 3 接收处理

	}
}


// UART4 中断
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
        // 进入下载模式
        if(res == 0x7F)
        {
            if(dwon_count4++ > 20)
                IAP_CONTR = 0x60;
        }
        else
        {
            dwon_count4 = 0;
        }
				
		// 处理无线模块串口接收数据 S4BUF
		if(wireless_module_uart_handler != NULL)
		{
			// 调用上位机串口接收回调
			// 将当前字节交给无线模块处理函数
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
	INT2_CLEAR_FLAG;  // 清除外部中断 2 标志
			
			
}
void INT3_Isr() interrupt 11
{
	INT3_CLEAR_FLAG;  // 清除外部中断 3 标志
}

void INT4_Isr() interrupt 16
{
	INT4_CLEAR_FLAG;  // 清除外部中断 4 标志
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
			Motor_Speed_Left[i] = ctimer_count_read(CTIM0_P34);
			Motor_Speed_Right[i] = ctimer_count_read(CTIM3_P04);
			
			array1[i] = adc_once(ADC_P11,ADC_8BIT); // 左侧电感
            array2[i] = adc_once(ADC_P00,ADC_8BIT);// 左中电感
            array3[i] = adc_once(ADC_P05,ADC_8BIT);// 右中电感
            array4[i] = adc_once(ADC_P06,ADC_8BIT);// 右侧电感
			array5[i] = adc_once(ADC_P01,ADC_8BIT);// 中间电感
    }
			
		
			motor_speed_L = (int16) Servo_Measure(Motor_Speed_Left,Times);// 编码器计数去极值平均，正负号由方向脚判断
			motor_speed_R = (int16) Servo_Measure(Motor_Speed_Right,Times);
			
				SpeedTarget_L = 300 + seekfree_assistant_parameter[4];//650
				
//				if(SpeedTarget_L >= 350)//1000
//					SpeedTarget_L = 350;
//				else if(SpeedTarget_L <= 100)//300
//					SpeedTarget_L = 100;
				
				Target_Right1 = SpeedTarget_L;
			//}
			
//				if(P35 == 1)// 左编码器方向脚
//			{
//				motor_speed_L = - motor_speed_L;
//			}
//			
//			if(P53 == 1)// 右编码器方向脚
//			{
//				motor_speed_R = - motor_speed_R;
//			}
			
			SpeedMeasure_L = motor_speed_L;
			SpeedMeasure_R = motor_speed_R;
			
			Result_L = Servo_Measure(array1, Times);  // 电磁值滤波
			Result_Middle_M_L = Servo_Measure(array2, Times);
			Result_Middle_M_R = Servo_Measure(array3, Times);
			Result_R = Servo_Measure(array4, Times);
			Result_Middle_M = Servo_Measure(array5, Times);
			
//			Result_L = Servo[0];
//			Result_Middle_M_L = Servo[1];
//			Result_Middle_M_R = Servo[2];
//		  	Result_R = Servo[3];
//			Result_Middle_M = Servo[4];// 中间电感值
			
			Result_L = (uint16)((Result_L/ (Max1 * 1.00) ) *100);
			Result_Middle_M_L = (uint16)((Result_Middle_M_L/ (Max2 * 1.00)) *100);
			Result_Middle_M_R = (uint16)((Result_Middle_M_R/ (Max4 * 1.00)) *100);
			Result_Middle_M = (uint16)((Result_Middle_M/ (Max3 * 1.00)) *100);
            Result_R = (uint16)((Result_R/ (Max5 * 1.00) ) *100);  
	
//			imu660ra_get_gyro();
//			Yaw_Angular_Speed = imu660ra_gyro_transition((float)imu660ra_gyro_z - imu_data.gyro_z);// 陀螺仪角速度



		// ========== 环岛六阶段角度积分与里程计数 ==========
		  imu660ra_get_gyro();
		  Yaw_Angular_Speed = imu660ra_gyro_transition((float)imu660ra_gyro_z - imu_data.gyro_z);

		  // 预处理阶段：不做角度积分，保持正常循迹
		//  if(Round_State == ROUND_NONE)
		//  {
		//      // 正常循迹，不处理
		//  }
		  // 入环阶段 0°~60°
			  // ========== 环岛里程计数处理 ==========
		  // 预处理阶段里程计数
		  if(Round_State == ROUND_PRE)
		  {
			  Second_distance_calculate();
			  Round_Pre_Distance += Second_encoder_ave;
			  if(Round_Pre_Distance >= Round_Params.pre_distance_thres)
			  {
				  // 里程达到阈值，开启偏航角度积分，进入入环阶段
				  Round_State = ROUND_ENTRY;
				  Round_Pre_Distance = 0;
				  Yaw_Angle = 0;
				  Buzzer_On();
			  }
		  }
		  else if(Round_State == ROUND_ENTRY)
		  {
			  Yaw_Angle += (imu660ra_gyro_transition((float)imu660ra_gyro_z - imu_data.gyro_z)) * 0.01;
			  if(fabs(Yaw_Angle) >= Round_Params.entry_angle_end)
			  {
				  Round_State = ROUND_INSIDE;
				  Yaw_Angle = Round_Params.entry_angle_end;
			  }
		  }
		  // 环内阶段 60°~270°
		  else if(Round_State == ROUND_INSIDE)
		  {
			  Yaw_Angle += (imu660ra_gyro_transition((float)imu660ra_gyro_z - imu_data.gyro_z)) * 0.01;
			  if(fabs(Yaw_Angle) >= Round_Params.inside_angle_end)
			  {
				  Round_State = ROUND_EXIT;
				  Yaw_Angle = Round_Params.inside_angle_end;
			  }
		  }
		  // 出环阶段 270°~330°
		  else if(Round_State == ROUND_EXIT)
		  {
			  Yaw_Angle += (imu660ra_gyro_transition((float)imu660ra_gyro_z - imu_data.gyro_z)) * 0.01;
			  if(fabs(Yaw_Angle) >= Round_Params.exit_angle_end)
			  {
				  Round_State = ROUND_EXIT_AFTER;
				  Yaw_Angle = 0;
				  Round_Exit_Distance = 0; 
			  }
		  }

		  // 出环后里程计数
		  else if(Round_State == ROUND_EXIT_AFTER)
		  {
			  Second_distance_calculate();
			  Round_Exit_Distance += Second_encoder_ave;
			  if(Round_Exit_Distance >= Round_Params.exit_distance_thres)
			  {
				  // 里程超过10cm，彻底退出环岛状态
				  Round_State = ROUND_NONE;
				  Round_Exit_Distance = 0;
				  Buzzer_Off();
			  }
		  }
			
			turn_cmd = Turn_Control_PID(Result_L,Result_Middle_M_L,Result_Middle_M_R,Result_R,Result_Middle_M);
			Turn_Output = turn_cmd;		
			
			Differential_Speed_Control(Turn_Output);// 差速分配
            
			Motor_PID(SpeedTarget_L,motor_speed_L,1.4,0.8,0,Left);// 左轮速度环PID //5,2.5,1.25/1.4,0.8,0/3,1.5,0.3
			Motor_PID(Target_Right1,motor_speed_R,1.7,0.6,0,Right);// 右轮速度环PID //4.8,2.4,1.25/1.7,0.6,0/2.5,0.8,0.2
	
			conservation = PID_Conservation(Result_L,Result_Middle_M_L,Result_Middle_M_R,Result_R);// 保护判断
			
			
			if(conservation == 0 && LCD_Config == 0)
			{
				Motor_PWM_set_L();// 输出左轮PWM
				Motor_PWM_set_R();// 输出右轮PWM
			}//PID	
			
//				if(Round_Config1 == 1)
//				{
//					imu660ra_get_gyro();
//					Yaw_Angle += (imu660ra_gyro_transition((float)imu660ra_gyro_z - imu_data.gyro_z)) * 0.01;
//					
////					quaternion_update();// 四元数更新
////					Yaw_Angle = (euler.yaw * 90) / 24.0f;// 角度换算，24 对应 90 度
//					
//					if(fabs(Yaw_Angle) >= 311)//310
//					{
//						Yaw_Angle = 0;
//						Round_Config1 = 0;
//						Round_Config2 = 1;
//						Buzzer_Off();
//					}
//				}
//				else if( Round_Config2 == 1 )// 环岛退出阶段
//				{
//					Second_distance_calculate();
//					if(Second_distance >= 2500)//2000
//					{
//						Round_Config2 = 0;// 退出环岛状态
//						Second_distance = 0;
//					}
//				}
			
			ctimer_count_clean(CTIM0_P34);
			ctimer_count_clean(CTIM3_P04);
			
			Timer_Config = 1;
}


void TM2_Isr() interrupt 12
{
	TIM2_CLEAR_FLAG;  // UART2 使用 TIM2 作波特率发生器，正常不再开启 TIM2 周期中断
}

void TM3_Isr() interrupt 19
{
	TIM3_CLEAR_FLAG; // 清除定时器 3 中断标志

}


void TM4_Isr() interrupt 20
{
	TIM4_CLEAR_FLAG; // 清除定时器 4 中断标志
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