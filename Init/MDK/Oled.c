#include "headfile.h"
#pragma float64

 extern volatile int16 motor_speed_L,motor_speed_R;
 extern volatile float SpeedTarget_L ,Target_Right1;
 extern float Motor_L_output,Motor_output_R;
 extern uint16 Result_L,Result_Middle_M_L,Result_Middle_M_R,Result_R,Result_Middle_M;
 extern float Angle;
 extern float Motor_L_output;
 extern float Motor_output_R;
 extern float xdata Source;

 extern float xdata First_distance;
 extern float Yaw_Angular_Speed;//Æ«º½½ÇÊý¾Ý
 extern float xdata Servo_PID_P2;

 extern uint8 xdata read_buff1[8];
 extern uint8 xdata read_buff2[4];

 extern float xdata Servo_GKD;
 extern int16 xdata I_Count_Max1;
 extern int16 xdata Change_Config_Number;
 extern int16 xdata Source_Start;
 extern float xdata Servo_PID_D;
 extern int16 xdata Change_Config_Number1;
 extern float xdata Second_distance;

void OLED_LCD_Init()
{
    lcd_init();
    lcd_clear(WHITE);
}


void Oled_Test()
{
	lcd_showstr(0,0,"Hello World");
}

void OLED_LCD_Show()
{
//	lcd_showstr(20,0,"ADC0: ");
//	lcd_showint32(65,0,Result_Middle_M,3);
	lcd_showstr(0,1,"ADC0: ");
	lcd_showint32(40,1,Result_L,3);
	lcd_showstr(0,3,"ADC1: ");
	lcd_showint32(40,3,Result_Middle_M_L,3);
	lcd_showstr(0,5,"ADC5: "); 
	lcd_showint32(40,5,Result_Middle_M_R,3); 
	lcd_showstr(0,7,"ADC6: ");
	lcd_showint32(40,7,Result_R,3);
	
//	lcd_showstr(0,3,"Start:");
//	lcd_showuint16(45,3,(uint16)Source_Start);
//	lcd_showstr(0,4,"+:");
//	lcd_showuint8(10,4,read_buff2[0]);
//	lcd_showstr(55,4,"-:");
//	lcd_showuint8(70,4,read_buff2[2]);
	
//	lcd_showstr(0,5,"Servo_D");
//	lcd_showfloat(60,5,Servo_PID_D,2,1);
//	lcd_showstr(0,6,"+:");
//	lcd_showuint8(10,6,read_buff2[1]);
//	lcd_showstr(55,6,"-:");
//	lcd_showuint8(70,6,read_buff2[3]);
	
//	lcd_showstr(0,7,"Source:");
//	lcd_showfloat(60,7,Source,3,1);//Source
	
//	lcd_showstr(0,8,"distance:");
//	lcd_showfloat(75,8,Second_distance,5,0);
	
//	lcd_showstr(0,9,"Now_Config:");
//	lcd_showuint8(80,9,(uint8)Change_Config_Number1);
		
		switch(Change_Config_Number1)
	{
		case 0:
		{
			lcd_showstr(105,3,"<");
			break;
		}
		case 1:
		{
			lcd_showstr(105,5,"<");
			break;
		}
	}

//	lcd_showstr(0,3,"route:");
//	lcd_showfloat(70,3,First_distance,5,1);
	
//	lcd_showstr(0,4,"Speed_L: ");
//	lcd_showfloat(0,5,SpeedTarget_L,3,1);
//	lcd_showint16(55,5,motor_speed_L);
	
//	lcd_showstr(0,6,"Speed_R: ");
//	lcd_showfloat(0,7,Target_Right1,3,1);
//	lcd_showint16(55,7,motor_speed_R);
	
//	lcd_showstr(60,8,"Source:");
//	lcd_showfloat(60,9,Source,4,1);
}

void OLED_LCD_Change()
{
	
	lcd_showstr(0,0,"Change_Parameter");
	
	lcd_showstr(0,1,"P2:");
	lcd_showfloat(25,1,Servo_PID_P2,1,5);
	lcd_showstr(0,2,"+:");
	lcd_showuint8(10,2,read_buff1[0]);
	lcd_showstr(55,2,"-:");
	lcd_showuint8(70,2,read_buff1[4]);
	
	lcd_showstr(0,3,"Speed:");
	lcd_showfloat(55,3,SpeedTarget_L,3,1);
	lcd_showstr(0,4,"+:");
	lcd_showuint8(10,4,read_buff1[1]);
	lcd_showstr(55,4,"-:");
	lcd_showuint8(70,4,read_buff1[5]);
	
	lcd_showstr(0,5,"I_Max:");
	lcd_showint16(55,5,I_Count_Max1);
	lcd_showstr(0,6,"+:");
	lcd_showuint8(10,6,read_buff1[2]);
	lcd_showstr(55,6,"-:");
	lcd_showuint8(70,6,read_buff1[6]);
	
	lcd_showstr(0,7,"GKD:");
	lcd_showfloat(35,7,Servo_GKD,1,5);
	lcd_showstr(0,8,"+:");
	lcd_showuint8(10,8,read_buff1[3]);
	lcd_showstr(55,8,"-:");
	lcd_showuint8(70,8,read_buff1[7]);
	
	lcd_showstr(0,9,"Now_Config:");
	lcd_showuint8(80,9,(uint8)Change_Config_Number);
	
	switch(Change_Config_Number)
	{
		case 0:
		{
			lcd_showstr(105,1,"<");
			break;
		}
		case 1:
		{
			lcd_showstr(105,3,"<");
			break;
		}
		case 2:
		{
			lcd_showstr(105,5,"<");
			break;
		}
			case 3:
		{
			lcd_showstr(105,7,"<");
			break;
		}
	}
}

