#include "headfile.h"
#pragma float64

 extern float SpeedMeasure_L,SpeedMeasure_R;
 extern float SpeedTarget_L,Target_Right1;
 extern float Yaw_Angular_Speed;
 extern uint16 Result_L, Result_Middle_M_L,Result_Middle_M_R,Result_R;// 电磁值
 extern float Motor_L_output, Motor_output_R;
 extern float Angle_Speed_Output1;
 extern float Turn_Output;
 extern float Angle_Speed_error1;
 extern float xdata Turn_Cmd1;

void Uarts_Init()
{
	uart_init(UART_4, UART4_RX_P02, UART4_TX_P03, 115200, TIM_2);
    //uart_putstr(UART_4, "UART2 Hello,world");
}

void Uarts_Send()
{
    //uart_putstr(UART_4, "UART2 Hello,world");
}

void SCI_Send_Datas(UARTN_enum uart_num)// 上位机查看波形和变量的函数
{
	  int i, j;
    static unsigned short int send_data[3][4] = { { 0 }, { 0 }, { 0 } };
    short int checksum = 0;
    unsigned char xorsum = 0, high, low;
		
    int16 target_L = 0,target_R = 0,measure_L = 0,measure_R = 0;
		
    target_L = (int16) SpeedTarget_L;
    target_R = -(int16) Target_Right1;
    measure_L = (int16) SpeedMeasure_L;
    measure_R = -(int16) SpeedMeasure_R;


    send_data[0][0] = (unsigned short int) (measure_L);
    send_data[0][1] = (unsigned short int) (measure_R);
    send_data[0][2] = (unsigned short int) (target_L);
    send_data[0][3] = (unsigned short int) (target_R);

    send_data[1][0] = (unsigned short int) (Yaw_Angular_Speed);
    send_data[1][1] = (unsigned short int) (0);
    send_data[1][2] = (unsigned short int)(0);
    send_data[1][3] = (unsigned short int)(0);

    send_data[2][0] = (unsigned short int)(Turn_Cmd1);
    send_data[2][1] = (unsigned short int)(0);
    send_data[2][2] = (unsigned short int)(0);
    send_data[2][3] = (unsigned short int)(0);
		
    uart_putchar(UART_4,'S');
    uart_putchar(UART_4,'T');
		
    for (i = 0; i < 3; i++)
        for (j = 0; j < 4; j++)
        {
            low = (unsigned char) (send_data[i][j] & 0x00ff);
            high = (unsigned char) (send_data[i][j] >> 8u);
            uart_putchar(uart_num, low);
            uart_putchar(uart_num, high);
            checksum += low;
            checksum += high;
            xorsum ^= low;
            xorsum ^= high;
        }
		
    uart_putchar(uart_num, (unsigned char) (checksum & 0x00ff));
    uart_putchar(uart_num, xorsum);

}