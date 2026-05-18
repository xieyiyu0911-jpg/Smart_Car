#include "headfile.h"
#include "Time_Pulse.h"

#define L_A_PIN CTIM0_P34
#define R_A_PIN CTIM3_P04

void Time_Pulse_Init()
{
	ctimer_count_init(L_A_PIN);//L_A
	ctimer_count_init(R_A_PIN);//R_A
	gpio_pull_set(P3_5,NOPULL);  //L_DIR
	gpio_mode(P3_5,GPI_IMPEDANCE);
	gpio_pull_set(P5_3,NOPULL);  //R_DIR
	gpio_mode(P5_3,GPI_IMPEDANCE);

	pit_timer_ms(TIM_4,10);//UART2 必须使用 TIM2 作波特率发生器，控制周期改用 TIM4
}