#include "headfile.h"


void Button_Init()
{
	gpio_pull_set(P3_3,PULLUP);
	gpio_mode(P3_3,GPI_IMPEDANCE);//Button1,上拉输入模式，刚开始读到的电平为高电平，为1
	gpio_pull_set(P3_2,PULLUP);
	gpio_mode(P3_2,GPI_IMPEDANCE);//Button2
	
}

void Button_Test()
{
	if(P33 == 0 || P32 == 0)
	{
		Buzzer_On();
		delay_ms(100);
		Buzzer_Off();
	}
}
