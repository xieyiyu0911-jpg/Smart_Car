#include "headfile.h"

void Sw_Init()
{
	gpio_pull_set(P0_5,PULLUP);
	gpio_mode(P0_5,GPI_IMPEDANCE);//ùùùù1
	gpio_pull_set(P0_6,PULLUP);
	gpio_mode(P0_6,GPI_IMPEDANCE);//2

	P05 = 1;
	P06 = 1;
}

void Sw_Test()
{
	if(P05 == 0 || P06 == 0)
	{
		Buzzer_On();
		delay_ms(50);
		Buzzer_Off();
	}
}
