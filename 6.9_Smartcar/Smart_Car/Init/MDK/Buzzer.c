#include "headfile.h"

void Buzzer_Init()
{
	gpio_pull_set(buzzer,PULLUP);
	gpio_mode(buzzer,GPO_PP);
	buzzer_IO = 0;
}

void Buzzer_On()
{
	buzzer_IO = 1;
}

void Buzzer_Off()
{
	buzzer_IO = 0;
}
