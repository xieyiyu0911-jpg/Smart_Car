#include "isr.h"

void LED_Init()
{
    gpio_init(,GPO,0,GPIO_PIN_CONFIG);
    gpio_init(E8,GPO,0,GPIO_PIN_CONFIG);
    gpio_init(E12,GPO,0,GPIO_PIN_CONFIG);
    gpio_init(E13,GPO,0,GPIO_PIN_CONFIG);

    system_delay_ms(1000);
    gpio_init(E10,GPO,1,GPIO_PIN_CONFIG);
    gpio_init(E8,GPO,1,GPIO_PIN_CONFIG);
    gpio_init(E12,GPO,1,GPIO_PIN_CONFIG);
    gpio_init(E13,GPO,1,GPIO_PIN_CONFIG);
}

void LED_On1()
{
    gpio_set_level(E8, 0);
}
void LED_Off1()
{
    gpio_set_level(E8, 1);
}

void LED_On2()
{
    gpio_set_level(E10, 0);
}
void LED_Off2()
{
    gpio_set_level(E10, 1);
}

void LED_On3()
{
    gpio_set_level(E12, 0);
}
void LED_Off3()
{
    gpio_set_level(E12, 1);
}

void LED_On4()
{
    gpio_set_level(E13, 0);
}
void LED_Off4()
{
    gpio_set_level(E13, 1);
}