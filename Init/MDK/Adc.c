#include "headfile.h"

void ADCs_Init()
{
	adc_init(ADC_P11,ADC_SYSclk_DIV_8);//ADC1,时钟频率先选择8，速度 = 频率 / 分频系数，不能太快也不能太慢
	adc_init(ADC_P00,ADC_SYSclk_DIV_8);
	adc_init(ADC_P01,ADC_SYSclk_DIV_8);
	adc_init(ADC_P05,ADC_SYSclk_DIV_8);
	adc_init(ADC_P06,ADC_SYSclk_DIV_8);
	//adc_init(ADC_P11,ADC_SYSclk_DIV_8);
	
}
