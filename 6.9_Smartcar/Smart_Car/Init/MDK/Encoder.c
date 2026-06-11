#include "isr.h"
#include "Encoder.h"

void init_encoder(ENCODER_pin_enum count_pin, PIN_enum dir_pin)
{
		gpio_pull_set(dir_pin,NOPULL);
		gpio_mode(dir_pin,GPI_IMPEDANCE);//Button2
		
//    init_gpio(dir_pin, IN_HIZ, 0);
    switch(count_pin)
    {
    case ENCODER0_P34:
//        init_gpio(GPIO_P34, IN_HIZ, 0);
				gpio_pull_set(P3_4,NOPULL);
				gpio_mode(P3_4,GPI_IMPEDANCE);
		
				T0_CT = 1;
        TL0 = 0xFF;
        TH0 = 0xFF;
        TR0 = 1;
        ET0 = 1;
        break;
		
    case ENCODER1_P35:
        //init_gpio(GPIO_P35, IN_HIZ, 0);
				gpio_pull_set(P3_5,NOPULL);
				gpio_mode(P3_5,GPI_IMPEDANCE);
		
        T1_CT = 1;
        TL1 = 0xFF;
        TH1 = 0xFF;
        TR1 = 1;
        ET1 = 1;
        break;
		
    case ENCODER2_P12:
//        init_gpio(GPIO_P12, IN_HIZ, 0);
				gpio_pull_set(P1_2,NOPULL);
				gpio_mode(P1_2,GPI_IMPEDANCE);
		
        T2CT = 1;
        T2L = 0xFF;
        T2H = 0xFF;
        T2R = 1;
        ET2 = 1;
        break;
		
    case ENCODER3_P04:
//        init_gpio(GPIO_P04, IN_HIZ, 0);
				gpio_pull_set(P0_4,NOPULL);
				gpio_mode(P0_4,GPI_IMPEDANCE);
		
        T4T3M |= 0x0C;
        T3L = 0xFF;
        T3H = 0xFF;
        ET3 = 1;
        break;
		
    case ENCODER4_P06:
//        init_gpio(GPIO_P06, IN_HIZ, 0);
				gpio_pull_set(P0_6,NOPULL);
				gpio_mode(P0_6,GPI_IMPEDANCE);
		
        T4T3M |= 0xC0;
        T4L = 0xFF;
        T4H = 0xFF;
        ET4 = 1;
        break;
    }
}

/**
*
* @brief    带方向输出编码器数据获取
* @param    count_pin			捕捉脉冲数的引脚,根据dmx_encoder.h中枚举查看
* @return   unsigned int	返回编码器的脉冲数,
* @notes    方向可通过方向引脚的电平判断
* Example:  get_encoder_count(ENCODER0_P34);	// 返回编码器ENCODER0_P34的脉冲数
*
**/
unsigned int get_encoder_count(ENCODER_pin_enum count_pin)
{
    unsigned int count;
    switch(count_pin)
    {
    case ENCODER0_P34:
        count = TL0;
        count = TH0 << 8 | count;
        break;
    case ENCODER1_P35:
        count = TL1;
        count = TH1 << 8 | count;
        break;
    case ENCODER2_P12:
        count = T2L;
        count = T2H << 8 | count;
        break;
    case ENCODER3_P04:
        count = T3L;
        count = T3H << 8 | count;
        break;
    case ENCODER4_P06:
        count = T4L;
        count = T4H << 8 | count;
        break;
    }
    return count;
}

/**
*
* @brief    带方向输出编码器计数值清除
* @param    count_pin			捕捉脉冲数的引脚,根据dmx_encoder.h中枚举查看
* @return   void
* @notes    采集完编码器脉冲数后要调用此函数
* Example:  get_encoder_count(ENCODER0_P34);
*
**/
void clean_encoder_count(ENCODER_pin_enum count_pin)
{
    switch(count_pin)
    {
    case ENCODER0_P34:
        TR0 = 0;
        TL0 = 0;
        TH0 = 0;
        TR0 = 1;
        break;
    case ENCODER1_P35:
        TR1 = 0;
        TL1 = 0;
        TH1 = 0;
        TR1 = 1;
        break;
    case ENCODER2_P12:
        T2R = 0;
        T2L = 0;
        T2H = 0;
        T2R = 1;
        break;
    case ENCODER3_P04:
        T3R = 0;
        T3L = 0;
        T3H = 0;
        T3R = 1;
        break;
    case ENCODER4_P06:
        T4R = 0;
        T4L = 0;
        T4H = 0;
        T4R = 1;
        break;
    }
}