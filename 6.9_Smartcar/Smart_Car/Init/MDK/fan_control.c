#include "headfile.h"
#include "fan_control.h"


//电调频率为50HZ，总周期为20ms，1ms时不转，占空比为500；2ms时满转，占空比为1000

uint8 xdata Element_Config = 0;
uint8 xdata Element_State = 0;

float xdata Flat_distance = 0;
float xdata Flat_encoder_ave = 0;


extern volatile uint16 Result_Middle_M;
extern float xdata Second_distance;
extern uint8 xdata circle_config;

static void element_advance(void)
{
    Element_Config++;
    if(Element_Config >= 4)
    {
        Element_Config = 0;
    }
}


void fan_flatground(void)//在平地时给20%的占空比防止小车飘移
{
    pwm_duty(PWMB_CH1_P20, 600);
}

// 具体元素的顺序根据赛场调整
void Element_Update(void)
{
    if(seekfree_assistant_parameter[7] != 0)
    {
        pwm_duty(PWMB_CH1_P20, 500);
        return;
    }

    if(Element_State == 0)
    {
        if(Result_Middle_M >= 99 && Flat_distance >= 1000)// 中间电感很大而且过往1m全是平地
        {
            switch(Element_Config)
            {
                case 0:
					// 第一次：墙
                    Second_distance = 0;
                    Flat_distance = 0;
                    Element_State = 1;
                    break;

                case 1:
					// 第二次：环岛
                    circle_config = 1;
                    Round_State = ROUND_PRE;
                    Round_Pre_Distance = 0;
                    Second_distance = 0;
                    Element_State = 2;
                    break;

                case 2:
					// 第三次：圆筒
                    Second_distance = 0;
                    Flat_distance = 0;
                    Element_State = 3;
                    break;

                case 3:
					// 第四次：环岛
                    circle_config = 1;
                    Round_State = ROUND_PRE;
                    Round_Pre_Distance = 0;
                    Second_distance = 0;
                    Element_State = 2;
                    break;

                default:
                    Element_Config = 0;
                    break;
            }
        }
    }
    else if(Element_State == 1)// 上墙
    {
        Second_distance_calculate();
        // 编码器3m后，检查最近1m俯仰角是否为平地
        if(Second_distance >= 3000 && Flat_distance >= 1000)
        {
            Second_distance = 0;
            Flat_distance = 0;
            element_advance();
            Element_State = 0;
        }
    }
    else if(Element_State == 2)// 进环岛
    {
		// 等待Round_State状态机完成环岛
        if(Round_State == ROUND_NONE)
        {
            Flat_distance = 0;
            element_advance();
            Element_State = 0;
        }
    }
    else if(Element_State == 3)// 圆筒
    {
        Second_distance_calculate();
        // 编码器3m后，检查最近1m俯仰角是否为平地
        if(Second_distance >= 3000 && Flat_distance >= 1000)
        {
            Second_distance = 0;
            Flat_distance = 0;
            element_advance();
            Element_State = 0;
        }
    }

    if(Element_State == 1 || Element_State == 3)// 只有在上墙和圆筒的时候负压变大
    {
        pwm_duty(PWMB_CH1_P20, 650);
    }
    else
    {
        pwm_duty(PWMB_CH1_P20, 600);
    }
}
