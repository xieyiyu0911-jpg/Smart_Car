#ifndef _DMX_ENCODER_H_
#define _DMX_ENCODER_H_

// 编码器脉冲数引脚枚举
typedef enum
{
    ENCODER0_P34,
    ENCODER1_P35,
    ENCODER2_P12,
    ENCODER3_P04,
    ENCODER4_P06,
} ENCODER_pin_enum;

/**
*
* @brief    带方向输出编码器初始化
* @param    count_pin			捕捉脉冲数的引脚,根据dmx_encoder.h中枚举查看
* @param    dir_pin				捕捉方向的引脚,普通IO口即可
* @return   void
* @notes    一个捕捉脉冲数的引脚占用一个定时计数器资源,即如果用定时器0就不能用ENCODER0_P34采集编码器脉冲
* @notes    ENCODER0_P34对应定时器0,ENCODER1_P35对应定时器1,ENCODER2_P12对应定时器2
* @notes    ENCODER3_P04对应定时器3,ENCODER4_P06对应定时器4
* Example:  init_encoder(ENCODER0_P34,GPIO_P35);	// 编码器初始化,脉冲数捕捉引脚为ENCODER0_P34,方向引脚为P35
*
**/
void init_encoder(ENCODER_pin_enum count_pin, PIN_enum dir_pin);

/**
*
* @brief    带方向输出编码器数据获取
* @param    count_pin			捕捉脉冲数的引脚,根据dmx_encoder.h中枚举查看
* @return   unsigned int	返回编码器的脉冲数,
* @notes    方向可通过方向引脚的电平判断
* Example:  get_encoder_count(ENCODER0_P34);	// 返回编码器ENCODER0_P34的脉冲数
*
**/
unsigned int get_encoder_count(ENCODER_pin_enum count_pin);

/**
*
* @brief    带方向输出编码器计数值清除
* @param    count_pin			捕捉脉冲数的引脚,根据dmx_encoder.h中枚举查看
* @return   void
* @notes    采集完编码器脉冲数后要调用此函数
* Example:  get_encoder_count(ENCODER0_P34);
*
**/
void clean_encoder_count(ENCODER_pin_enum count_pin);

#endif