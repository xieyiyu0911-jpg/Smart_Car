#include "headfile.h"
#include "Pwm.h"

void Pwms_Init()
{
    // 左电机方向脚和PWM初始化
    gpio_pull_set(P1_0,PULLUP);
    gpio_mode(P1_0,GPO_PP);
    MOTOR_L_DIR_PIN = 1;
    pwm_init(MOTOR_L_PWM_PIN, 15000, 0);

    // 右电机方向脚和PWM初始化    
    gpio_pull_set(P2_4,PULLUP);
    gpio_mode(P2_4,GPO_PP);
    MOTOR_R_DIR_PIN = 1;
    pwm_init(MOTOR_R_PWM_PIN, 15000, 0);
}

void PWM_Motor_Change_L()
{
    MOTOR_L_DIR_PIN = 1;
}

void PWM_Motor_Change_R()
{
    MOTOR_R_DIR_PIN = 1;
}