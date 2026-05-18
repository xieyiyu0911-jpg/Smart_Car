#ifndef __PWM_H_
#define __PWM_H_

#define MOTOR_R_PWM_PIN  PWMA_CH2P_P62
#define MOTOR_L_PWM_PIN  PWMA_CH4P_P66

#define MOTOR_R_DIR_PIN P60
#define MOTOR_L_DIR_PIN P64

void Pwms_Init(void);
void PWM_Motor_Change_L(void);
void PWM_Motor_Change_R(void);

#endif