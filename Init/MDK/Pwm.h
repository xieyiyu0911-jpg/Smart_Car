#ifndef __PWM_H_
#define __PWM_H_

#define MOTOR_R_PWM_PIN  PWMA_CH2N_P13
#define MOTOR_L_PWM_PIN  PWMA_CH4P_P26

#define MOTOR_R_DIR_PIN P24
#define MOTOR_L_DIR_PIN P10

void Pwms_Init(void);
void PWM_Motor_Change_L(void);
void PWM_Motor_Change_R(void);

#endif