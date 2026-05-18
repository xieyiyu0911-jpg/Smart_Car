#ifndef __MOTOR_PID_H_
#define __MOTOR_PID_H_

#define MOTOR_R_PWM_PIN  PWMA_CH2P_P62
#define MOTOR_L_PWM_PIN  PWMA_CH4P_P66

#define MOTOR_R_DIR_PIN P60
#define MOTOR_L_DIR_PIN P64

uint16 PID_Conservation(uint16 Result_L,uint16 Result_Middle_M_L,uint16 Result_Middle_M_R, uint16 Result_R);
uint16 Uart_Stop(void);

uint16 Servo_Measure(int* inductance_array,int times);

void First_distance_calculate(void);
void Second_distance_calculate(void);

float Turn_Control_PID(uint16 Result_L,uint16 Result_Middle_M_L,uint16 Result_Middle_M_R,uint16 Result_R,uint16 Result_Middle_M);
float constrain_float(float amt, float low, float high);
void Angle_PID_Control(float Angle_error,float Angle_P,float Angle_I,float Angle_D,int16 Config);

void Differential_Speed_Control(float turn_cmd);
void Motor_PID(float SpeedTarget,int16 motor_speed,float Motor_P,float Motor_I,float Motor_D,int16 Config);

void Motor_PWM_set_L(void);
void Motor_PWM_set_R(void);

#endif