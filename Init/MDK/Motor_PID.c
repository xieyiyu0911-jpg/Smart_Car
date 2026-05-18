#include "isr.h"
#include "Motor_PID.h"
#pragma float64

#define TURN_CMD_MAX           (90)
#define TURN_CMD_MIN           (-90)
#define ANGLE_SPEED_OUTPUT_MAX (90)
#define ANGLE_SPEED_OUTPUT_MIN (-90)
#define ANGLE_I_COUNT_MAX      (800)
#define MOTOR_PWM_MAX          (5000)
#define SPEED_TARGET_MAX       (1000)

float Motor_L_output = 0;
float Motor_output_R = 0;

uint32 xdata Left_Round_Config1 = 0;
uint32 xdata Left_Round_Config2 = 0;
uint32 xdata Right_Round_Config1 = 0;
uint32 xdata Right_Round_Config2 = 0;

float xdata First_encoder_ave = 0, First_distance = 0;
float Turn_Cmd1 = 0;
float error1 = 0;
uint32 xdata Round_Config1 = 0;
uint32 xdata Round_Config2 = 0;
uint32 xdata Cross_Config = 0;
float xdata Servo_PID_P2 = 0;
float xdata Servo_GKD = 0;
int16 xdata I_Count_Max1 = 0;
float xdata Servo_PID_D = 0;

uint8 xdata circle_config = 0;



 extern volatile int16 motor_speed_L;
 extern volatile int16 motor_speed_R;
 extern uint8 xdata read_buff1[8];
 extern uint8 xdata read_buff2[4];
 extern float Angle_Speed_Output1;
 extern float Yaw_Angular_Speed;
 extern float xdata Yaw_Angle;
 extern float xdata Source;
 extern float xdata Second_distance;
 extern float xdata Second_encoder_ave;
 extern float SpeedTarget_L, Target_Right1;
 extern float SpeedMeasure_L, SpeedMeasure_R;
 extern float Turn_Output;

// 电磁全丢线或上位机急停时，直接清零目标速度并关闭 PWM
uint16 PID_Conservation(uint16 Result_L,uint16 Result_Middle_M_L,uint16 Result_Middle_M_R,uint16 Result_R)
{
    if ((Result_L <= 5 && Result_Middle_M_L <= 5 && Result_Middle_M_R <= 5 && Result_R <= 5) || (seekfree_assistant_parameter[7] == 1))
    {
        SpeedTarget_L = 0;
        Target_Right1 = 0;
        MOTOR_L_DIR_PIN = 1;
        MOTOR_R_DIR_PIN = 1;
        pwm_duty(MOTOR_L_PWM_PIN, 0);
        pwm_duty(MOTOR_R_PWM_PIN, 0);
        return 1;
    }
    return 0;
}

uint16 Uart_Stop(void)
{
    if (seekfree_assistant_parameter[7] == 1)
    {
        SpeedTarget_L = 0;
        Target_Right1 = 0;
        pwm_duty(MOTOR_L_PWM_PIN, 0);
        pwm_duty(MOTOR_R_PWM_PIN, 0);
        return 1;
    }
    return 0;
}

void First_distance_calculate(void)
{
    First_encoder_ave = (motor_speed_L + motor_speed_R) / 2;
    First_distance += First_encoder_ave;
}

void Second_distance_calculate(void)
{
    Second_encoder_ave = (motor_speed_L + motor_speed_R) / 2;
    Second_distance += Second_encoder_ave;
}

// 中位值平均滤波
uint16 Servo_Measure(int* inductance_array,int times)
{
    int i = 0, min = 0, max = 0, sum = 0;

    min = inductance_array[0];
    max = inductance_array[0];

    for(i = 0; i < times; i++)
    {
        if(inductance_array[i] < min) min = inductance_array[i];
        if(inductance_array[i] > max) max = inductance_array[i];
        sum += inductance_array[i];
    }
    return ((sum - min - max) / (times - 2));
}

// 将电磁误差转换成差速转向命令，输出 turn_cmd，增量式
float Turn_Control_PID(uint16 Result_L,uint16 Result_Middle_M_L,uint16 Result_Middle_M_R,uint16 Result_R,uint16 Result_Middle_M)
{
    float error = 0;
    float turn_cmd = 0;
    static float error_last = 0;
    
	float Servo_P1 = 1.5f;
    float Servo_P2 = 0.0105f;
    float Servo_D = 9.55f;
    float GKD = 0.0075f;
	
    float Result_Left = (float)Result_L;
    float Result_Right = (float)Result_R;
    float Result_Middle_M_Right = (float)Result_Middle_M_R;
    float Result_Middle_M_Left = (float)Result_Middle_M_L;
    float member1 = 0;
    float denominator1 = 0;
    float Vertical_Weight = 0;//垂直分量
    float Denominator_Weight = 0;

	 Servo_P1 = seekfree_assistant_parameter[0];
     Servo_P2 = seekfree_assistant_parameter[1];
     Servo_D = seekfree_assistant_parameter[2];
     GKD = seekfree_assistant_parameter[3];
	 
    Cross_Config = 0;//过十字标志位
	
	circle_config = 0;
	
    // 直道和特殊元素对中间电感的依赖不同，这里动态调整左右/中间权重
    if(Result_Middle_M <= 75)
    {
        Vertical_Weight = 0.75f;
        Denominator_Weight = Vertical_Weight + 0.01f;
    }
    else//靠近环岛的参数，随着中间电感线性增大
    {
        //Vertical_Weight = ((0.85f - 0.66f) / 25.0f) * Result_Middle_M + 4 * 0.66f - 2.55f;
		Vertical_Weight = 0.75f;
        Denominator_Weight = Vertical_Weight + 0.01f;
    }


//	if(Result_Middle_M >= 99 && Round_Config2 == 0)//环岛
//    {
//        Round_Config1 = 1;//进环标志位
//        circle_config = 1;    
//		Second_distance = 0;
//		Buzzer_On();
//    }
	
	//Result_Middle_M_Left >= 35 && Result_Middle_M_Right >= 20 && Result_Left >= 7 && Result_Right >= 7 && Result_Middle_M <= 85
//    else if(Result_Middle_M_Left >= 40 && Result_Middle_M_Right >= 40 && Result_Left >= 10 && Result_Right >= 10 && Result_Middle_M <= 85)//ʮ��·���ص�Ϊ���ȶ�
     if(Result_Middle_M_Left >= 40 && Result_Middle_M_Right >=0 && Result_Left >= 30 && Result_Right >= 15 && Result_Middle_M <= 85)//ʮ��·���ص�Ϊ���ȶ�
    {
        Vertical_Weight = 0.2f;
        Servo_D += 5;//加阻尼
        if(Servo_D >= 42.5f) Servo_D = 42.5f;
        GKD = 0;
        Servo_P2 = 0;
        Servo_P1 = 0.1f;
        Cross_Config = 1;//十字标志位，主要用来让它过了十字之后减速，因为需要拐一个直角弯，容易过冲
    }
    else if(Round_Config1 == 1)//入环在环内走的参数
    {
        Vertical_Weight = 0.85f;
        Denominator_Weight = Vertical_Weight + 0.01f;
		circle_config = 2;
    }
    else if(Round_Config2 == 1)//出环参数
    {
        Vertical_Weight = 0.6f;
        Denominator_Weight = Vertical_Weight + 0.01f;
        Servo_P2 = 0;
        Servo_P1 = 0.1f;
        GKD = 0;
		circle_config = 3;
    }

    // member1 表示方向偏差，denominator1 用于归一化，避免不同强度下误差量级漂移
    member1 = ((1 - Vertical_Weight) * (Result_Left - Result_Right) + Vertical_Weight * (Result_Middle_M_Left - Result_Middle_M_Right));
    denominator1 = ((1 - Vertical_Weight) * (Result_Left + Result_Right) + Denominator_Weight * fabs(Result_Middle_M_Left - Result_Middle_M_Right));

    if(Servo_P2 <= 0) Servo_P2 = 0;
    else if(Servo_P2 >= 0.05f) Servo_P2 = 0.05f;

    if(Servo_D <= 15) Servo_D = 15;
    else if(Servo_D >= 50) Servo_D = 50;

    if(GKD <= 0) GKD = 0;
    else if(GKD >= 0.1f) GKD = 0.1f;

    // 最终转向命令由比例、非线性比例、微分和陀螺仪阻尼共同组成
    error = (member1 / (denominator1 + 0.00001f)) * 90;
    turn_cmd = error * Servo_P1 + Servo_P2 * fabs(error) * error + (error - error_last) * Servo_D + GKD * Yaw_Angular_Speed;
    error_last = error;

    if(turn_cmd >= TURN_CMD_MAX) turn_cmd = TURN_CMD_MAX;
    else if(turn_cmd <= TURN_CMD_MIN) turn_cmd = TURN_CMD_MIN;

    error1 = error;
    Turn_Cmd1 = turn_cmd;
    Servo_PID_P2 = Servo_P2;
    Servo_GKD = GKD;
    Servo_PID_D = Servo_D;

    return turn_cmd;
}

float constrain_float(float amt, float low, float high)
{
    return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

void Angle_PID_Control(float Angle_error,float Angle_P,float Angle_I,float Angle_D,int16 Config)
{
    float P_data = 0, I_data = 0, D_data = 0;
    static float Angle_Speed_error_last = 0;
    static float Angle_I_count = 0;

    if(Config == 1)
    {
        P_data = Angle_P * Angle_error;
        Angle_I_count += Angle_error;

        if(Angle_I_count > ANGLE_I_COUNT_MAX) Angle_I_count = ANGLE_I_COUNT_MAX;
        else if(Angle_I_count < -ANGLE_I_COUNT_MAX) Angle_I_count = -ANGLE_I_COUNT_MAX;

        I_data = Angle_I * Angle_I_count;
        D_data = Angle_D * (Angle_error - Angle_Speed_error_last);
        Angle_Speed_error_last = Angle_error;
        Angle_Speed_Output1 = P_data + I_data + D_data;

        if(Angle_Speed_Output1 > ANGLE_SPEED_OUTPUT_MAX) Angle_Speed_Output1 = ANGLE_SPEED_OUTPUT_MAX;
        else if(Angle_Speed_Output1 < ANGLE_SPEED_OUTPUT_MIN) Angle_Speed_Output1 = ANGLE_SPEED_OUTPUT_MIN;
    }
}

// 把转向命令分配为左右轮目标速度差，实现差速转向
void Differential_Speed_Control(float turn_cmd)
{
    float base_speed = 0;
    float diff_ratio = 0;
    float diff_output = 0;

    // 先取基础速度，再按 turn_cmd 比例拉开左右轮速度差
    base_speed = (SpeedTarget_L + Target_Right1) / 2.0f;
    diff_ratio = turn_cmd / ANGLE_SPEED_OUTPUT_MAX;
    diff_output = diff_ratio * base_speed;

	if(diff_ratio >= 0)
	{
		SpeedTarget_L = base_speed - diff_output;
		Target_Right1 = base_speed + diff_output * 0.4;
	}
	if(diff_ratio < 0)
	{
		SpeedTarget_L = base_speed - diff_output * 0.4;
		Target_Right1 = base_speed + diff_output;
	}
	
    if(SpeedTarget_L < 0) SpeedTarget_L = 0;
    if(Target_Right1 < 0) Target_Right1 = 0;
	
    if(SpeedTarget_L > SPEED_TARGET_MAX) SpeedTarget_L = SPEED_TARGET_MAX;
    if(Target_Right1 > SPEED_TARGET_MAX) Target_Right1 = SPEED_TARGET_MAX;
}

// 左右轮各自做速度闭环，目标速度来自差速分配结果
void Motor_PID(float SpeedTarget,int16 motor_speed,float Motor_P,float Motor_I,float Motor_D,int16 Config)
{
    static float I_count_L = 0, I_count_R = 0;
    static float error_last_L = 0, error_last_R = 0;
    float speed_error = 0;
    float P_data = 0, I_data = 0, D_data = 0;
    int I_Count_Max = 2500;

//    // 转向越激烈，积分上限越小，避免弯道中积分堆积导致出弯过冲
//    I_Count_Max = -fabs(turn_cmd) * (10.0f / 3.0f) + 100 + seekfree_assistant_parameter[0];

//    if(Cross_Config == 1) I_Count_Max = 150;
	
//    if(I_Count_Max >= 2500) I_Count_Max = 1500;
//    else if(I_Count_Max <= 0) I_Count_Max = 0;

    speed_error = SpeedTarget - (float)motor_speed;
    P_data = Motor_P * speed_error;

    if(Config == 1)
    {
        I_count_L += speed_error;
        if(I_count_L >= I_Count_Max) I_count_L = I_Count_Max;
        else if(I_count_L < -I_Count_Max) I_count_L = -I_Count_Max;

        I_data = I_count_L * Motor_I;
        D_data = Motor_D * (speed_error - error_last_L);
        error_last_L = speed_error;
        Motor_L_output = P_data + I_data + D_data;
    }
    else
    {
        I_count_R += speed_error;
        if(I_count_R >= I_Count_Max) I_count_R = I_Count_Max;
        else if(I_count_R < -I_Count_Max) I_count_R = -I_Count_Max;

        I_data = I_count_R * Motor_I;
        D_data = Motor_D * (speed_error - error_last_R);
        error_last_R = speed_error;
        Motor_output_R = P_data + I_data + D_data;
    }

    I_Count_Max1 = I_Count_Max;
}

void Motor_PWM_set_L(void)
{
    if(Motor_L_output >= MOTOR_PWM_MAX) Motor_L_output = MOTOR_PWM_MAX;
    else if(Motor_L_output <= -MOTOR_PWM_MAX)
        Motor_L_output = -MOTOR_PWM_MAX;

    if(Motor_L_output >= 0)
    {
        MOTOR_L_DIR_PIN = 1;//正转
	    pwm_duty(MOTOR_L_PWM_PIN,(uint32)Motor_L_output);
    }
	
    else if(Motor_L_output < 0)
    {
        Motor_L_output = -Motor_L_output;
        MOTOR_L_DIR_PIN = 0;//反转
        pwm_duty(MOTOR_L_PWM_PIN,(uint32)Motor_L_output);
    }
}


void Motor_PWM_set_R(void)
{
    if(Motor_output_R >= MOTOR_PWM_MAX) Motor_output_R = MOTOR_PWM_MAX;
    else if(Motor_output_R <= -MOTOR_PWM_MAX)
        Motor_output_R = -MOTOR_PWM_MAX;

       if(Motor_output_R >= 0)
       {
				MOTOR_R_DIR_PIN = 1;//正转
				pwm_duty(MOTOR_R_PWM_PIN,(uint32)Motor_output_R);
       }
       else if(Motor_output_R < 0)
       {
          Motor_output_R =  -Motor_output_R;
					MOTOR_R_DIR_PIN = 0;//反转
          pwm_duty(MOTOR_R_PWM_PIN,(uint32)Motor_output_R);
       }
}