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

// 模糊 PID 参数：error 和 error_change 的最大范围
// error_change 单位 °/tick(10ms), 800°/s 实际角速度 → 8°/tick
#define FUZZY_E_MAX   90.0f
#define FUZZY_EC_MAX  8.0f

// 模糊规则表：使用语言值 (-3=NB, -2=NM, -1=NS, 0=ZO, 1=PS, 2=PM, 3=PB)
// 行 = E(NB→PB)，列 = EC(NB→PB)，比例因子将语言值映射为实际调整量
//#define FUZZY_KP_SCALE 0.2f    // 语言值 ×0.2 = 实际 ΔKp,  范围 [-0.6, +0.6]
//#define FUZZY_KD_SCALE 2.333f  // 语言值 ×2.333 = 实际 ΔKd, 范围 [-7, +7]

// Kp 规则表：左上角(大偏差+恶化)大力纠正，右下角(大偏差+回正)减力防超调
int8 code rule_Kp[7][7] = {
    //  EC: NB  NM  NS  ZO  PS  PM  PB
    { 3,  3,  2,  2,  1,  0,  0}, // E: NB
    { 3,  3,  2,  1,  1,  0, -1}, // E: NM
    { 2,  2,  2,  1,  0, -1, -1}, // E: NS
    { 2,  2,  1,  0, -1, -2, -2}, // E: ZO
    { 1,  1,  0, -1, -2, -2, -2}, // E: PS
    { 1,  0, -1, -2, -2, -3, -3}, // E: PM
    { 0,  0, -1, -2, -2, -3, -3}, // E: PB
};

// Kd 规则表：同号(恶化)→负值减阻尼, 异号(回正)→正值加阻尼。反对称
int8 code rule_Kd[7][7] = {
    //  EC: NB   NM   NS   ZO   PS   PM   PB
    {-2, -1,  0,  0,  1,  2,  3}, // E: NB
    {-2, -1,  0,  0,  1,  1,  2}, // E: NM
    {-1, -1,  0,  0,  0,  1,  1}, // E: NS
    { 0,  0,  0,  0,  0,  0,  0}, // E: ZO
    {-1, -1,  0,  0,  0,  1,  1}, // E: PS
    {-2, -1, -1,  0,  0,  1,  2}, // E: PM
    {-3, -2, -1,  0,  0,  1,  2}, // E: PB
};

float xdata Motor_L_output = 0;
float xdata Motor_output_R = 0;

uint32 xdata Left_Round_Config1 = 0;
uint32 xdata Left_Round_Config2 = 0;
uint32 xdata Right_Round_Config1 = 0;
uint32 xdata Right_Round_Config2 = 0;

float xdata First_encoder_ave = 0, First_distance = 0;
float xdata Turn_Cmd1 = 0;
float xdata error1 = 0;
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
extern float xdata Source;
extern float xdata Second_distance;
extern float xdata Second_encoder_ave;
extern float SpeedTarget_L, Target_Right1;
extern float SpeedMeasure_L, SpeedMeasure_R;
extern float Turn_Output;
 
float error_change = 0;
float error = 0;

// ========== 环岛处理变量 ==========
  Round_State_TypeDef Round_State = ROUND_NONE;  // 环岛状态机
  uint8 Round_Direction = 0;                      // 环岛方向：0=左环，1=右环

  // 里程计数
  float Round_Pre_Distance = 0;                  // 预处理阶段里程
  float Round_Exit_Distance = 0;                 // 出环后里程

  // 环岛参数配置
  const Round_Config_TypeDef Round_Params = {
      0,     // pre_adc_thres_L
      100,     // pre_adc_thres_M
      0,     // pre_adc_thres_R
      95,     // entry_adc_thres
      60,     // entry_angle_end
      270,    // inside_angle_end
      330,    // exit_angle_end
      1500,    // pre_distance_thres
      5000,   // exit_distance_thres
      1.5     // entry_amplify
  };

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

// 将连续值量化，返回整数索引和用于插值的小数部分
static uint8 quantize_frac(float val, float vmax, float *frac)
{
    static float xdata ratio, xdata idx_f;// ratio=归一化值, idx_f=连续索引
    static uint8 xdata idx;// idx=整数索引(0-6)

    ratio = val / vmax;//归一化，映射到[-1,1]
	
	//限幅
    if(ratio >  1.0f) ratio =  1.0f;
    if(ratio < -1.0f) ratio = -1.0f;
	
	//将归一化值映射到连续索引
    idx_f = ratio * 3.0f + 3.0f;
    idx = (uint8)idx_f;
    if(idx > 5) idx = 5;
    *frac = idx_f - (float)idx;//小数部分即隶属度
    return idx;
}

// 模糊 PID 调整：双线性插值 + 比例因子
void Fuzzy_PID_Adjust(float error, float error_change, float *delta_Kp, float *delta_Kd)
{
    static float xdata frac_e, xdata frac_ec;
    static uint8 xdata e_idx, xdata ec_idx, xdata e_next, xdata ec_next;
    static float xdata v00, xdata v01, xdata v10, xdata v11, xdata v0, xdata v1;
    float result;
	float xdata FUZZY_KP_SCALE = 0.2;
	float xdata FUZZY_KD_SCALE = 2.33;
	
	FUZZY_KP_SCALE = seekfree_assistant_parameter[0];
	FUZZY_KD_SCALE = seekfree_assistant_parameter[1];

    e_idx  = quantize_frac(error,        FUZZY_E_MAX, &frac_e);
    ec_idx = quantize_frac(error_change, FUZZY_EC_MAX, &frac_ec);

    e_next = (e_idx < 6) ? (uint8)(e_idx + 1) : e_idx;
    ec_next = (ec_idx < 6) ? (uint8)(ec_idx + 1) : ec_idx;

    // Kp：双线性插值语言值 → 乘比例因子
    v00 = (float)rule_Kp[e_idx][ec_idx];
    v01 = (float)rule_Kp[e_idx][ec_next];
    v10 = (float)rule_Kp[e_next][ec_idx];
    v11 = (float)rule_Kp[e_next][ec_next];
    
	v0 = v00 + (v01 - v00) * frac_ec;//误差
    v1 = v10 + (v11 - v10) * frac_ec;//误差变化率
    result = v0 + (v1 - v0) * frac_e;
    *delta_Kp = result * FUZZY_KP_SCALE;

    // Kd：双线性插值语言值 → 乘比例因子
    v00 = (float)rule_Kd[e_idx][ec_idx];
    v01 = (float)rule_Kd[e_idx][ec_next];
    v10 = (float)rule_Kd[e_next][ec_idx];
    v11 = (float)rule_Kd[e_next][ec_next];
    
	v0 = v00 + (v01 - v00) * frac_ec;
    v1 = v10 + (v11 - v10) * frac_ec;
    result = v0 + (v1 - v0) * frac_e;
    *delta_Kd = result * FUZZY_KD_SCALE;
}

// 将电磁误差转换成差速转向命令，输出 turn_cmd，增量式
float Turn_Control_PID(uint16 Result_L,uint16 Result_Middle_M_L,uint16 Result_Middle_M_R,uint16 Result_R,uint16 Result_Middle_M)
{
    float turn_cmd = 0;
    static float delta_Kp = 0, delta_Kd = 0;
    static float fuzzy_P1 = 0, fuzzy_D = 0;
    static float error_last = 0;

	float Servo_P1 = 1.3f;
    float Servo_D = 10.0f;
	
    float Result_Left = (float)Result_L;
    float Result_Right = (float)Result_R;
    float Result_Middle_M_Right = (float)Result_Middle_M_R;
    float Result_Middle_M_Left = (float)Result_Middle_M_L;
    float member1 = 0;
    float denominator1 = 0;
    float Vertical_Weight = 0;//垂直分量
    float Denominator_Weight = 0;

	 Servo_P1 = seekfree_assistant_parameter[2];
     Servo_D = seekfree_assistant_parameter[3];
	 
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
//    else if(Result_Middle_M_Left >= 40 && Result_Middle_M_Right >= 40 && Result_Left >= 10 && Result_Right >= 10 && Result_Middle_M <= 85)//十字
     if(Result_Middle_M_Left >= 40 && Result_Middle_M_Right >=40 && Result_Left >= 10 && Result_Right >= 10 && Result_Middle_M <= 85 && Round_State == ROUND_NONE)//十字
    {
        Vertical_Weight = 0.2f;
        Servo_D += 5;//加阻尼
        if(Servo_D >= 42.5f) Servo_D = 42.5f;
        Servo_P1 = 0.1f;
        Cross_Config = 1;//十字标志位，主要用来让它过了十字之后减速，因为需要拐一个直角弯，容易过冲
    }
	
//    else if(Round_Config1 == 1)//入环在环内走的参数
//    {
//        Vertical_Weight = 0.85f;
//        Denominator_Weight = Vertical_Weight + 0.01f;
//		circle_config = 2;
//    }
//    else if(Round_Config2 == 1)//出环参数
//    {
//        Vertical_Weight = 0.6f;
//        Denominator_Weight = Vertical_Weight + 0.01f;
//        Servo_P2 = 0;
//        Servo_P1 = 0.1f;
//        GKD = 0;
//		circle_config = 3;
//    }
	
	// ========== 环岛六阶段处理 ==========
	  else
	  {
		  // ① 预处理阶段检测：三路电感分别达到各自阈值
		  uint8 all_inductance_high = (Result_L >= Round_Params.pre_adc_thres_L)
									&& (Result_Middle_M >= Round_Params.pre_adc_thres_M)
									&& (Result_R >= Round_Params.pre_adc_thres_R);

		  // 正常循迹状态下检测到环岛特征
		  if(Round_State == ROUND_NONE && all_inductance_high)
		  {
			  Round_State = ROUND_PRE;
			  Round_Pre_Distance = 0;
			  Round_Config1 = 0;
		  }

		  // 预处理阶段：判断环岛方向
		  else if(Round_State == ROUND_PRE)
		  {
			  // 如果左侧电感更强，判断为左环；否则为右环
			  if(Result_L > Result_R)
			  {
				  Round_Direction = 0;  // 左环
			  }
			  else
			  {
				  Round_Direction = 1;  // 右环
			  }
			  Vertical_Weight = 0.75f;
			  Denominator_Weight = Vertical_Weight + 0.01f;
		  }
		  // ② 入环阶段 0°~60°：电感值放大约两倍
		  else if(Round_State == ROUND_ENTRY)
		  {
			  // 根据环岛方向放大对应侧电感
			  if(Round_Direction == 0)  // 左环
			  {
				  Result_L = (uint16)(Result_L * Round_Params.entry_amplify);
				  Result_Middle_M_L = (uint16)(Result_Middle_M_L * Round_Params.entry_amplify);
				  if(Result_L > 100) Result_L = 100;
				  if(Result_Middle_M_L > 100) Result_Middle_M_L = 100;
			  }
			  else  // 右环
			  {
				  Result_R = (uint16)(Result_R * Round_Params.entry_amplify);
				  Result_Middle_M_R = (uint16)(Result_Middle_M_R * Round_Params.entry_amplify);
				  if(Result_R > 100) Result_R = 100;
				  if(Result_Middle_M_R > 100) Result_Middle_M_R = 100;
			  }
			  Vertical_Weight = 0.83f;
			  Denominator_Weight = Vertical_Weight + 0.01f;
			  circle_config = 2;
		  }
		  // ③ 环内阶段 60°~270°：常规循迹
		  else if(Round_State == ROUND_INSIDE)
		  {
			  Vertical_Weight = 0.83f;
			  Denominator_Weight = Vertical_Weight + 0.01f;
			  circle_config = 2;
		  }
		  // ④ 出环阶段 270°~330°：特殊处理，保持直线出环
		  else if(Round_State == ROUND_EXIT)
		  {
			  Vertical_Weight = 0.6f;
			  Denominator_Weight = Vertical_Weight + 0.01f;
			  Servo_P1 = 0.1f;
			  circle_config = 3;
		  }
		  // ⑤ 出环后阶段 330°后：恢复正常循迹，避免再次进环
		  else if(Round_State == ROUND_EXIT_AFTER)
		  {
			  Vertical_Weight = 0.75f;
			  Denominator_Weight = Vertical_Weight + 0.01f;
			  circle_config = 4;
		  }
	  }

    // member1 表示方向偏差，denominator1 用于归一化，避免不同强度下误差量级漂移
    member1 = ((1 - Vertical_Weight) * (Result_Left - Result_Right) + Vertical_Weight * (Result_Middle_M_Left - Result_Middle_M_Right));
    denominator1 = ((1 - Vertical_Weight) * (Result_Left + Result_Right) + Denominator_Weight * fabs(Result_Middle_M_Left - Result_Middle_M_Right));


    if(Servo_D <= 15) Servo_D = 15;
    else if(Servo_D >= 50) Servo_D = 50;


    // 最终转向命令由比例、非线性比例、微分和陀螺仪阻尼共同组成
    // 模糊 PID：根据 error 和 error_change 动态微调 P1 和 D
    error = (member1 / (denominator1 + 0.00001f)) * 90;

    error_change = error - error_last;

    // 特殊元素（十字、环岛）使用硬编码参数，跳过模糊调整
    if(Cross_Config == 0 && Round_State == ROUND_NONE)
    {
        delta_Kp = 0;
        delta_Kd = 0;
        Fuzzy_PID_Adjust(error, error_change, &delta_Kp, &delta_Kd);

        fuzzy_P1 = Servo_P1 + delta_Kp;
        fuzzy_D  = Servo_D + delta_Kd;
    }
    else
    {
        fuzzy_P1 = Servo_P1;
        fuzzy_D  = Servo_D;
    }

    if(fuzzy_P1 < 0.8f) fuzzy_P1 = 0.8f;
    if(fuzzy_P1 > 2.0f) fuzzy_P1 = 2.0f;
    if(fuzzy_D  < 5.0f) fuzzy_D =  5.0f;
    if(fuzzy_D  > 30.0f) fuzzy_D = 30.0f;

    turn_cmd = error * fuzzy_P1 + error_change * fuzzy_D;
    error_last = error;

    if(turn_cmd >= TURN_CMD_MAX) turn_cmd = TURN_CMD_MAX;
    else if(turn_cmd <= TURN_CMD_MIN) turn_cmd = TURN_CMD_MIN;

    error1 = error;
    Turn_Cmd1 = turn_cmd;
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