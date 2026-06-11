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

// ģ�� PID ������error �� error_change �����Χ
// error_change ��λ ��/tick(10ms), 800��/s ʵ�ʽ��ٶ� �� 8��/tick
#define FUZZY_E_MAX   90.0f
#define FUZZY_EC_MAX  8.0f

// ģ���������ʹ������ֵ (-3=NB, -2=NM, -1=NS, 0=ZO, 1=PS, 2=PM, 3=PB)
// �� = E(NB��PB)���� = EC(NB��PB)���������ӽ�����ֵӳ��Ϊʵ�ʵ�����
//#define FUZZY_KP_SCALE 0.2f    // ����ֵ ��0.2 = ʵ�� ��Kp,  ��Χ [-0.6, +0.6]
//#define FUZZY_KD_SCALE 2.333f  // ����ֵ ��2.333 = ʵ�� ��Kd, ��Χ [-7, +7]

// Kp ����������Ͻ�(��ƫ��+��)�������������½�(��ƫ��+����)����������
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

// Kp2 ���������������Ҫ�� |E| ��ֵ������EC ������ Kp ���������� P1 ����ǿָ����
// �����ʱ�ṩ������ boost Ӧ�Լ��䣬С���ʱ���㣨�������������������С��˥������
int8 code rule_Kp2[7][7] = {
    //  EC: NB  NM  NS  ZO  PS  PM  PB
    {  2,  2,  1,  1,  0,  0,  0}, // E: NB
    {  1,  1,  1,  0,  0,  0,  0}, // E: NM
    {  0,  0,  0,  0,  0,  0,  0}, // E: NS
    {  0,  0,  0,  0,  0,  0,  0}, // E: ZO
    {  0,  0,  0,  0,  0,  0,  0}, // E: PS
    {  0,  0,  0,  0,  1,  1,  1}, // E: PM
    {  0,  0,  0,  1,  1,  2,  2}, // E: PB
};

// Kd �������ͬ��(��)����ֵ������, ���(����)����ֵ�����ᡣ���Գ�
int8 code rule_Kd[7][7] = {
    //  EC: NB   NM   NS   ZO   PS   PM   PB
    {-3, -1,  0,  0,  1,  2,  3}, // E: NB
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

uint8 xdata Fuzzy_Config = 0;
extern uint8 xdata Tiaocan_Config;


extern volatile int16 motor_speed_L;
extern volatile int16 motor_speed_R;
extern uint8 xdata read_buff1[8];
extern uint8 xdata read_buff2[4];
extern float xdata Angle_Speed_Output1;
extern float xdata Yaw_Angular_Speed;
extern float xdata Source;
extern float xdata Second_distance;
extern float xdata Second_encoder_ave;
extern float xdata SpeedTarget_L, xdata Target_Right1;
extern float xdata SpeedMeasure_L, xdata SpeedMeasure_R;
extern float xdata Turn_Output;
 
float xdata error_change = 0;
float xdata error = 0;

float xdata FUZZY_KP_SCALE = 0.2f;
float xdata FUZZY_KD_SCALE = 2.33f;
float xdata FUZZY_KP2_SCALE = 0.001f;
float xdata Servo_P1 = 0.9f;
float xdata Servo_D = 17.0f;
float xdata Servo_P2 = 0.009f;

// ========== ������������ ==========
  Round_State_TypeDef Round_State = ROUND_NONE;  // ����״̬��
  uint8 Round_Direction = 0;                      // ��������0=�󻷣�1=�һ�

  // ��̼���
  float xdata Round_Pre_Distance = 0;                  // Ԥ�����׶����
  float xdata Round_Exit_Distance = 0;                 // ���������

  // ������������
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

// ���ȫ���߻���λ����ͣʱ��ֱ������Ŀ���ٶȲ��ر� PWM
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

// ��λֵƽ���˲�
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

// ������ֵ�����������������������ڲ�ֵ��С������
static uint8 quantize_frac(float val, float vmax, float *frac)
{
    static float xdata ratio, xdata idx_f;// ratio=��һ��ֵ, idx_f=��������
    static uint8 xdata idx;// idx=��������(0-6)

    ratio = val / vmax;//��һ����ӳ�䵽[-1,1]
	
	//�޷�
    if(ratio >  1.0f) ratio =  1.0f;
    if(ratio < -1.0f) ratio = -1.0f;
	
	//����һ��ֵӳ�䵽��������
    idx_f = ratio * 3.0f + 3.0f;
    idx = (uint8)idx_f;
    if(idx > 5) idx = 5;
    *frac = idx_f - (float)idx;//С�����ּ�������
    return idx;
}


//  fuzzy_P1 = Servo_P1 + delta_Kp
//           = [2] + result_Kp �� [0]
//  - result_Kp��˫���Բ�ֵ�������[-3, 3]
//  - FUZZY_KP_SCALE = [0]��Ĭ�� 0.2 �� delta_Kp �� [-0.6, +0.6]
//  - ���� Servo_P1 = 1.3���� fuzzy ���ں�[0.7, 1.9]
//  - ����Ӳ�޷���[0.8, 4.0]

//  fuzzy_D

//  fuzzy_D = Servo_D + delta_Kd
//          = [3] + result_Kd �� [1]
//  - result_Kd��˫���Բ�ֵ�������[-3, 3]
//  - FUZZY_KD_SCALE = [1]��Ĭ�� 2.33 �� delta_Kd �� [-6.99, +6.99]
//  - ���� Servo_D = 10.0���� fuzzy ���ں�[3.0, 17.0]
//  - ����Ӳ�޷���[5.0, 25.0]

//  fuzzy_P2

//  fuzzy_P2 = Servo_P2 + delta_Kp2
//           = [4] + result_Kp2 �� [6]
//  - result_Kp2��˫���Բ�ֵ�������[0, 2]���������û�и�ֵ��
//  - FUZZY_KP2_SCALE = [6]��Ĭ�� 0.001 �� delta_Kp2 �� [0, +0.002]
//  - ���� Servo_P2 = 0.004���� fuzzy ���ں�[0.004, 0.006]
//  - ����Ӳ�޷���[0.0, 0.012]

// ģ�� PID ������˫���Բ�ֵ + ��������
void Fuzzy_PID_Adjust(float error, float error_change, float *delta_Kp, float *delta_Kd, float *delta_Kp2)
{
    static float xdata frac_e, xdata frac_ec;
    static uint8 xdata e_idx, xdata ec_idx, xdata e_next, xdata ec_next;
    static float xdata v00, xdata v01, xdata v10, xdata v11, xdata v0, xdata v1;
    float result;
	


    e_idx  = quantize_frac(error,        FUZZY_E_MAX, &frac_e);
    ec_idx = quantize_frac(error_change, FUZZY_EC_MAX, &frac_ec);

    e_next = (e_idx < 6) ? (uint8)(e_idx + 1) : e_idx;
    ec_next = (ec_idx < 6) ? (uint8)(ec_idx + 1) : ec_idx;

    // Kp��˫���Բ�ֵ����ֵ �� �˱�������
    v00 = (float)rule_Kp[e_idx][ec_idx];
    v01 = (float)rule_Kp[e_idx][ec_next];
    v10 = (float)rule_Kp[e_next][ec_idx];
    v11 = (float)rule_Kp[e_next][ec_next];
    
	v0 = v00 + (v01 - v00) * frac_ec;//���
    v1 = v10 + (v11 - v10) * frac_ec;//���仯��
    result = v0 + (v1 - v0) * frac_e;
    *delta_Kp = result * FUZZY_KP_SCALE;

    // Kd��˫���Բ�ֵ����ֵ �� �˱�������
    v00 = (float)rule_Kd[e_idx][ec_idx];
    v01 = (float)rule_Kd[e_idx][ec_next];
    v10 = (float)rule_Kd[e_next][ec_idx];
    v11 = (float)rule_Kd[e_next][ec_next];
    
	v0 = v00 + (v01 - v00) * frac_ec;
    v1 = v10 + (v11 - v10) * frac_ec;
    result = v0 + (v1 - v0) * frac_e;
    *delta_Kd = result * FUZZY_KD_SCALE;

    // Kp2 ˫���Բ�ֵ + ��������
    v00 = (float)rule_Kp2[e_idx][ec_idx];
    v01 = (float)rule_Kp2[e_idx][ec_next];
    v10 = (float)rule_Kp2[e_next][ec_idx];
    v11 = (float)rule_Kp2[e_next][ec_next];

    v0 = v00 + (v01 - v00) * frac_ec;
    v1 = v10 + (v11 - v10) * frac_ec;
    result = v0 + (v1 - v0) * frac_e;
    *delta_Kp2 = result * FUZZY_KP2_SCALE;
}

// ��������ת���ɲ���ת�������� turn_cmd������ʽ
float Turn_Control_PID(uint16 Result_L,uint16 Result_Middle_M_L,uint16 Result_Middle_M_R,uint16 Result_R,uint16 Result_Middle_M)
{
    float turn_cmd = 0;
    static float xdata delta_Kp = 0, xdata delta_Kd = 0, xdata delta_Kp2 = 0;
    static float xdata fuzzy_P1 = 0, xdata fuzzy_D = 0, xdata fuzzy_P2 = 0;
    static float xdata error_last = 0;

	
    float Result_Left = (float)Result_L;
    float Result_Right = (float)Result_R;
    float Result_Middle_M_Right = (float)Result_Middle_M_R;
    float Result_Middle_M_Left = (float)Result_Middle_M_L;
    float member1 = 0;
    float denominator1 = 0;
    float Vertical_Weight = 0;//��ֱ����
    float Denominator_Weight = 0;
	


    Cross_Config = 0;//��ʮ�ֱ�־λ
	
	circle_config = 0;
	
    // ֱ��������Ԫ�ض��м��е�������ͬ�����ﶯ̬��������/�м�Ȩ��
    if(Result_Middle_M <= 75)
    {
        Vertical_Weight = 0.75;//0.75
        Denominator_Weight = Vertical_Weight + 0.01f;
    }
    else//���������Ĳ����������м�����������
    {
        //Vertical_Weight = ((0.85f - 0.66f) / 25.0f) * Result_Middle_M + 4 * 0.66f - 2.55f;
		Vertical_Weight = 0.75;//0.75
        Denominator_Weight = Vertical_Weight + 0.01f;
    }


//	if(Result_Middle_M >= 99 && Round_Config2 == 0)//����
//    {
//        Round_Config1 = 1;//������־λ
//        circle_config = 1;    
//		Second_distance = 0;
//		Buzzer_On();
//    }
	

     if(Result_Middle_M_Left >= 40 && Result_Middle_M_Right >=40 && Result_Left >= 10 && Result_Right >= 10 && Result_Middle_M <= 85 && Round_State == ROUND_NONE)//ʮ��
    {
        Vertical_Weight = 0.2f;
        Servo_D += 5;//������
        if(Servo_D >= 42.5f) Servo_D = 42.5f;
        Servo_P1 = 0.1f;
        Cross_Config = 1;//ʮ�ֱ�־λ����Ҫ������������ʮ��֮����٣���Ϊ��Ҫ��һ��ֱ���䣬���׹���
    }
	
//    else if(Round_Config1 == 1)//�뻷�ڻ����ߵĲ���
//    {
//        Vertical_Weight = 0.85f;
//        Denominator_Weight = Vertical_Weight + 0.01f;
//		circle_config = 2;
//    }
//    else if(Round_Config2 == 1)//��������
//    {
//        Vertical_Weight = 0.6f;
//        Denominator_Weight = Vertical_Weight + 0.01f;
//        Servo_P2 = 0;
//        Servo_P1 = 0.1f;
//        GKD = 0;
//		circle_config = 3;
//    }
	
//	// ========== �������׶δ��� ==========
//	  else
//	  {
//		  // �� Ԥ�����׶μ�⣺��·��зֱ�ﵽ������ֵ
//		  uint8 all_inductance_high = (Result_L >= Round_Params.pre_adc_thres_L)
//									&& (Result_Middle_M >= Round_Params.pre_adc_thres_M)
//									&& (Result_R >= Round_Params.pre_adc_thres_R);

//		  // ����ѭ��״̬�¼�⵽��������
//		  if(Round_State == ROUND_NONE && all_inductance_high)
//		  {
//			  Round_State = ROUND_PRE;
//			  Round_Pre_Distance = 0;
//			  Round_Config1 = 0;
//		  }

//		  // Ԥ�����׶Σ��жϻ�������
//		  else if(Round_State == ROUND_PRE)
//		  {
//			  // �������и�ǿ���ж�Ϊ�󻷣�����Ϊ�һ�
//			  if(Result_L > Result_R)
//			  {
//				  Round_Direction = 0;  // ��
//			  }
//			  else
//			  {
//				  Round_Direction = 1;  // �һ�
//			  }
//			  Vertical_Weight = 0.75f;
//			  Denominator_Weight = Vertical_Weight + 0.01f;
//		  }
//		  // �� �뻷�׶� 0��~60�㣺���ֵ�Ŵ�Լ����
//		  else if(Round_State == ROUND_ENTRY)
//		  {
//			  // ���ݻ�������Ŵ��Ӧ����
//			  if(Round_Direction == 0)  // ��
//			  {
//				  Result_L = (uint16)(Result_L * Round_Params.entry_amplify);
//				  Result_Middle_M_L = (uint16)(Result_Middle_M_L * Round_Params.entry_amplify);
//				  if(Result_L > 100) Result_L = 100;
//				  if(Result_Middle_M_L > 100) Result_Middle_M_L = 100;
//			  }
//			  else  // �һ�
//			  {
//				  Result_R = (uint16)(Result_R * Round_Params.entry_amplify);
//				  Result_Middle_M_R = (uint16)(Result_Middle_M_R * Round_Params.entry_amplify);
//				  if(Result_R > 100) Result_R = 100;
//				  if(Result_Middle_M_R > 100) Result_Middle_M_R = 100;
//			  }
//			  Vertical_Weight = 0.83f;
//			  Denominator_Weight = Vertical_Weight + 0.01f;
//			  circle_config = 2;
//		  }
//		  // �� ���ڽ׶� 60��~270�㣺����ѭ��
//		  else if(Round_State == ROUND_INSIDE)
//		  {
//			  Vertical_Weight = 0.83f;
//			  Denominator_Weight = Vertical_Weight + 0.01f;
//			  circle_config = 2;
//		  }
//		  // �� �����׶� 270��~330�㣺���⴦��������ֱ�߳���
//		  else if(Round_State == ROUND_EXIT)
//		  {
//			  Vertical_Weight = 0.6f;
//			  Denominator_Weight = Vertical_Weight + 0.01f;
//			  Servo_P1 = 0.1f;
//			  circle_config = 3;
//		  }
//		  // �� ������׶� 330��󣺻ָ�����ѭ���������ٴν���
//		  else if(Round_State == ROUND_EXIT_AFTER)
//		  {
//			  Vertical_Weight = 0.75f;
//			  Denominator_Weight = Vertical_Weight + 0.01f;
//			  circle_config = 4;
//		  }
//	  }

    // member1 ��ʾ����ƫ�denominator1 ���ڹ�һ�������ⲻͬǿ�����������Ư��
    member1 = ((1 - Vertical_Weight) * (Result_Left - Result_Right) + Vertical_Weight * (Result_Middle_M_Left - Result_Middle_M_Right));
    denominator1 = ((1 - Vertical_Weight) * (Result_Left + Result_Right) + Denominator_Weight * fabs(Result_Middle_M_Left - Result_Middle_M_Right));


//    if(Servo_D <= 15) Servo_D = 15;
//    else if(Servo_D >= 50) Servo_D = 50;


    // ����ת�������ɱ����������Ա�����΢�ֺ����������Ṳͬ���
    // ģ�� PID������ error �� error_change ��̬΢�� P1 �� D
    error = (member1 / (denominator1 + 0.00001f)) * 90;

    error_change = error - error_last;

    // ����Ԫ�أ�ʮ�֡�������ʹ��Ӳ�������������ģ������
    if(Cross_Config == 0 && Round_State == ROUND_NONE)
    {
        Fuzzy_PID_Adjust(error, error_change, &delta_Kp, &delta_Kd, &delta_Kp2);

        fuzzy_P1 = Servo_P1 + delta_Kp;
        fuzzy_D  = Servo_D + delta_Kd;
        fuzzy_P2 = Servo_P2 + delta_Kp2;
    }
    else
    {
        fuzzy_P1 = Servo_P1;
        fuzzy_D  = Servo_D;
        fuzzy_P2 = Servo_P2;
    }
		
		if(seekfree_assistant_parameter[0] == 0 && seekfree_assistant_parameter[1] == 0 && seekfree_assistant_parameter[2] == 0) Fuzzy_Config = 1;
			
		if(Fuzzy_Config == 0)
		{
				if(fuzzy_P1 < 0.6f) fuzzy_P1 = 0.6f;
				if(fuzzy_P1 > 2.5f) fuzzy_P1 = 2.5f;
				if(fuzzy_D  < 5.0f) fuzzy_D =  5.0f;
				if(fuzzy_D  > 28.0f) fuzzy_D = 28.0f;
				if(fuzzy_P2 < 0.0f) fuzzy_P2 = 0.0f;
				if(fuzzy_P2 > 0.01f) fuzzy_P2 = 0.01f;
		}
    // 模糊PID 计算转向指令
    turn_cmd = error * fuzzy_P1 + fuzzy_P2 * fabs(error) * error + error_change * fuzzy_D;
    error_last = error;


    if(turn_cmd >= TURN_CMD_MAX) turn_cmd = TURN_CMD_MAX;
    else if(turn_cmd <= TURN_CMD_MIN) turn_cmd = TURN_CMD_MIN;

//    error1 = error;
//    Turn_Cmd1 = turn_cmd;
//    Servo_PID_D = Servo_D;
//    Servo_PID_P2 = Servo_P2;

    return turn_cmd;
}

float constrain_float(float amt, float low, float high)
{
    return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

void Angle_PID_Control(float Angle_error,float Angle_P,float Angle_I,float Angle_D,int16 Config)
{
    float P_data = 0, I_data = 0, D_data = 0;
    static float xdata Angle_Speed_error_last = 0;
    static float xdata Angle_I_count = 0;

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

// ��ת���������Ϊ������Ŀ���ٶȲʵ�ֲ���ת��
void Differential_Speed_Control(float turn_cmd)
{
    float base_speed = 0;
    float diff_ratio = 0;
    float diff_output = 0;
	float q = 0.4;


    diff_ratio = turn_cmd / ANGLE_SPEED_OUTPUT_MAX;


	if(diff_ratio >= 0)
	{
		SpeedTarget_L = (1-diff_ratio) * SpeedTarget_L;
		Target_Right1 = (diff_ratio * q + 1) * Target_Right1;
	}
	if(diff_ratio < 0)
	{
		SpeedTarget_L = ( - diff_ratio * q + 1) * SpeedTarget_L;
		Target_Right1 = (1 + diff_ratio) * Target_Right1;
	}
	
    if(SpeedTarget_L < 0) SpeedTarget_L = 0;
    if(Target_Right1 < 0) Target_Right1 = 0;
	
    if(SpeedTarget_L > SPEED_TARGET_MAX) SpeedTarget_L = SPEED_TARGET_MAX;
    if(Target_Right1 > SPEED_TARGET_MAX) Target_Right1 = SPEED_TARGET_MAX;
}

// �����ָ������ٶȱջ���Ŀ���ٶ����Բ��ٷ�����
void Motor_PID(float SpeedTarget,int16 motor_speed,float Motor_P,float Motor_I,float Motor_D,int16 Config,float turn_cmd)
{
    static float xdata I_count_L = 0, xdata I_count_R = 0;
    static float xdata error_last_L = 0, xdata error_last_R = 0;
    float speed_error = 0;
    float P_data = 0, I_data = 0, D_data = 0;
    float I_Count_Max = 500;

//    // ת��Խ���ң���������ԽС����������л��ֶѻ����³������
//    I_Count_Max = -fabs(turn_cmd) * (10.0f / 3.0f) + 100 + 2400;

//    if(Cross_Config == 1) I_Count_Max = 150;
	
//    if(I_Count_Max >= 2500) I_Count_Max = 2500;
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
        MOTOR_L_DIR_PIN = 1;//��ת
	    pwm_duty(MOTOR_L_PWM_PIN,(uint32)Motor_L_output);
    }
	
    else if(Motor_L_output < 0)
    {
        Motor_L_output = -Motor_L_output;
        MOTOR_L_DIR_PIN = 0;//��ת
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
				MOTOR_R_DIR_PIN = 1;//��ת
				pwm_duty(MOTOR_R_PWM_PIN,(uint32)Motor_output_R);
       }
       else if(Motor_output_R < 0)
       {
          Motor_output_R =  -Motor_output_R;
					MOTOR_R_DIR_PIN = 0;//��ת
          pwm_duty(MOTOR_R_PWM_PIN,(uint32)Motor_output_R);
       }
}