#ifndef __MOTOR_PID_H_
#define __MOTOR_PID_H_

#define MOTOR_R_PWM_PIN  PWMA_CH4P_P26
#define MOTOR_L_PWM_PIN  PWMA_CH2N_P13

#define MOTOR_R_DIR_PIN P10
#define MOTOR_L_DIR_PIN P24


#define Max1 255
#define Max2 255
#define Max3 255
#define Max4 255
#define Max5 255
#define Times 7
#define Left 1
#define Right 0


extern volatile int xdata Motor_Speed_Left[7];
extern volatile int xdata Motor_Speed_Right[7];
extern volatile float xdata Yaw_Angle;
extern float xdata Second_distance;
extern float xdata Second_encoder_ave;

uint16 PID_Conservation(uint16 Result_L,uint16 Result_Middle_M_L,uint16 Result_Middle_M_R, uint16 Result_R);
uint16 Uart_Stop(void);

uint16 Servo_Measure(int* inductance_array,int times);

void First_distance_calculate(void);
void Second_distance_calculate(void);

float Turn_Control_PID(uint16 Result_L,uint16 Result_Middle_M_L,uint16 Result_Middle_M_R,uint16 Result_R,uint16 Result_Middle_M);
float constrain_float(float amt, float low, float high);
extern float xdata Gyro_Kp;    // 角速度内环 Kp
extern float xdata Gyro_Kd;    // 角速度内环 Kd

void Angle_PID_Control(float Angle_error,float Angle_P,float Angle_I,float Angle_D,int16 Config);

void Differential_Speed_Control(float turn_cmd);
void Motor_PID(float SpeedTarget,int16 motor_speed,float Motor_P,float Motor_I,float Motor_D,int16 Config,float turn_cmd);

void Motor_PWM_set_L(void);
void Motor_PWM_set_R(void);


void Fuzzy_PID_Adjust(float error, float error_change, float *delta_Kp, float *delta_Kd, float *delta_Kp2);
static uint8 quantize_frac(float val, float vmax, float *frac);



//=========================================================================================================
//  ����������ض��� - �������׶λ���״̬��
//=========================================================================================================

// ����״̬ö�� - ���廷�������������׶�
typedef enum {
    ROUND_NONE = 0,      // ����ѭ��״̬��δ��⵽������
    ROUND_PRE,           // Ԥ�����׶Σ���·���ͬʱ�ﵽ��ֵ���ٽ��������
    ROUND_ENTRY,         // �뻷�׶Σ��ǶȻ��� 0��~60�㣬���ֵ�Ŵ������뻷
    ROUND_INSIDE,        // ���ڽ׶Σ��ǶȻ��� 60��~270�㣬���ڻ�������ѭ��
    ROUND_EXIT,          // �����׶Σ��ǶȻ��� 270��~330�㣬���⴦������ֱ�߳���
    ROUND_EXIT_AFTER     // ������׶Σ��Ƕȳ���330�㣬��ʱ���ֵ��ƫ�ߣ�������̼����������
} Round_State_TypeDef;

// �������ò����ṹ�� - �洢�������ʹ����ĸ�����ֵ
typedef struct {
    // Ԥ�����׶θ������ֵ����·��и��Զ�����ֵ����Ϊ��ֵ��ѹ���ܲ�ͬ��
    uint16 pre_adc_thres_L;     // �����Ԥ������ֵ���� 65��
    uint16 pre_adc_thres_M;     // �м���Ԥ������ֵ���� 80��
    uint16 pre_adc_thres_R;     // �Һ���Ԥ������ֵ���� 65��
    uint16 entry_adc_thres;     // �뻷�����ֵ���� 95��

    // �Ƕ���ֵ����λ���ȣ�- ����ƫ���ǶȻ��ֿ���
    float entry_angle_end;       // �뻷�����Ƕȣ��� 60�㣩�������˽ǶȽ��뻷�ڽ׶�
    float inside_angle_end;     // ���ڽ����Ƕȣ��� 270�㣩�������˽ǶȽ�������׶�
    float exit_angle_end;        // ���������Ƕȣ��� 330�㣩�������˽ǶȽ��������׶�

    // �����ֵ - ����Ԥ�����ͳ�����׶εľ����ж�
    float pre_distance_thres;    // Ԥ�����׶������ֵ���� 500�����ﵽ�����ǶȻ����뻷
    float exit_distance_thres;   // �����������ֵ���� 1000����1�ף��������󳹵��˳�����״̬

    // ��ŷŴ���
    float entry_amplify;         // �뻷�׶ε�зŴ������� 2.0����ʹ�뻷����ֵ��������С��
} Round_Config_TypeDef;

// ========== ���������������� ==========
  extern Round_State_TypeDef Round_State;
  extern uint8 Round_Direction;
  extern float xdata Round_Pre_Distance;
  extern float xdata Round_Exit_Distance;
  extern const Round_Config_TypeDef Round_Params;
//=========================================================================================================

#endif