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


void Fuzzy_PID_Adjust(float error, float error_change, float *delta_Kp, float *delta_Kd);
static uint8 quantize_frac(float val, float vmax, float *frac);



//=========================================================================================================
//  环岛处理相关定义 - 用于六阶段环岛状态机
//=========================================================================================================

// 环岛状态枚举 - 定义环岛处理的六个阶段
typedef enum {
    ROUND_NONE = 0,      // 正常循迹状态（未检测到环岛）
    ROUND_PRE,           // 预处理阶段：三路电感同时达到峰值，临近环岛入口
    ROUND_ENTRY,         // 入环阶段：角度积分 0°~60°，电感值放大引导入环
    ROUND_INSIDE,        // 环内阶段：角度积分 60°~270°，车在环内正常循迹
    ROUND_EXIT,          // 出环阶段：角度积分 270°~330°，特殊处理保持直线出环
    ROUND_EXIT_AFTER     // 出环后阶段：角度超过330°，此时电感值仍偏高，开启里程计数规避误判
} Round_State_TypeDef;

// 环岛配置参数结构体 - 存储环岛检测和处理的各项阈值
typedef struct {
    // 预处理阶段各电感阈值（三路电感各自独立阈值，因为峰值电压可能不同）
    uint16 pre_adc_thres_L;     // 左横电感预处理阈值（如 65）
    uint16 pre_adc_thres_M;     // 中间电感预处理阈值（如 80）
    uint16 pre_adc_thres_R;     // 右横电感预处理阈值（如 65）
    uint16 entry_adc_thres;     // 入环电感阈值（如 95）

    // 角度阈值（单位：度）- 用于偏航角度积分控制
    float entry_angle_end;       // 入环结束角度（如 60°），超过此角度进入环内阶段
    float inside_angle_end;     // 环内结束角度（如 270°），超过此角度进入出环阶段
    float exit_angle_end;        // 出环结束角度（如 330°），超过此角度进入出环后阶段

    // 里程阈值 - 用于预处理和出环后阶段的距离判断
    float pre_distance_thres;    // 预处理阶段里程阈值（如 500），达到后开启角度积分入环
    float exit_distance_thres;   // 出环后里程阈值（如 1000，即1米），超过后彻底退出环岛状态

    // 电磁放大倍数
    float entry_amplify;         // 入环阶段电感放大倍数（如 2.0），使入环侧电感值增大引导小车
} Round_Config_TypeDef;


//=========================================================================================================

#endif