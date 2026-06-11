#ifndef __KALMANFILTER_H_
#define __KALMANFILTER_H_

// 卡尔曼滤波器结构体定义
typedef struct {
    float q_angle;   // 过程噪声协方差（角度）
    float q_bias;    // 过程噪声协方差（偏差）
    float r_measure; // 测量噪声协方差
    
    float angle;     // 最优估计角度值
    float bias;      // 最优估计偏差值
    float rate;      // 未经滤波的角速度
    
    float P[2][2];   // 误差协方差矩阵
} KalmanFilter;


void KalmanFilter_Init(KalmanFilter *kf);
float KalmanFilter_Update(KalmanFilter *kf, float new_angle, float new_rate, float dt);
float Get_Filtered_Yaw(float dt);

#endif