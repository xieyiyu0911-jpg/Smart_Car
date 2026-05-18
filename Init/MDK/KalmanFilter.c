//#include "headfile.h"
//#include "KalmanFilter.h"
//#include "math.h"
//#include "SEEKFREE_IMU660RA.h"

//KalmanFilter yaw_filter;

//// 初始化卡尔曼滤波器
//void KalmanFilter_Init(KalmanFilter *kf) {
//    // 设置噪声协方差
//    kf->q_angle = 0.001f;
//    kf->q_bias = 0.003f;
//    kf->r_measure = 0.03f;

//    // 初始化状态
//    kf->angle = 0.0f;
//    kf->bias = 0.0f;
//    kf->rate = 0.0f;

//    // 初始化协方差矩阵
//    kf->P[0][0] = 0.0f;
//    kf->P[0][1] = 0.0f;
//    kf->P[1][0] = 0.0f;
//    kf->P[1][1] = 0.0f;
//}

//// 卡尔曼滤波更新函数
//float KalmanFilter_Update(KalmanFilter *kf, float new_angle, float new_rate, float dt) {
//    // 预测步骤
//	  float S = 0; float y = 0 ;float P00_temp = 0;float P01_temp = 0;
// 	  float K[2];
//	
//    kf->rate = new_rate - kf->bias;
//    kf->angle += dt * kf->rate;

//    // 更新协方差矩阵
//    kf->P[0][0] += dt * (dt*kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->q_angle);
//    kf->P[0][1] -= dt * kf->P[1][1];
//    kf->P[1][0] -= dt * kf->P[1][1];
//    kf->P[1][1] += kf->q_bias * dt;

//    // 计算卡尔曼增益
//    S = kf->P[0][0] + kf->r_measure;
//    
//    K[0] = kf->P[0][0] / S;
//    K[1] = kf->P[1][0] / S;

//    // 更新估计值
//    y = new_angle - kf->angle;
//    kf->angle += K[0] * y;
//    kf->bias += K[1] * y;

//    // 更新协方差矩阵
//     P00_temp = kf->P[0][0];
//     P01_temp = kf->P[0][1];

//    kf->P[0][0] -= K[0] * P00_temp;
//    kf->P[0][1] -= K[0] * P01_temp;
//    kf->P[1][0] -= K[1] * P00_temp;
//    kf->P[1][1] -= K[1] * P01_temp;

//    return kf->angle;
//}

//// 修改后的获取偏航角函数（带卡尔曼滤波）
//float Get_Filtered_Yaw(float dt) {
//		static float prev_angle = 0.0f;float raw_rate = 0.0f;float current_angle = 0.0f;float filtered_angle  = 0.0f;
//	  
//		imu660ra_get_gyro();
//    raw_rate = imu660ra_gyro_transition(imu660ra_gyro_z);  // 原始获取函数
//    
//    // 使用卡尔曼滤波
//    current_angle = prev_angle + raw_rate * dt;
//    filtered_angle = KalmanFilter_Update(&yaw_filter, current_angle, raw_rate, dt);
//    prev_angle = filtered_angle;
//    
//    return filtered_angle;
//}