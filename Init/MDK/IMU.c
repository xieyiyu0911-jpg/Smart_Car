#include "headfile.h"
#include <STC32G.H>

/* 全局变量定义 */
imu_data_t imu_data;                           /* IMU数据 */
Quaternion q = {1.0f, 0.0f, 0.0f, 0.0f};       /* 四元数，初始化为单位四元数 */
EulerAngle euler = {0.0f, 0.0f, 0.0f};         /* 欧拉角 */
float Yaw_Angular_Speed = 0;

float custom_atan2(float y, float x)
{
	if(x>0)
		return atan(y/x);
	else if(x<0 && y>=0)
		return atan(y/x) + PI;
	else if(x<0 && y<0)
		return atan(y/x) - PI;
	else if(x==0 && y>0)
		return PI/2;
	else if(x==0 && y<0)
		return -PI/2;
	return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     四元数初始化函数，用于初始化四元数和陀螺仪偏置
// 参数说明     void
// 返回参数     void
// 使用示例     quaternion_init();
//-------------------------------------------------------------------------------------------------------------------
void quaternion_init(void) 
{
    /* 初始化为单位四元数 */
    q.w = 1.0f;
    q.x = 0.0f;
    q.y = 0.0f;
    q.z = 0.0f;
    
    /* 初始化陀螺仪偏置 */
    imu_data.gyro_x = 0.0f;
    imu_data.gyro_y = 0.0f;
    imu_data.gyro_z = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     角速度数据低通滤波函数，用于滤除小幅度噪声
// 参数说明     cy: 输入的角速度值
// 返回参数     float: 滤波后的角速度值
// 使用示例     filtered_gyro = IMU_lvbo(raw_gyro);
//-------------------------------------------------------------------------------------------------------------------
float IMU_lvbo(float cy)
{
    if (cy < 0.15f && cy > -0.15f)
    {
        cy = 0.0f;
    }
    return cy;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     陀螺仪校准函数，用于计算陀螺仪的零偏值
// 参数说明     samples: 采样点数
// 返回参数     void
// 使用示例     gyro_calibrate(200);
//-------------------------------------------------------------------------------------------------------------------
void gyro_calibrate(uint16 samples) 
{
    uint16 i;
    
    /* 清除校准前的数据 */
    imu_data.gyro_x = 0.0f;
    imu_data.gyro_y = 0.0f;
    imu_data.gyro_z = 0.0f;
    
    /* 收集多个样本求平均值 */
    for(i = 0; i < samples; i++) 
    {
        imu660ra_get_gyro();
       
        imu_data.gyro_x += (float)imu660ra_gyro_x;
        imu_data.gyro_y += (float)imu660ra_gyro_y;
        imu_data.gyro_z += (float)imu660ra_gyro_z;
			  
				delay_ms(5);
    }
    
    /* 计算偏置 */
    imu_data.gyro_x = imu_data.gyro_x / (float)samples;
    imu_data.gyro_y = imu_data.gyro_y / (float)samples;
    imu_data.gyro_z = imu_data.gyro_z / (float)samples;

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     四元数归一化函数，用于保持四元数的单位长度
// 参数说明     q: 指向待归一化的四元数的指针
// 返回参数     void
// 使用示例     quat_normalize(&q);
//-------------------------------------------------------------------------------------------------------------------
void quat_normalize(Quaternion* q)
{
    float norm = sqrt(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
    
    /* 防止除以零 */
    if(norm < 0.0001f)
    {
        q->w = 1.0f;
        q->x = 0.0f;
        q->y = 0.0f;
        q->z = 0.0f;
        return;
    }
    
    q->w = q->w / norm;
    q->x = q->x / norm;
    q->y = q->y / norm;
    q->z = q->z / norm;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     四元数更新函数，用于根据陀螺仪数据更新四元数
// 参数说明     void
// 返回参数     void
// 使用示例     quaternion_update();
//-------------------------------------------------------------------------------------------------------------------
void quaternion_update(void) 
{
    float gx, gy, gz;
    float qDot1, qDot2, qDot3, qDot4;
    
    /* 读取IMU传感器数据 */
    imu660ra_get_gyro();
	
    /* 处理陀螺仪数据，减去偏置并转换为弧度/秒 */
    gx = imu660ra_gyro_transition((float)imu660ra_gyro_x - imu_data.gyro_x);
    gy = imu660ra_gyro_transition((float)imu660ra_gyro_y - imu_data.gyro_y);
    gz = imu660ra_gyro_transition((float)imu660ra_gyro_z - imu_data.gyro_z);
	
		gx = gx * PI / 180.0f;
    gy = gy * PI / 180.0f;
    gz = gz * PI / 180.0f;
	
    /* 应用低通滤波 */
    gx = IMU_lvbo(gx);
    gy = IMU_lvbo(gy);
    gz = IMU_lvbo(gz);
    
    /* 基于四元数微分方程进行积分更新 */
    /* q_dot = 0.5 * q ? ω，其中ω为角速度四元数[0,gx,gy,gz] */
    qDot1 = 0.5f * (-q.x * gx - q.y * gy - q.z * gz);
    qDot2 = 0.5f * (q.w * gx + q.y * gz - q.z * gy);
    qDot3 = 0.5f * (q.w * gy - q.x * gz + q.z * gx);
    qDot4 = 0.5f * (q.w * gz + q.x * gy - q.y * gx);
    
    /* 使用欧拉积分法更新四元数 */
    q.w = q.w + qDot1 * SAMPLE_FREQ;
    q.x = q.x + qDot2 * SAMPLE_FREQ;
    q.y = q.y + qDot3 * SAMPLE_FREQ;
    q.z = q.z + qDot4 * SAMPLE_FREQ;
    
    /* 四元数归一化 */
    quat_normalize(&q);
    
    /* 计算欧拉角 */
    euler = quaternion_to_euler(q);

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取当前四元数函数
// 参数说明     void
// 返回参数     Quaternion: 当前的四元数值
// 使用示例     Quaternion current_q = get_quaternion();
//-------------------------------------------------------------------------------------------------------------------
Quaternion get_quaternion(void) 
{
    return q;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     四元数转欧拉角函数，仅计算偏航角
// 参数说明     q: 输入的四元数
// 返回参数     EulerAngle: 计算得到的欧拉角（仅偏航角有效）
// 使用示例     EulerAngle angles = quaternion_to_euler(current_q);
//-------------------------------------------------------------------------------------------------------------------
EulerAngle quaternion_to_euler(Quaternion q) 
{
    float siny_cosp, cosy_cosp;
    
    /* 初始化其他角度为0 */
    euler.roll = 0.0f;
    euler.pitch = 0.0f;
    
    /* 计算偏航角 (Yaw) */
    siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
	
    euler.yaw = custom_atan2(siny_cosp, cosy_cosp);
    
    /* 转换为角度 */
    euler.yaw = euler.yaw * 180.0f / PI;
    
    return euler;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取俯仰角
// 参数说明     ax：x方向上的加速度;  az：z方向上的加速度;
// 返回参数     void
// 使用示例     imu_get_pitch();
//-------------------------------------------------------------------------------------------------------------------
float imu_get_pitch(void)
{
    float ax, az;

    imu660ra_get_acc();//获取加速度

    ax = imu660ra_acc_transition(imu660ra_acc_x);
    az = imu660ra_acc_transition(imu660ra_acc_z);

    return atan2(ax, az) * 180.0f / PI;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取z方向加速度
// 参数说明     az：z方向上的加速度;
// 返回参数     void
// 使用示例     imu_get_az();
//-------------------------------------------------------------------------------------------------------------------
float imu_get_az(void)
{
    imu660ra_get_acc();

    return imu660ra_acc_transition(imu660ra_acc_z);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断是否在墙上
// 参数说明     pitch：俯仰角;  az：z方向上的加速度;
// 返回参数     void
// 使用示例     imu_is_on_wall();
//-------------------------------------------------------------------------------------------------------------------
uint8 imu_is_on_wall(void)
{
    float pitch, az;

    pitch = imu_get_pitch();
    az = imu_get_az();

    if (fabs(pitch) > 60.0f && fabs(az) < 0.5f)
        return 1;
    else
        return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     直接给风扇调用
// 参数说明     pitch：俯仰角;  az：z方向上的加速度; is_on_wall：是否在墙上;
// 返回参数     float *pitch, float *az, bool *is_on_wall
// 使用示例     ***
//-------------------------------------------------------------------------------------------------------------------
void imu_get_wall_state(float *pitch, float *az, uint8 *is_on_wall)
{
    float ax;

    imu660ra_get_acc();

    ax = imu660ra_acc_transition(imu660ra_acc_x);
    *az = imu660ra_acc_transition(imu660ra_acc_z);

    *pitch = atan2(ax, *az) * 180.0f / PI;

    if (fabs(*pitch) > 60.0f && fabs(*az) < 0.5f)
        *is_on_wall = 1;
    else
        *is_on_wall = 0;
}