#include "headfile.h"
#include <STC32G.H>

/* ȫ�ֱ������� */
imu_data_t imu_data;                                    /* IMU���� */
Quaternion xdata q = {1.0f, 0.0f, 0.0f, 0.0f};        /* ��Ԫ������ʼ��Ϊ��λ��Ԫ�� */
EulerAngle xdata euler = {0.0f, 0.0f, 0.0f};          /* ŷ���� */
float xdata Yaw_Angular_Speed = 0;

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
// �������     ��Ԫ����ʼ�����������ڳ�ʼ����Ԫ����������ƫ��
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     quaternion_init();
//-------------------------------------------------------------------------------------------------------------------
void quaternion_init(void) 
{
    /* ��ʼ��Ϊ��λ��Ԫ�� */
    q.w = 1.0f;
    q.x = 0.0f;
    q.y = 0.0f;
    q.z = 0.0f;
    
    /* ��ʼ��������ƫ�� */
    imu_data.gyro_x = 0.0f;
    imu_data.gyro_y = 0.0f;
    imu_data.gyro_z = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ���ٶ����ݵ�ͨ�˲������������˳�С��������
// ����˵��     cy: ����Ľ��ٶ�ֵ
// ���ز���     float: �˲���Ľ��ٶ�ֵ
// ʹ��ʾ��     filtered_gyro = IMU_lvbo(raw_gyro);
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
// �������     ������У׼���������ڼ��������ǵ���ƫֵ
// ����˵��     samples: ��������
// ���ز���     void
// ʹ��ʾ��     gyro_calibrate(200);
//-------------------------------------------------------------------------------------------------------------------
void gyro_calibrate(uint16 samples) 
{
    uint16 i;
    
    /* ���У׼ǰ������ */
    imu_data.gyro_x = 0.0f;
    imu_data.gyro_y = 0.0f;
    imu_data.gyro_z = 0.0f;
    
    /* �ռ����������ƽ��ֵ */
    for(i = 0; i < samples; i++) 
    {
        imu660ra_get_gyro();
       
        imu_data.gyro_x += (float)imu660ra_gyro_x;
        imu_data.gyro_y += (float)imu660ra_gyro_y;
        imu_data.gyro_z += (float)imu660ra_gyro_z;
			  
				delay_ms(5);
    }
    
    /* ����ƫ�� */
    imu_data.gyro_x = imu_data.gyro_x / (float)samples;
    imu_data.gyro_y = imu_data.gyro_y / (float)samples;
    imu_data.gyro_z = imu_data.gyro_z / (float)samples;

}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��Ԫ����һ�����������ڱ�����Ԫ���ĵ�λ����
// ����˵��     q: ָ�����һ������Ԫ����ָ��
// ���ز���     void
// ʹ��ʾ��     quat_normalize(&q);
//-------------------------------------------------------------------------------------------------------------------
void quat_normalize(Quaternion* q)
{
    float norm = sqrt(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
    
    /* ��ֹ������ */
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
// �������     ��Ԫ�����º��������ڸ������������ݸ�����Ԫ��
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     quaternion_update();
//-------------------------------------------------------------------------------------------------------------------
void quaternion_update(void) 
{
    float gx, gy, gz;
    float qDot1, qDot2, qDot3, qDot4;
    
    /* ��ȡIMU���������� */
    imu660ra_get_gyro();
	
    /* �������������ݣ���ȥƫ�ò�ת��Ϊ����/�� */
    gx = imu660ra_gyro_transition((float)imu660ra_gyro_x - imu_data.gyro_x);
    gy = imu660ra_gyro_transition((float)imu660ra_gyro_y - imu_data.gyro_y);
    gz = imu660ra_gyro_transition((float)imu660ra_gyro_z - imu_data.gyro_z);
	
		gx = gx * PI / 180.0f;
    gy = gy * PI / 180.0f;
    gz = gz * PI / 180.0f;
	
    /* Ӧ�õ�ͨ�˲� */
    gx = IMU_lvbo(gx);
    gy = IMU_lvbo(gy);
    gz = IMU_lvbo(gz);
    
    /* ������Ԫ��΢�ַ��̽��л��ָ��� */
    /* q_dot = 0.5 * q ? �أ����Ц�Ϊ���ٶ���Ԫ��[0,gx,gy,gz] */
    qDot1 = 0.5f * (-q.x * gx - q.y * gy - q.z * gz);
    qDot2 = 0.5f * (q.w * gx + q.y * gz - q.z * gy);
    qDot3 = 0.5f * (q.w * gy - q.x * gz + q.z * gx);
    qDot4 = 0.5f * (q.w * gz + q.x * gy - q.y * gx);
    
    /* ʹ��ŷ�����ַ�������Ԫ�� */
    q.w = q.w + qDot1 * SAMPLE_FREQ;
    q.x = q.x + qDot2 * SAMPLE_FREQ;
    q.y = q.y + qDot3 * SAMPLE_FREQ;
    q.z = q.z + qDot4 * SAMPLE_FREQ;
    
    /* ��Ԫ����һ�� */
    quat_normalize(&q);
    
    /* ����ŷ���� */
    euler = quaternion_to_euler(q);

}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ��ǰ��Ԫ������
// ����˵��     void
// ���ز���     Quaternion: ��ǰ����Ԫ��ֵ
// ʹ��ʾ��     Quaternion current_q = get_quaternion();
//-------------------------------------------------------------------------------------------------------------------
Quaternion get_quaternion(void) 
{
    return q;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��Ԫ��תŷ���Ǻ���
// ����˵��     q: �������Ԫ��
// ���ز���     EulerAngle: ����õ���ŷ���ǣ���ƫ������Ч��
// ʹ��ʾ��     EulerAngle angles = quaternion_to_euler(current_q);
//-------------------------------------------------------------------------------------------------------------------
EulerAngle quaternion_to_euler(Quaternion q) 
{
    float sinr_cosp, cosr_cosp;
    float sinp;
    float siny_cosp, cosy_cosp;
    
    /* ����ƫ���� (Yaw) */
    siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
	
    euler.yaw = custom_atan2(siny_cosp, cosy_cosp);
	
	/* ���㸩���� (Pitch) */
	sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if(sinp > 1.0f) sinp = 1.0f;
    else if(sinp < -1.0f) sinp = -1.0f;
    
	euler.pitch = asin(sinp);
	
	/* �����ת�� (Roll) */
	sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    
	euler.roll = custom_atan2(sinr_cosp, cosr_cosp);
    
    /* ת��Ϊ�Ƕ� */
    euler.yaw = - euler.yaw * 180.0f / PI;
	euler.roll = euler.roll * 180.0f / PI;
	euler.pitch = - euler.pitch * 180.0f / PI;
    
    return euler;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ������
// ����˵��     ax��x�����ϵļ��ٶ�;  az��z�����ϵļ��ٶ�;
// ���ز���     void
// ʹ��ʾ��     imu_get_pitch();
//-------------------------------------------------------------------------------------------------------------------
float imu_get_pitch(void)
{
    float ax, az;

    imu660ra_get_acc();//��ȡ���ٶ�

    ax = imu660ra_acc_transition(imu660ra_acc_x);
    az = imu660ra_acc_transition(imu660ra_acc_z);

    return atan2(ax, az) * 180.0f / PI;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡz������ٶ�
// ����˵��     az��z�����ϵļ��ٶ�;
// ���ز���     void
// ʹ��ʾ��     imu_get_az();
//-------------------------------------------------------------------------------------------------------------------
float imu_get_az(void)
{
    imu660ra_get_acc();

    return imu660ra_acc_transition(imu660ra_acc_z);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     �ж��Ƿ���ǽ��
// ����˵��     pitch��������;  az��z�����ϵļ��ٶ�;
// ���ز���     void
// ʹ��ʾ��     imu_is_on_wall();
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
// �������     ֱ�Ӹ����ȵ���
// ����˵��     pitch��������;  az��z�����ϵļ��ٶ�; is_on_wall���Ƿ���ǽ��;
// ���ز���     float *pitch, float *az, bool *is_on_wall
// ʹ��ʾ��     ***
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