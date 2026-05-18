#ifndef __IMU_H__
#define __IMU_H__

#define SAMPLE_FREQ  0.005f

typedef struct
{
    float gyro_x;
    float gyro_y;
    float gyro_z;
} xdata imu_data_t;

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} Quaternion;

typedef struct
{
    float roll;
    float pitch;
    float yaw;
} EulerAngle;

extern imu_data_t imu_data;
extern Quaternion q;
extern EulerAngle euler;
extern float Yaw_Angular_Speed;

void quaternion_init(void);
void gyro_calibrate(uint16 samples);
float custom_atan2(float y, float x);

void quaternion_update(void);
EulerAngle quaternion_to_euler(Quaternion q);
Quaternion get_quaternion(void);
void quat_normalize(Quaternion* q);
float imu_get_pitch(void);
float imu_get_az(void);
uint8 imu_is_on_wall(void);
void imu_get_wall_state(float *pitch, float *az, uint8 *is_on_wall);

float IMU_lvbo(float cy);

#endif
