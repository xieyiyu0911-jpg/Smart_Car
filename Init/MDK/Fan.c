#include "headfile.h"
#include "Fan.h"
#include "IMU.h"

#define FAN_PWM_CH                (PWMB_CH1_P74)
#define FAN_PWM_FREQ              (50U)
#define FAN_CTRL_PERIOD_MS        (10U)
#define FAN_ESC_ARM_TIME_MS       (3000U)
#define FAN_STARTUP_TIME_MS       (800U)
#define FAN_STARTUP_COUNT         (FAN_STARTUP_TIME_MS / FAN_CTRL_PERIOD_MS)

#define FAN_DUTY_STOP             (500.0f)
#define FAN_DUTY_STARTUP          (900.0f)
#define FAN_DUTY_CRUISE_BASE      (780.0f)  //标准占空比
#define FAN_DUTY_CRUISE_MIN       (650.0f)  //最小占空比
#define FAN_DUTY_CRUISE_MAX       (950.0f)  //最大占空比
#define FAN_DUTY_UPHILL_ADD       (80.0f)  //上坡加
#define FAN_DUTY_DOWNHILL_SUB     (40.0f)  //下坡剪
#define FAN_PITCH_UPHILL_TH       (10.0f)  //俯仰角>10上坡
#define FAN_PITCH_DOWNHILL_TH     (-10.0f)  //俯仰角<-10下坡

typedef enum
{
    FAN_OFF = 0,
    FAN_STARTUP,
    FAN_CRUISING
} Fan_State_t;

static Fan_State_t fan_state = FAN_OFF;
static float fan_duty_cycle = FAN_DUTY_STOP;

//占空比限幅
static void fan_set_duty(float duty)
{
    if (duty < FAN_DUTY_STOP)
    {
        duty = FAN_DUTY_STOP;
    }
    else if (duty > FAN_DUTY_CRUISE_MAX)
    {
        duty = FAN_DUTY_CRUISE_MAX;
    }

    fan_duty_cycle = duty;
    pwm_duty(FAN_PWM_CH, (uint32)fan_duty_cycle);
}

//风扇停转
static void fan_stop(void)
{
    fan_state = FAN_OFF;
    fan_set_duty(FAN_DUTY_STOP);
}

void fan_init(void)
{
    pwm_init(FAN_PWM_CH, FAN_PWM_FREQ, (uint32)FAN_DUTY_STOP);
    delay_ms(FAN_ESC_ARM_TIME_MS);
    fan_stop();
}

void update_fan_control(void)
{
    static uint16 startup_count = 0;
    float pitch;
    float az;
    float adjust;
    float target_duty;
    uint8 is_on_wall;

    imu_get_wall_state(&pitch, &az, &is_on_wall);

    switch (fan_state)
    {
        case FAN_OFF:
        {
            if (is_on_wall)
            {
                fan_state = FAN_STARTUP;
                startup_count = 0;
                fan_set_duty(FAN_DUTY_STARTUP);
            }
            break;
        }

        case FAN_STARTUP:
        {
            if (!is_on_wall)
            {
                fan_stop();
                startup_count = 0;
                break;
            }

            startup_count++;
            if (startup_count >= FAN_STARTUP_COUNT)
            {
                fan_state = FAN_CRUISING;
                fan_set_duty(FAN_DUTY_CRUISE_BASE);
            }
            break;
        }

        case FAN_CRUISING:
        {
            if (!is_on_wall)
            {
                fan_stop();
                startup_count = 0;
                break;
            }

            adjust = 0.0f;
            if (pitch > FAN_PITCH_UPHILL_TH)
            {
                adjust = FAN_DUTY_UPHILL_ADD;
            }
            else if (pitch < FAN_PITCH_DOWNHILL_TH)
            {
                adjust = -FAN_DUTY_DOWNHILL_SUB;
            }

            target_duty = FAN_DUTY_CRUISE_BASE + adjust;
            fan_duty_cycle = 0.9f * fan_duty_cycle + 0.1f * target_duty;

            if (fan_duty_cycle > FAN_DUTY_CRUISE_MAX)
            {
                fan_duty_cycle = FAN_DUTY_CRUISE_MAX;
            }
            else if (fan_duty_cycle < FAN_DUTY_CRUISE_MIN)
            {
                fan_duty_cycle = FAN_DUTY_CRUISE_MIN;
            }

            fan_set_duty(fan_duty_cycle);
            break;
        }

        default:
        {
            fan_stop();
            startup_count = 0;
            break;
        }
    }
}
