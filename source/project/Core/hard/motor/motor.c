#include "motor.h"
#include "encoder.h"
#include <stdlib.h>

/* 控制周期，需与定时器中断周期一致（单位：秒） */
#define CONTROL_PERIOD_S  0.01f    // 100Hz → 10ms

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

SpeedCtrl g_speed_left;
SpeedCtrl g_speed_right;

volatile uint8_t g_motor_control_flag = 0;

// ----------------- 原有电机控制函数 -----------------
void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}

void Motor_Stop(void)
{
    AIN1_L(); AIN2_L();
    BIN1_L(); BIN2_L();
    PWM_L_SET(0);
    PWM_R_SET(0);
}

void Motor(int16_t left_speed, int16_t right_speed)
{
    if (left_speed > PWM_MAX) left_speed = PWM_MAX;
    if (left_speed < -PWM_MAX) left_speed = -PWM_MAX;
    if (right_speed > PWM_MAX) right_speed = PWM_MAX;
    if (right_speed < -PWM_MAX) right_speed = -PWM_MAX;

    if (left_speed >= 0) {
        AIN1_H(); AIN2_L();
        PWM_L_SET((uint16_t)left_speed);
    } else {
        AIN1_L(); AIN2_H();
        PWM_L_SET((uint16_t)(-left_speed));
    }

    if (right_speed >= 0) {
        BIN1_H(); BIN2_L();
        PWM_R_SET((uint16_t)right_speed);
    } else {
        BIN1_L(); BIN2_H();
        PWM_R_SET((uint16_t)(-right_speed));
    }
}

void Motor_Forward(uint16_t pwm_l, uint16_t pwm_r)
{
    Motor((int16_t)pwm_l, (int16_t)pwm_r);
}

void Motor_Backward(uint16_t pwm_l, uint16_t pwm_r)
{
    Motor(-(int16_t)pwm_l, -(int16_t)pwm_r);
}

void Motor_Turn_Left(uint16_t pwm)
{
    uint16_t pwm_left = pwm * 60 / 100;
    if (pwm_left > PWM_MAX) pwm_left = PWM_MAX;
    if (pwm > PWM_MAX) pwm = PWM_MAX;
    Motor((int16_t)pwm_left, (int16_t)pwm);
}

void Motor_Turn_Right(uint16_t pwm)
{
    uint16_t pwm_right = pwm * 60 / 100;
    if (pwm_right > PWM_MAX) pwm_right = PWM_MAX;
    if (pwm > PWM_MAX) pwm = PWM_MAX;
    Motor((int16_t)pwm, (int16_t)pwm_right);
}

// ----------------- PID 控制器函数 -----------------
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float out_max)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->out_max = out_max;
    pid->out_min = -out_max;
}

float PID_Calc_Motor(PID_Controller *pid, float setpoint, float measurement)
{
    float error = setpoint - measurement;
    float Pout = pid->Kp * error;

    pid->integral += error * CONTROL_PERIOD_S;

    float max_integral = pid->out_max / (pid->Ki > 0.0001f ? pid->Ki : 1.0f);
    if (pid->integral >  max_integral) pid->integral =  max_integral;
    if (pid->integral < -max_integral) pid->integral = -max_integral;
    float Iout = pid->Ki * pid->integral;

    float derivative = (error - pid->prev_error) / CONTROL_PERIOD_S;
    float Dout = pid->Kd * derivative;
    pid->prev_error = error;

    float output = Pout + Iout + Dout;
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;
    return output;
}

// ----------------- 速度控制初始化 -----------------
void Motor_Control_Init(void)
{
    HAL_TIM_Base_Start_IT(&htim1);

    PID_Init(&g_speed_left.pid,  0.5f, 0.1f, 0.02f, PWM_MAX);
    PID_Init(&g_speed_right.pid, 0.5f, 0.1f, 0.02f, PWM_MAX);

    g_speed_left.target_speed = 0.0f;
    g_speed_right.target_speed = 0.0f;
    g_speed_left.current_speed = 0.0f;
    g_speed_right.current_speed = 0.0f;
    g_speed_left.output = 0;
    g_speed_right.output = 0;
    g_speed_left.test = 0;
    g_speed_right.test = 0;

    g_speed_left.last_count  = (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    g_speed_right.last_count = (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(&htim4);
}

void Motor_SetSpeed(float left_mm_s, float right_mm_s)
{
    g_speed_left.target_speed  = left_mm_s;
    g_speed_right.target_speed = right_mm_s;
}

// ----------------- 控制主循环（由主循环调用，不再由中断直接调用） -----------------
void Motor_Control_Loop(void)
{
    /* 1. 读取当前编码器计数值（不清零） */
    int16_t count_l = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    int16_t count_r = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);

    /* 2. 计算原始增量 */
    int16_t delta_l_raw = count_l - (int16_t)g_speed_left.last_count;
    int16_t delta_r_raw = count_r - (int16_t)g_speed_right.last_count;

    // 记录本次原始值（不清零）
    g_speed_left.last_count  = count_l;
    g_speed_right.last_count = count_r;

    /* 3. 方向修正 */
    int16_t delta_l = delta_l_raw * LEFT_ENC_DIR;
    int16_t delta_r = delta_r_raw * RIGHT_ENC_DIR;

    /* 4. 速度换算 */
    float speed_l = (delta_l * MM_PER_PULSE) / CONTROL_PERIOD_S;
    float speed_r = (delta_r * MM_PER_PULSE) / CONTROL_PERIOD_S;

    g_speed_left.current_speed  = speed_l;
    g_speed_right.current_speed = speed_r;

    /* 5. 距离累加（有符号净距离，与 encoder.c 共享全局变量） */
    extern uint8_t g_car_dir;  // 若需区分前进后退可再用，此处简单：前进累加正，后退累加负
    // 直接用 speed 的符号，无需 g_car_dir
    g_encoder_distance += (int32_t)((delta_l + delta_r) / 2 * MM_PER_PULSE);
    // 绝对值距离
    g_abs_distance += (uint32_t)((abs(delta_l_raw) + abs(delta_r_raw)) * MM_PER_PULSE / 2);

    /* 6. PID 计算 */
    float out_l = PID_Calc_Motor(&g_speed_left.pid,  g_speed_left.target_speed,  speed_l);
    float out_r = PID_Calc_Motor(&g_speed_right.pid, g_speed_right.target_speed, speed_r);

    g_speed_left.output  = (int16_t)out_l;
    g_speed_right.output = (int16_t)out_r;

    /* 7. 驱动电机 */
    Motor(g_speed_left.output, g_speed_right.output);
}

// ----------------- 定时器中断回调（只置标志） -----------------
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        g_speed_left.test++;
        g_speed_right.test++;
        g_motor_control_flag = 1;   // 通知主循环
       // Motor_Control_Loop();
    }
}