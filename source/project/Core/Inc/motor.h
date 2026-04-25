#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

/* 速度控制结构体 */
typedef struct {
    float target_speed;   // mm/s
    float current_speed;
    int16_t output;       // PWM 值
    int32_t last_count;
    uint32_t test;
    PID_Controller pid;
} SpeedCtrl;

/* PID 控制器 */
typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    float out_max, out_min;
} PID_Controller;

extern SpeedCtrl g_speed_left, g_speed_right;
extern volatile uint8_t g_motor_control_flag;
extern int32_t g_encoder_distance;   // 全局距离

void Motor_Init(void);
void Motor_Stop(void);
void Motor_Forward(uint16_t pwm_l, uint16_t pwm_r);
void Motor_Backward(uint16_t pwm_l, uint16_t pwm_r);
void Motor(int16_t left_speed, int16_t right_speed);

void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float out_max);
void Motor_SetSpeed(float left_mm_s, float right_mm_s);
void Motor_Control_Loop(void);
int32_t Motor_GetDistance(void);

#endif