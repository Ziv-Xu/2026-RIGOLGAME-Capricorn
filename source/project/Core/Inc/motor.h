#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

void Motor_Stop(void);
void Motor_Forward(uint16_t pwm_l, uint16_t pwm_r);
void Motor_Backward(uint16_t pwm_l, uint16_t pwm_r);
void Motor_Turn_Left(uint16_t pwm);
void Motor_Turn_Right(uint16_t pwm);

#endif