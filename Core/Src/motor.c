#include "motor.h"

#define PWM_MAX 999

extern TIM_HandleTypeDef htim8;

#define AIN1_H()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)
#define AIN1_L()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET)
#define AIN2_H()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)
#define AIN2_L()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)

#define BIN1_H()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define BIN1_L()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define BIN2_H()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)
#define BIN2_L()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)

#define PWM_L_SET(duty)  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, duty)
#define PWM_R_SET(duty)  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, duty)

void Motor_Stop(void)
{
    AIN1_L(); AIN2_L();
    BIN1_L(); BIN2_L();
    PWM_L_SET(0);
    PWM_R_SET(0);
}

void Motor_Forward(uint16_t pwm_l, uint16_t pwm_r)
{
    if(pwm_l > PWM_MAX) pwm_l = PWM_MAX;
    if(pwm_r > PWM_MAX) pwm_r = PWM_MAX;
    AIN1_H(); AIN2_L();
    BIN1_H(); BIN2_L();
    PWM_L_SET(pwm_l);
    PWM_R_SET(pwm_r);
}

void Motor_Backward(uint16_t pwm_l, uint16_t pwm_r)
{
    if(pwm_l > PWM_MAX) pwm_l = PWM_MAX;
    if(pwm_r > PWM_MAX) pwm_r = PWM_MAX;
    AIN1_L(); AIN2_H();
    BIN1_L(); BIN2_H();
    PWM_L_SET(pwm_l);
    PWM_R_SET(pwm_r);
}

void Motor_Turn_Left(uint16_t pwm)
{
    if(pwm > PWM_MAX) pwm = PWM_MAX;
    uint16_t pwm_left = pwm * 60 / 100;
    uint16_t pwm_right = pwm;
    AIN1_H(); AIN2_L();
    BIN1_H(); BIN2_L();
    PWM_L_SET(pwm_left);
    PWM_R_SET(pwm_right);
}

void Motor_Turn_Right(uint16_t pwm)
{
    if(pwm > PWM_MAX) pwm = PWM_MAX;
    uint16_t pwm_left = pwm;
    uint16_t pwm_right = pwm * 60 / 100;
    AIN1_H(); AIN2_L();
    BIN1_H(); BIN2_L();
    PWM_L_SET(pwm_left);
    PWM_R_SET(pwm_right);
}