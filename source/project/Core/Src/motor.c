#include "motor.h"

/* ========== 硬件 Abstraction ========== */
#define PWM_MAX 999
#define AIN1_H()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)
#define AIN1_L()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET)
#define AIN2_H()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)
#define AIN2_L()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)
#define BIN1_H()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define BIN1_L()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define BIN2_H()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)
#define BIN2_L()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)


extern TIM_HandleTypeDef htim2;   // PWM 定时器
#define PWM_L_SET(d)  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, d)
#define PWM_R_SET(d)  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, d)

#define LEFT_ENC_DIR   1
#define RIGHT_ENC_DIR  1

#define WHEEL_DIAMETER  65.0f
#define ENCODER_LINES   13
#define GEAR_RATIO      30
#define PULSE_PER_ROUND (ENCODER_LINES * GEAR_RATIO * 4)
#define MM_PER_PULSE    (3.1415926f * WHEEL_DIAMETER / PULSE_PER_ROUND)

#define CONTROL_PERIOD_S  0.01f   // 100Hz

SpeedCtrl g_speed_left, g_speed_right;
volatile uint8_t g_motor_control_flag = 0;
int32_t g_encoder_distance = 0;
static int32_t g_abs_distance = 0;

/* ---------- 初始化 ---------- */
void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    Motor_Stop();

    PID_Init(&g_speed_left.pid,  0.5f, 0.1f, 0.02f, PWM_MAX);
    PID_Init(&g_speed_right.pid, 0.5f, 0.1f, 0.02f, PWM_MAX);

    g_speed_left.last_count  = (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    g_speed_right.last_count = (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(&htim4);
}

/* ---------- 底层 PWM 驱动 ---------- */
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

void Motor_Stop(void) { Motor(0,0); }

/* ---------- PID ---------- */
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float out_max)
{
    pid->Kp = kp; pid->Ki = ki; pid->Kd = kd;
    pid->integral = 0; pid->prev_error = 0;
    pid->out_max = out_max; pid->out_min = -out_max;
}

static float PID_Calculate(PID_Controller *pid, float setpoint, float measurement)
{
    float error = setpoint - measurement;
    float Pout = pid->Kp * error;
    pid->integral += error * CONTROL_PERIOD_S;
    // 积分钳位
    float max_i = pid->out_max / (pid->Ki > 0.0001f ? pid->Ki : 1.0f);
    if (pid->integral > max_i) pid->integral = max_i;
    if (pid->integral < -max_i) pid->integral = -max_i;
    float Iout = pid->Ki * pid->integral;
    float derivative = (error - pid->prev_error) / CONTROL_PERIOD_S;
    float Dout = pid->Kd * derivative;
    float output = Pout + Iout + Dout;
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;
    pid->prev_error = error;
    return output;
}

/* ---------- 设置目标速度 ---------- */
void Motor_SetSpeed(float left_mm_s, float right_mm_s)
{
    g_speed_left.target_speed  = left_mm_s;
    g_speed_right.target_speed = right_mm_s;
}

/* ---------- 核心控制循环（主循环中调用） ---------- */
void Motor_Control_Loop(void)
{
    /* 读取编码器增量 */
    int16_t count_l = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    int16_t count_r = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    int16_t delta_l = (count_l - (int16_t)g_speed_left.last_count) * LEFT_ENC_DIR;
    int16_t delta_r = (count_r - (int16_t)g_speed_right.last_count) * RIGHT_ENC_DIR;
    g_speed_left.last_count  = count_l;
    g_speed_right.last_count = count_r;

    /* 速度估计 */
    float speed_l = (delta_l * MM_PER_PULSE) / CONTROL_PERIOD_S;
    float speed_r = (delta_r * MM_PER_PULSE) / CONTROL_PERIOD_S;
    g_speed_left.current_speed  = speed_l;
    g_speed_right.current_speed = speed_r;

    /* 距离累加：根据实际速度方向 */
    if (speed_l > 0 && speed_r > 0)
        g_encoder_distance += (int32_t)((delta_l + delta_r) / 2 * MM_PER_PULSE);
    else if (speed_l < 0 && speed_r < 0)
        g_encoder_distance -= (int32_t)((delta_l + delta_r) / 2 * MM_PER_PULSE);
    // 若左右不同向（原地旋转），不累加净位移

    /* PID */
    float out_l = PID_Calculate(&g_speed_left.pid,  g_speed_left.target_speed,  speed_l);
    float out_r = PID_Calculate(&g_speed_right.pid, g_speed_right.target_speed, speed_r);
    g_speed_left.output  = (int16_t)out_l;
    g_speed_right.output = (int16_t)out_r;

    /* 驱动电机 */
    Motor(g_speed_left.output, g_speed_right.output);
}

int32_t Motor_GetDistance(void) { return g_encoder_distance; }

// 定时器中断回调
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        g_motor_control_flag = 1;
    }
}