#include "track.h"
#include "soft_i2c_track.h"

/*==================== 权重配置 ====================*/
static const int IR_Weight[8] = {-4, -3, -2, -1, 1, 2, 3, 4};

/*==================== 浮点 PID 结构 ====================*/
static struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    float out_max, out_min;
} track_pid;

void Track_Init(void)
{
    HAL_Delay(10);
}

uint8_t Track_Read_All(void)
{
    return TRACK_I2C_Read_One_Byte(TRACK_I2C_ADDR, TRACK_REG_DATA);
}

int Get_Track_Error(void)
{
    uint8_t status = Track_Read_All();
    int sum = 0, count = 0;
    for (int i = 0; i < 8; i++) {
        if (status & (1 << i)) {
            sum += IR_Weight[i];
            count++;
        }
    }
    return (count == 0) ? 0 : sum / count;
}

void Track_PID_Init(float kp, float ki, float kd, float out_max)
{
    track_pid.Kp = kp;
    track_pid.Ki = ki;
    track_pid.Kd = kd;
    track_pid.integral = 0;
    track_pid.prev_error = 0;
    track_pid.out_max = out_max;
    track_pid.out_min = -out_max;
}

float Track_PID_Calc(int error)
{
    float p_out = track_pid.Kp * error;
    track_pid.integral += error * 0.01f;  // 假设调用周期 10ms，与主循环一致
    if (track_pid.integral > track_pid.out_max / track_pid.Ki) 
        track_pid.integral = track_pid.out_max / track_pid.Ki;
    if (track_pid.integral < track_pid.out_min / track_pid.Ki) 
        track_pid.integral = track_pid.out_min / track_pid.Ki;
    float i_out = track_pid.Ki * track_pid.integral;
    float d_out = track_pid.Kd * (error - track_pid.prev_error) / 0.01f;
    float output = p_out + i_out + d_out;
    if (output > track_pid.out_max) output = track_pid.out_max;
    if (output < track_pid.out_min) output = track_pid.out_min;
    track_pid.prev_error = error;
    return output;
}

void Track_PID_Reset(void)
{
    track_pid.integral = 0;
    track_pid.prev_error = 0;
}