#include "encoder.h"
#include "motor.h"   // 为了访问 g_encoder_distance

extern int32_t g_encoder_distance;  // 声明 motor.c 中的全局变量

void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    g_encoder_distance = 0;
}

void Encoder_ResetDistance(void)
{
    g_encoder_distance = 0;
}

int32_t Encoder_GetAbsDistance(void)
{
    // 如果需要绝对值，可以累加，这里只简单返回有符号距离的绝对值
    return abs(g_encoder_distance);
}