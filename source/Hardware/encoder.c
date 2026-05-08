#include "encoder.h"
#include <stdlib.h>
#include "extern.h"
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    g_encoder_distance = 0;

}

void Encoder_Update(void)
{
    //int16_t left = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    //int16_t right = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    //g_encoder_distance = (left + right) / 2;
    int16_t left = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    int16_t right = -(int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    int16_t avg = (left + right) / 2; 
	
	

    if(g_car_dir == 0)
        g_encoder_distance += (int16_t)(avg);
    else
        g_encoder_distance -= (int16_t)(avg);
	total_distance_mm = g_encoder_distance * PULSE_TO_DISTANCE;
}

int16_t Encoder_GetDistance(void) { return g_encoder_distance; }
