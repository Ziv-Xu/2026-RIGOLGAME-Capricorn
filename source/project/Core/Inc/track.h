#ifndef __TRACK_H
#define __TRACK_H

#include "main.h"

void Track_Init(void);
uint8_t Track_Read_All(void);
int Get_Track_Error(void);          // 返回加权偏差（整数）

void Track_PID_Init(float kp, float ki, float kd, float out_max);
float Track_PID_Calc(int error);    // 返回 PID 输出（浮点）
void Track_PID_Reset(void);

#endif