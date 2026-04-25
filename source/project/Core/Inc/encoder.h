#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

void Encoder_Init(void);
int32_t Encoder_GetAbsDistance(void);
void Encoder_ResetDistance(void);  // 会清零 motor 中的距离变量

#endif