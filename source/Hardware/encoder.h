#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"
#include "encoder.h"

//#define WHEEL_DIAMETER  65.0f
//#define ENCODER_LINES   13
//#define GEAR_RATIO      30
//#define PULSE_PER_ROUND (ENCODER_LINES * GEAR_RATIO * 4)
//#define MM_PER_PULSE    (3.1415926f * WHEEL_DIAMETER / PULSE_PER_ROUND)

/*============= 编码器测距相关 ============*/
#define PI                  3.1415926f  
#define WHEEL_DIAMETER_MM   48.0f      // 车轮直径48mm
#define PPR                 13.0f		 //编码器线数
#define GEAR_RATIO          20.409f	 //减速比
#define PULSE_PER_WHEEL_REV (PPR * GEAR_RATIO) // 265.317 
#define PULSE_TO_DISTANCE   ( (PI * WHEEL_DIAMETER_MM) / PULSE_PER_WHEEL_REV )




void Encoder_Init(void);
void Encoder_Update(void);
int16_t Encoder_GetDistance(void);
void Encoder_ResetDistance(void);


#endif
