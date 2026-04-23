#ifndef __ARM_H
#define __ARM_H

#include "main.h"

// 机械臂点位枚举
typedef enum {
    ARM_POINT_RESET = 0,
    ARM_POINT_ALIGN,        // 对准位（底座右转90°）
    ARM_POINT_PICK_A,
    ARM_POINT_PICK_B,
    ARM_POINT_PICK_C,
    ARM_POINT_STORE,        // 车上暂存区
    ARM_POINT_DROP          // 统一卸货位（底座右转90°）
} ArmPoint;

void Arm_Init(void);
void Arm_MoveToPoint(ArmPoint point);
void Arm_PickSequence(uint8_t goods_index); // 0:A,1:B,2:C
void Arm_DropSequence(void);
void Arm_SetDoneCallback(void (*cb)(void));
uint8_t Arm_IsDone(void);
void Arm_ClearDoneFlag(void);

// 电磁铁控制
void Magnet_On(void);
void Magnet_Off(void);

#endif