#ifndef __STATE_MACHINE_H
#define __STATE_MACHINE_H

#include "main.h"
#include <stdint.h>

/* 状态枚举 */
typedef enum {
    STATE_IDLE,
    STATE_ORDER_SETTING,
    STATE_READY,
    STATE_GO_WAREHOUSE,
    STATE_ALIGN_WAREHOUSE,
    STATE_PICK_GOODS,
    STATE_GO_UNLOAD,
    STATE_ALIGN_UNLOAD,
    STATE_DROP_GOODS,
    STATE_RETURN_HOME,
    STATE_FINISH
} SystemState;

/* 宏定义  */
#define ENC_POS_WAREHOUSE   1200
#define ENC_POS_A           2500
#define ENC_POS_B           3800
#define ENC_POS_C           5100

#define PWM_BASE     400
#define PWM_BACK     350
#define TRACK_KP     1.2
#define TRACK_KI     0.01
#define TRACK_KD     0.3

/* 函数声明 */
void Arm_Done_Callback(void);          // 机械手完成回调
void State_Machine(void);              // 状态机主函数
void Start_Order_Setting(void);        // 订单设置
void Execute_LineTracking(int32_t target_enc, uint8_t direction); // 循迹执行
uint8_t Wait_AlignComplete(uint32_t timeout_ms);                  // 等待视觉对齐完成
void Beep(uint16_t ms);                                           
void LED_Blink(uint8_t times);                                    

#endif