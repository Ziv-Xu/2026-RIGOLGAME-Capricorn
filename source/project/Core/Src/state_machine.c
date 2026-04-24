#include "state_machine.h"
#include "oled.h"
#include "motor.h"
#include "track.h"
#include "encoder.h"
#include "arm.h"
#include "button.h"
#include "uart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* 全局变量定义 */
SystemState g_state = STATE_IDLE;
uint8_t g_car_dir = 0;
int32_t g_target_enc = 0;

uint8_t g_arm_done_event = 0;
uint8_t g_vision_done_event = 0;

char g_unload_order[4] = "ABC";
uint8_t g_pick_index = 0;
uint8_t g_order_index = 0;
uint8_t g_unload_count = 0;

/* 回调：机械手动作完成时调用 */
void Arm_Done_Callback(void)
{
    g_arm_done_event = 1;
}

/* ====================== 状态机主函数 ====================== */
void State_Machine(void)
{
    uint8_t key;
    int32_t cur_dist;

    switch(g_state)
    {
        case STATE_IDLE:
            OLED_Show_State("Press KEY1 start");
            key = Key_Scan();
            if(key == 1)
            {
                Start_Order_Setting();
            }
            break;

        case STATE_ORDER_SETTING:
            // 订单设置完成后会自动切换到下一状态
            break;

        case STATE_READY:
            OLED_Show_State("Ready! Press KEY1");
            key = Key_Scan();
            if(key == 1)
            {
                g_pick_index = 0;
                g_unload_count = 0;
                g_order_index = 0;
                Encoder_ResetDistance();
                g_target_enc = ENC_POS_WAREHOUSE;
                g_state = STATE_GO_WAREHOUSE;
                OLED_Clear();
            }
            break;

        case STATE_GO_WAREHOUSE:
            Execute_LineTracking(g_target_enc, 0);
            if(Encoder_GetDistance() >= g_target_enc)
            {
                Motor_Stop();
                g_state = STATE_ALIGN_WAREHOUSE;
                OLED_Clear();
            }
            break;

        case STATE_ALIGN_WAREHOUSE:
            OLED_Show_State("Aligning...");
            OpenMV_SetMode(3);
            Arm_MoveToPoint(ARM_POINT_ALIGN);
            if(Wait_AlignComplete(5000))
            {
                OLED_Show_AlignResult(1);
                Arm_MoveToPoint(ARM_POINT_RESET);
                g_state = STATE_PICK_GOODS;
                g_arm_done_event = 0;
                Arm_ClearDoneFlag();
            }
            else
            {
                OLED_Show_AlignResult(0);
                g_state = STATE_PICK_GOODS;
            }
            break;

        case STATE_PICK_GOODS:
            if(g_pick_index < 3)
            {
                char buf[16];
                sprintf(buf, "Pick %c", 'A' + g_pick_index);
                OLED_Show_State(buf);
                if(!Arm_IsDone())
                {
                    Arm_PickSequence(g_pick_index);
                }
                if(g_arm_done_event)
                {
                    g_arm_done_event = 0;
                    Arm_ClearDoneFlag();
                    g_pick_index++;
                    Beep(100);
                    LED_Blink(1);
                }
            }
            else
            {
                g_unload_count = 0;
                g_order_index = 0;
                char target_char = g_unload_order[g_order_index];
                if(target_char == 'A')      g_target_enc = ENC_POS_A;
                else if(target_char == 'B') g_target_enc = ENC_POS_B;
                else                        g_target_enc = ENC_POS_C;
                g_state = STATE_GO_UNLOAD;
                OLED_Clear();
            }
            break;

        case STATE_GO_UNLOAD:
            cur_dist = Encoder_GetDistance();
            uint8_t dir = (g_target_enc > cur_dist) ? 0 : 1;
            Execute_LineTracking(g_target_enc, dir);
            if((dir == 0 && cur_dist >= g_target_enc) || (dir == 1 && cur_dist <= g_target_enc))
            {
                Motor_Stop();
                g_state = STATE_ALIGN_UNLOAD;
                OLED_Clear();
            }
            break;

        case STATE_ALIGN_UNLOAD:
            OLED_Show_State("Align Unload");
            OpenMV_SetMode(3);
            Arm_MoveToPoint(ARM_POINT_ALIGN);
            if(Wait_AlignComplete(5000))
            {
                OLED_Show_AlignResult(1);
                Arm_MoveToPoint(ARM_POINT_RESET);
                g_state = STATE_DROP_GOODS;
                g_arm_done_event = 0;
                Arm_ClearDoneFlag();
            }
            else
            {
                g_state = STATE_DROP_GOODS;
            }
            break;

        case STATE_DROP_GOODS:
            {
                char buf[16];
                sprintf(buf, "Drop %c", g_unload_order[g_order_index]);
                OLED_Show_State(buf);
                if(!Arm_IsDone())
                {
                    Arm_DropSequence();
                }
                if(g_arm_done_event)
                {
                    g_arm_done_event = 0;
                    Arm_ClearDoneFlag();
                    Beep(200);
                    LED_Blink(2);
                    g_order_index++;
                    g_unload_count++;
                    if(g_unload_count < 3)
                    {
                        char next = g_unload_order[g_order_index];
                        if(next == 'A')      g_target_enc = ENC_POS_A;
                        else if(next == 'B') g_target_enc = ENC_POS_B;
                        else                 g_target_enc = ENC_POS_C;
                        g_state = STATE_GO_UNLOAD;
                    }
                    else
                    {
                        g_target_enc = 0;
                        g_state = STATE_RETURN_HOME;
                    }
                    OLED_Clear();
                }
            }
            break;

        case STATE_RETURN_HOME:
            cur_dist = Encoder_GetDistance();
            dir = (0 > cur_dist) ? 0 : 1;
            Execute_LineTracking(0, dir);
            if((dir == 0 && cur_dist >= 0) || (dir == 1 && cur_dist <= 0))
            {
                Motor_Stop();
                g_state = STATE_FINISH;
            }
            break;

        case STATE_FINISH:
            OLED_Show_State("Task Finish!");
            Beep(500);
            HAL_Delay(2000);
            g_state = STATE_IDLE;
            OLED_Clear();
            break;
    }
}

/* 订单设置 */
void Start_Order_Setting(void)
{
    char order[4] = "   ";
    uint8_t idx = 0;
    uint8_t key;
    OLED_Clear();
    OLED_Show_Order_Setting(order, idx);
    while(idx < 3)
    {
        key = Key_Scan();
        if(key)
        {
            order[idx] = (key == 1) ? 'A' : (key == 2) ? 'B' : 'C';
            OLED_Show_Order_Setting(order, idx);
            idx++;
            HAL_Delay(300);
        }
        HAL_Delay(10);
    }
    order[3] = '\0';
    strcpy(g_unload_order, order);
    OLED_Clear();
    OLED_ShowString(0, 0, "Order Saved:");
    OLED_ShowString(0, 1, order);
    HAL_Delay(1500);
    g_state = STATE_READY;
}

/* 循迹执行 */
void Execute_LineTracking(int32_t target_enc, uint8_t direction)
{
    Track_Sensor_Read();
    Track_PID_Calc(TRACK_KP, TRACK_KI, TRACK_KD);
    Encoder_Update();
    float pid_out = Get_Track_PID_Out();
    uint16_t base_pwm = (direction == 0) ? PWM_BASE : PWM_BACK;
    int16_t l = base_pwm - pid_out;
    int16_t r = base_pwm + pid_out;
    if(l < 0) l = 0;
    if(l > 999) l = 999;
    if(r < 0) r = 0;
    if(r > 999) r = 999;

    if(direction == 0)
        Motor_Forward(l, r);
    else
        Motor_Backward(l, r);
    g_car_dir = direction;
}

/* 等待视觉对齐完成 */
uint8_t Wait_AlignComplete(uint32_t timeout_ms)
{
    uint32_t tick = 0;
    uint8_t flag;
    while(tick < timeout_ms)
    {
        flag = Vision_GetAlignFlag();
        if(flag == 1)
        {
            Vision_ClearAlignFlag();
            float offset = Vision_GetOffset();
            if(fabsf(offset) < 0.5f)
                return 1;
        }
        else if(flag == 2)
        {
            Vision_ClearAlignFlag();
            return 1;
        }
        HAL_Delay(10);
        tick += 10;
    }
    return 0;
}