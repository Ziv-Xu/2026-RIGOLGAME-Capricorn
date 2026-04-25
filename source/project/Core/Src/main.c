/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 编码器目标位置宏定义（单位：mm），请根据实际场地调整
#define ENC_POS_WAREHOUSE   1200    // HOME -> 仓库区
#define ENC_POS_A           2500    // HOME -> A区（累计）
#define ENC_POS_B           3800    // HOME -> B区
#define ENC_POS_C           5100    // HOME -> C区

// #define PWM_BASE     400
// #define PWM_BACK     350
#define TRACK_KP     1.2
#define TRACK_KI     0.01
#define TRACK_KD     0.3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// ==================== 状态机 ====================
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

//小车相关
SystemState g_state = STATE_IDLE;   // 初始小车状态变量
uint8_t g_car_dir = 0;              // 小车行进状态：0:前进，1:后退
int32_t g_target_enc = 0;           // 当前目标编码器距离

//机械臂相关
uint8_t g_arm_done_event = 0;       // 机械臂动作完成事件
uint8_t g_vision_done_event = 0;    // 视觉对准完成事件

//物块相关
char g_unload_order[4] = "ABC";     // 默认顺序
uint8_t g_pick_index = 0;           // 抓取索引 0:A,1:B,2:C
uint8_t g_order_index = 0;          // 当前卸货索引序号
uint8_t g_unload_count = 0;         // 已卸货物计数



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void State_Machine(void);
void Start_Order_Setting(void);
void Execute_LineTracking(int32_t target_enc, uint8_t direction);
uint8_t Wait_AlignComplete(uint32_t timeout_ms);
void Beep(uint16_t ms);
void LED_Blink(uint8_t times);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Arm_Done_Callback(void)
{
    g_arm_done_event = 1;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    Motor_Init();
    Track_Init();
    Track_PID_Init(12.0, 0.0, 6.0, 150.0);  // 输出限幅 ±150 mm/s
    Encoder_Init();
    OLED_Init();
    OLED_Clear();
    Arm_Init();
    Arm_SetDoneCallback(Arm_Done_Callback);
    UART_Init();            // 开启串口接收中断

    OLED_ShowString(0, 0, "System Ready!");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    State_Machine();       // 依然由状态机控制

     if (g_motor_control_flag) {
        g_motor_control_flag = 0;
        Motor_Control_Loop();   // 10ms 执行一次，更新速度和距离
    }


    HAL_Delay(50);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// ====================== 状态机实现 ======================
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
            // 此状态在Start_Order_Setting中处理，处理完直接自动跳转
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
            Execute_LineTracking(g_target_enc, 0);  // 前进
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
                // 超时也继续，可加声光报警
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
                // 抓取完毕，准备前往第一个卸货区
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
                        g_target_enc = 0;   // 返回HOME
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

/* 顺序设置函数（阻塞式，但仅在设置时运行，因此不影响任务执行） */
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

/* 巡线执行函数（非阻塞，每次调用只执行一次PID计算与电机输出） */
void Execute_LineTracking(int32_t target_enc, uint8_t direction)
{
    // 1. 读取传感器
    Track_Sensor_Read();
    // 2. 循迹 PID 输出速度修正量（mm/s）
    int error = Get_Track_Error();
    float turn_comp = Track_PID_Calc(error);
    // 3. 基础线速度（mm/s）
    float base_speed = (direction == 0) ? 200.0f : -200.0f;
    // 4. 左右目标速度
    float left_target  = base_speed - turn_comp;
    float right_target = base_speed + turn_comp;
    // 5. 设置到电机控制模块
    Motor_SetSpeed(left_target, right_target);
    
    // 方向记录（用于距离显示等，可选）
    g_car_dir = direction;
}


/* 等待视觉对准完成（带超时，内部使用HAL_Delay，不中断其他中断） */
uint8_t Wait_AlignComplete(uint32_t timeout_ms)
{
    uint32_t tick = 0;
    uint8_t flag;
    while(tick < timeout_ms)
    {
        flag = Vision_GetAlignFlag();
        if(flag == 1)   // 收到偏移数据
        {
            Vision_ClearAlignFlag();
            float offset = Vision_GetOffset();
            if(fabsf(offset) < 0.5f)
                return 1;
            // 这里可加入微调代码，省略
        }
        else if(flag == 2)  // 收到OK信号
        {
            Vision_ClearAlignFlag();
            return 1;
        }
        HAL_Delay(10);
        tick += 10;
    }
    return 0;
}

void Beep(uint16_t ms)
{
    // 实际蜂鸣器代码，按需实现
    // HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    // HAL_Delay(ms);
    // HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
}

void LED_Blink(uint8_t times)
{
    for(uint8_t i=0; i<times; i++)
    {
        // LED_ON(); HAL_Delay(100); LED_OFF(); HAL_Delay(100);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */