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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ==================== TB6612  ====================
#define AIN1_H()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)
#define AIN1_L()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET)
#define AIN2_H()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)
#define AIN2_L()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)

#define BIN1_H()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define BIN1_L()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define BIN2_H()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)
#define BIN2_L()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)

#define PWM_L_SET(duty)  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty)
#define PWM_R_SET(duty)  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, duty)

#define PWM_MAX    999      //TIM2最大限幅
#define PWM_BASE   400      // 巡航速度
#define PWM_TURN   600      // 转弯速度

// ==================== 8路循迹  ====================
#define TRACK_READ()  (\
  (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) << 0) |\
  (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) << 1) |\
  (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) << 2) |\
  (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) << 3) |\
  (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)  << 4) |\
  (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)  << 5) |\
  (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6)  << 6) |\
  (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)  << 7) )

// ==================== 红色边界线检测（仓库/卸载/启停区） ====================
// Hong1=PC2, Hong2=PC3，两个都检测到红线触发停车
#define BORDER_READ()  ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) << 0) | (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) << 1))
#define BORDER_TRIGGER 0x03  // 两个红外都检测到红线，触发停车，可改

  // ====================  编码器距离计算  ====================

#define WHEEL_DIAMETER  65    // 65mm
#define ENCODER_LINES   13    // 编码器每转13线，单边每转26个脉冲，双边每转52个脉冲
#define GEAR_RATIO      30    // 减速比30:1
#define PULSE_PER_ROUND (ENCODER_LINES * GEAR_RATIO * 4)  // 每转脉冲数（4倍频）=编码器线数*减速比*4
#define MM_PER_PULSE    (3.1415926 * WHEEL_DIAMETER / PULSE_PER_ROUND)  // 每个脉冲对应的距离（mm）

// ==================== 按键宏定义（3个按键设置卸货顺序） ====================
// KEY1=PC0(对应A), KEY2=PC1(对应B), KEY3=PB5(对应C)
#define KEY1_READ()  HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)
#define KEY2_READ()  HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1)
#define KEY3_READ()  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)
#define KEY_PRESSED  0  // 按键按下为低电平（可根据你的硬件反转）

// ==================== 循迹PID参数（你可以自己调参学习） ====================
#define TRACK_KP  0.8f
#define TRACK_KI  0.01f
#define TRACK_KD  0.3f

// ====================  车辆状态  ====================
typedef enum {
    STATE_IDLE = 0,        // 待机状态：设置卸货顺序，等待启动
    STATE_START,           // 任务启动，出发
    STATE_GO_WAREHOUSE,    // 前往仓库区
    STATE_LOAD_GOODS,      // 仓库区停车，执行装货
    STATE_GO_UNLOAD,       // 前往对应卸载区
    STATE_UNLOAD_GOODS,    // 卸载区停车，执行卸货
    STATE_RETURN_HOME,     // 所有卸货完成，返回启停区
    STATE_FINISH           // 任务完成，停车
} Car_State;

// ==================== 货物枚举 ====================
typedef enum {
    GOODS_A = 0,
    GOODS_B = 1,
    GOODS_C = 2
} Goods_Type;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 核心状态变量
Car_State g_car_state = STATE_IDLE;  // 初始为待机状态
Goods_Type g_unload_order[3] = {GOODS_A, GOODS_B, GOODS_C};  // 默认卸货顺序A→B→C
uint8_t g_unload_index = 0;           // 当前要卸货的序号
uint8_t g_goods_loaded = 0;           // 货物装载完成标志 0=未装 1=已装

// 机械臂接口标志位（和你后续机械臂代码对接）
uint8_t g_arm_done_flag = 0;  // 机械臂动作完成标志 0=未完成 1=已完成

// 循迹相关变量
uint8_t g_track_raw = 0;        // 循迹传感器原始值
int8_t  g_track_error = 0;      // 循迹偏差
float   g_track_pid_out = 0;    // PID输出值

// PID历史变量
int8_t  g_track_error_last = 0;
float   g_track_integral = 0;

// 编码器&距离变量
int32_t g_total_distance = 0;   // 总行驶距离(mm)
int16_t g_encoder_left = 0;
int16_t g_encoder_right = 0;

// 按键相关变量
uint8_t g_key_val = 0;
uint8_t g_order_set_step = 0;   // 顺序设置步骤 0=未开始 1=选第一个 2=选第二个

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

//电机控制函数
void Motor_Stop(void);
void Motor_Forward(uint16_t pwm_l, uint16_t pwm_r);
void Motor_Turn_Left(uint16_t pwm);
void Motor_Turn_Right(uint16_t pwm);

// 循环传感器读取和编码器距离计算
void Track_Sensor_Read(void);   //循环传感器读取
void Encoder_Distance_Calc(void);   //编码器距离计算

// 状态机函数
void Car_State_Machine(void);     //状态机

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//PTD / PD / PM / PV / PFP / 0 / 1 / 2 / 3

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
//系统初始化，在执行前初始化      

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
//系统时钟配置

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
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
//系统初始化，在执行前初始化      

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //循环执行状态机
      Car_State_Machine();
      HAL_Delay(10);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
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
//电机控制函数实现

// ====================== 电机控制 ======================
void Motor_Stop(void)
{
  AIN1_L(); AIN2_L();
  BIN1_L(); BIN2_L();
  PWM_L_SET(0);
  PWM_R_SET(0);
}

void Motor_Forward(uint16_t pwm_l, uint16_t pwm_r)
{
  if(pwm_l > PWM_MAX) pwm_l = PWM_MAX;
  if(pwm_r > PWM_MAX) pwm_r = PWM_MAX;

  AIN1_H(); AIN2_L();
  BIN1_H(); BIN2_L();

  PWM_L_SET(pwm_l);
  PWM_R_SET(pwm_r);
}

void Motor_Turn_Left(uint16_t pwm)
{

  
}

void Motor_Turn_Right(uint16_t pwm)
{
  // 
}

// ====================== 8路循迹 ======================
void Track_Sensor_Read(void)
{
  g_track_raw = TRACK_READ();

  switch(g_track_raw)
  {
    case 0x01: g_track_error = -7; break;
    case 0x02: g_track_error = -5; break;
    case 0x04: g_track_error = -3; break;
    case 0x08: g_track_error = -1; break;
    case 0x10: g_track_error = 1;  break;
    case 0x20: g_track_error = 3;  break;
    case 0x40: g_track_error = 5;  break;
    case 0x80: g_track_error = 7;  break;
    default:
      g_track_error = (g_track_error < 0) ? -2 : 2;
      break;
  }
}

// ====================== 编码器距离计算 ======================
void Encoder_Distance_Calc(void)
{
  int16_t l = __HAL_TIM_GET_COUNTER(&htim3);
  int16_t r = __HAL_TIM_GET_COUNTER(&htim4);

  __HAL_TIM_SET_COUNTER(&htim3, 0);
  __HAL_TIM_SET_COUNTER(&htim4, 0);

  g_distance += (l + r) / 2;
}

// ====================== 状态机 ======================
void Car_State_Machine(void)
{
  switch(g_car_state)
  {
    case STATE_TRACKING:
      Track_Sensor_Read();
      Encoder_Distance_Calc();
      Motor_Forward(PWM_BASE, PWM_BASE);
      break;

    case STATE_STOP_WAREHOUSE:
      Motor_Stop();
      if(g_arm_done_flag)
      {
        g_arm_done_flag = 0;
        g_car_state = STATE_TRACKING;
      }
      break;

    case STATE_STOP_UNLOAD:
      Motor_Stop();
      if(g_arm_done_flag)
      {
        g_arm_done_flag = 0;
        g_unload_count++;
        g_car_state = (g_unload_count >=3) ? STATE_RETURN_HOME : STATE_TRACKING;
      }
      break;

    case STATE_RETURN_HOME:
      break;

    case STATE_FINISH:
      Motor_Stop();
      break;
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
