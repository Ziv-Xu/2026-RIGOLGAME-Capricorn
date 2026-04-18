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

// ==================== 电机控制 ====================
#define AIN1_H()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)
#define AIN1_L()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET)
#define AIN2_H()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)
#define AIN2_L()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)

#define BIN1_H()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define BIN1_L()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define BIN2_H()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)
#define BIN2_L()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)

// PWM控制
#define PWM_L_SET(duty)  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty)
#define PWM_R_SET(duty)  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, duty)

#define PWM_MAX    999      //TIM2最大比较值
#define PWM_BASE   400      // 前进巡线基准PWM
#define PWM_BACK   350      // 后退巡线基准PWM
#define PWM_TURN   300      // 转向巡线基准PWM

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

// ==================== 边框检测（似乎被视觉模块代替了） ====================
// Hong1=PC2, Hong2=PC3
// #define BORDER_READ()  ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) << 0) | (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) << 1))
// #define BORDER_TRIGGER 0x03  // 当两个边界传感器都被触发时，返回0x03

  // ==================== 编码器 ====================

#define WHEEL_DIAMETER  65    // 65mm
#define ENCODER_LINES   13    // 编码器每转的线数
#define GEAR_RATIO      30    // 减速比30:1
#define PULSE_PER_ROUND (ENCODER_LINES * GEAR_RATIO * 4)  // 每转脉冲数=编码器每转线数*减速比*4
#define MM_PER_PULSE    (3.1415926 * WHEEL_DIAMETER / PULSE_PER_ROUND)  // 每脉冲移动距离(mm)

// ==================== 编码器 ====================
// KEY1=PC0(对应A), KEY2=PC1(对应B), KEY3=PB5(对应C)
#define KEY1_READ()  HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)
#define KEY2_READ()  HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1)
#define KEY3_READ()  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)
#define KEY_PRESSED  0  // 按键按下时读取为0

// ==================== 循迹PID参数 ====================
#define TRACK_KP  1.2
#define TRACK_KI  0.01
#define TRACK_KD  0.3

// ==================== 2. 任务模式定义 ====================
#define TASK_MODE_FIXED    0  // 模式1：固定顺序 (A->B->C)
#define TASK_MODE_CUSTOM   1  // 模式2：自定义顺序

// ==================== 1. 节点/点位定义 ====================
typedef enum {
    NODE_HOME = 0,       // 起始区
    NODE_WAREHOUSE,      // 仓库区
    NODE_A,              // 卸货区A
    NODE_B,              // 卸货区B
    NODE_C,              // 卸货区C
    NODE_TRANSITION,     // 过渡位（放物块的地方，暂时采用“斜坡”方案）
} Node_e;

// ==================== 车辆方向状态 ====================
typedef enum {
	FORWARD = 0;	//前进状态
	BACKWARD = 1;	//后退状态
} DERICTION_STATE;

// ==================== 车辆状态 ====================
typedef enum {
    STATE_IDLE = 0,
	STATE_START,
	STATE_GO_WAREHOUSE,
	STATE_ALIGN_CENTER,     // 区域中心前后对准
	STATE_LOAD_ALL_GOODS,   // 装载所有货物
	STATE_GO_UNLOAD_A,
	STATE_UNLOAD_A,
	STATE_GO_UNLOAD_B,
	STATE_UNLOAD_B,
	STATE_GO_UNLOAD_C,
	STATE_UNLOAD_C,
	STATE_RETURN_HOME,
	STATE_FINISH
} Car_State;

// ==================== 货物类型 ====================
typedef enum {
    GOODS_A = 0,
    GOODS_B = 1,
    GOODS_C = 2
} Goods_Type;

// ==================== 机械臂参数 ====================
#define BASE_H 30.0	//机械臂基座高度(mm)
#define L1 100.0	//机械臂第一关节长度(mm)
#define L2 50.0		//机械臂第二关节长度(mm)
#define J0_MIN  -90.0	//底座旋转关节最小角度
#define J0_MAX   90.0	//底座旋转关节最大角度
#define J1_MIN    0.0	//第一关节最小角度
#define J1_MAX   90.0	//第一关节最大角度
#define J2_MIN  -90.0	//第二关节最小角度
#define J2_MAX    0.0	//第二关节最大角度
#define RAD_TO_DEG(r) (r / 3.1415926 * 180.0) //弧度转角度
#define DEG_TO_RAD(d) (d / 180.0 * 3.1415926) //角度转弧度

#define MAGNET_ON()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET)   // 磁铁开启
#define MAGNET_OFF() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET) // 磁铁关闭

// ==================== 机械臂目标位置 ====================
#define ARM_RESET     {0, 45, -80}		//初始位置
#define ARM_ALIGN     {90, 0, -80}		//对准位置
#define ARM_PICK_A 	  {90, 30, -30}		//拾取A位置
#define ARM_PICK_B 	  {90, 30, -30}		//拾取B位置
#define ARM_PICK_C    {90, 30, -30}		//拾取C位置
#define ARM_MOVE      {90, 30, -30}		//移动位置
#define ARM_DROP_A    {90, 80, -30}		//卸载A位置
#define ARM_DROP_B    {-90, 30, -30}	//卸载B位置
#define ARM_DROP_C    {-90, 30, -30}	//卸载C位置
const float arm_point[][3] = {ARM_RESET, ARM_ALIGN, ARM_PICK_A, ARM_PICK_B, ARM_PICK_C, ARM_MOVE, ARM_DROP_A, ARM_DROP_B, ARM_DROP_C};
typedef enum 
{ 
	POINT_RESET=0,
	POINT_ALIGN,
	POINT_PICK_A, 
	POINT_PICK_B, 
	POINT_PICK_C, 
	POINT_MOVE, 
	POINT_DROP_A, 
	POINT_DROP_B, 
	POINT_DROP_C 
} Arm_Point_e;

// ====================串口通信缓存规模 ====================
#define UART_BUF_SIZE 50

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// ==================== 小车状态 ====================
// 小车/模式/初始状态
Car_State g_car_state = STATE_IDLE;  // 初始为 idle 状态
Task_Mode g_task_mode = TASK_MODE_1;	// 默认任务模式1
Car_Dir   g_car_dir = DIR_FORWARD;		// 默认前进方向
//卸载顺序模式1默认装载状态
Goods_Type g_unload_order[3] = {GOODS_A, GOODS_B, GOODS_C};  // 默认卸载顺序为A->B->C
uint8_t g_unload_index = 0;           // 当前要卸载的货物索引
uint8_t g_goods_loaded = 0;           // 货物装载状态 0=未装 1=已装
uint8_t g_current_node = NODE_HOME;		// 当前所在节点（初始节点）	
uint8_t g_target_node = 0;			   // 目标节点（当前任务的目标节点）

// ==================== 物块相关变量 ====================
uint8_t g_goods_index = 0;      // 当前处理第几个物块 (0/1/2)
uint8_t g_unload_index = 0;     // 当前卸货第几个 (0/1/2)

// ==================== 机械臂参数 ====================
// 机械臂运动状态
uint8_t g_arm_done_flag = 0;  // 机械臂运动完成标志 0=未完成 1=已完成
float target_X = 150.0, target_Y = 0.0, target_Z = 100.0;  	// 机械臂目标位置
float delta_X = 0.0, delta_Y = 0.0;				// 机械臂位置微调增量（根据视觉反馈调整）
uint8_t line_flag = 0, align_flag = 0; 			// 视觉对准标志位（line_flag=1表示收到视觉数据，align_flag=1表示对准完成）


// ==================== 串口通信缓存规模 ====================
uint8_t uart_error_flag = 0;		//串口通信错误标志
uint8_t uart_rx_buf[UART_BUF_SIZE], uart_rx_data, uart_rx_cnt = 0;		//串口接收缓冲区，接收数据，接收计数


// ==================== 循环&PID控制 ====================
// 循环传感器数据
uint8_t g_track_raw = 0;        // 循环传感器原始值
int8_t  g_track_error = 0;      // 循环传感器偏差
float   g_track_pid_out = 0;    // PID输出
// PID历史数据
int8_t  g_track_error_last = 0;		//上次偏差
float   g_track_integral = 0;		//偏差积分
// 总行驶距离
int32_t g_total_distance = 0;   // 总行驶距离(mm)
int16_t g_encoder_left = 0;
int16_t g_encoder_right = 0;


// ==================== 按键参数 ====================
// 按键状态
uint8_t g_key_val = 0;
uint8_t g_order_set_step = 0;   // 顺时针设置步骤 0=未开始 1=选择货物 2=选择卸载位置

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// 电机控制
void Motor_Stop(void);
void Motor_Forward(uint16_t pwm_l, uint16_t pwm_r);
void Motor_Backward(uint16_t pwm_l, uint16_t pwm_r);
void Motor_Turn_Left(uint16_t pwm);
void Motor_Turn_Right(uint16_t pwm);

// 传感器读取
void Track_Sensor_Read(void);    // 8路循迹传感器读取+偏差计算
void Encoder_Distance_Calc(void); // 编码器距离计算
uint8_t Border_Check(void);      // 边界检测
uint8_t Key_Scan(void);          // 按键扫描

// PID控制
void Track_PID_Calc(void);       // 循环PID计算
void Unload_Order_Set(void);     // 设置卸载顺序

// 小车状态机
void Task_Switch(void);			// 任务模式切换
uint8_t Path_Plan(uint8_t current, uint8_t target);			// 路径规划
void Car_State_Machine(void);		

// ==================== 机械臂 ====================
void PCA9685_Init(void);
void Servo_SetAngle(uint8_t ch, float angle);
uint8_t Arm_Inverse(float X, float Y, float Z, float *J0, float *J1, float *J2);
void Arm_Forward(float J0, float J1, float J2, float *X, float *Y, float *Z);
void Arm_MoveXYZ(float X, float Y, float Z);
void Arm_MovePoint(Arm_Point_e point_num);
void Arm_Pick(void);
void Arm_Release(void);

// ==================== 视觉处理 ====================
void OpenMV_SetMode(uint8_t mode);
void Car_Align_Center(void)				// 锟竭匡拷巡锟竭讹拷准小锟斤拷
void Block_Align(void);				// 锟斤拷榫硷拷锟阶?

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
//      

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
//系统时锟斤拷锟斤拷锟斤拷

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
	      
	// 1. PWM
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

	// 2. TIM
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

	// 3. OLED锟斤拷示锟斤拷始锟斤拷
	OLED_Init();
	OLED_Clear();
	OLED_ShowString(0, 0, "System Ready!");

	// 4. 锟斤拷锟斤拷锟绞硷拷锟?
	Motor_Stop();

	// 5. PCA9685初始化
	PCA9685_Init();

	// 6. 磁铁关闭
	MAGNET_OFF();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //锟斤拷锟斤拷状态锟斤拷
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
	if(pwm_l < 0) pwm_l = 0;
	if(pwm_r > PWM_MAX) pwm_r = PWM_MAX;
	if(pwm_r < 0) pwm_r = 0;

	AIN1_H(); AIN2_L();
	BIN1_H(); BIN2_L();

	PWM_L_SET(pwm_l);
	PWM_R_SET(pwm_r);
}

void Motor_Backward(uint16_t pwm_l, uint16_t pwm_r)
{
  	if(pwm_l>PWM_MAX) pwm_l=PWM_MAX;
	if(pwm_l<0) pwm_l=0;
  	if(pwm_r>PWM_MAX) pwm_r=PWM_MAX;
	if(pwm_r<0) pwm_r=0;

 	AIN1_L(); AIN2_H();
	BIN1_L(); BIN2_H();
  	PWM_L_SET(pwm_l); 
	PWM_R_SET(pwm_r); 
	g_car_dir=DIR_BACKWARD;
}

void Motor_Turn_Left(uint16_t pwm)
{
    if(pwm > PWM_MAX) pwm = PWM_MAX;
    if(pwm < 0) pwm = 0;

    uint16_t pwm_left = pwm * 60 / 100;
    uint16_t pwm_right = pwm;

    if(pwm_left > PWM_MAX) pwm_left = PWM_MAX;
    if(pwm_right > PWM_MAX) pwm_right = PWM_MAX;
    if(pwm_left < 0) pwm_left = 0;
    if(pwm_right < 0) pwm_right = 0;
	
	AIN1_H(); AIN2_L();
    BIN1_H(); BIN2_L(); 

	PWM_L_SET(pwm_left);
	PWM_R_SET(pwm_right);
}

void Motor_Turn_Right(uint16_t pwm)
{
	if(pwm > PWM_MAX) pwm = PWM_MAX;
    if(pwm < 0) pwm = 0;

    uint16_t pwm_left = pwm;
    uint16_t pwm_right = pwm * 60 / 100;

    if(pwm_left > PWM_MAX) pwm_left = PWM_MAX;
    if(pwm_right > PWM_MAX) pwm_right = PWM_MAX;
    if(pwm_left < 0) pwm_left = 0;
    if(pwm_right < 0) pwm_right = 0;
	
	AIN1_H(); AIN2_L();
    BIN1_H(); BIN2_L(); 

	PWM_L_SET(pwm_left);
	PWM_R_SET(pwm_right);
}

// ====================== 8路循迹 ======================
void Track_Sensor_Read(void)
{
  g_track_raw = TRACK_READ();

  // 8路循迹传感器读取
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
			//没检测到黑线时，根据上次的误差决定偏转方向
			g_track_error = (g_track_error < 0) ? -2 : 2;  
			break;
  }
}

// ====================== 编码器距离计算 ======================
void Encoder_Distance_Calc(void)
{
	g_encoder_left = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
	g_encoder_right = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);

	__HAL_TIM_SET_COUNTER(&htim3, 0);
	__HAL_TIM_SET_COUNTER(&htim4, 0);

	g_distance += (uint32_t)(g_encoder_left + g_encoder_right) * MM_PER_PULSE / 2;
}

// ====================== 边框检测 ======================
uint8_t Border_Check(void)
{
	if(BORDER_READ() == BORDER_TRIGGER)
		return 1; //检测到边框
	else
		return 0; //未检测到边框
}

// ====================== 按键扫描 ======================
uint8_t Key_Scan(void)
{
	static uint8_t key_up =  1;	//初始按键未按下状态

	if(key_up && KEY1_READ() == KEY_PRESSED || KEY2_READ() == KEY_PRESSED || KEY3_READ() == KEY_PRESSED)
	{
		HAL_Delay(10);
		if(KEY1_READ() == KEY_PRESSED) 	   	 return 1;
		else if(KEY2_READ() == KEY_PRESSED)	 return 2; 
		else if(KEY3_READ() == KEY_PRESSED)  return 3; 

		key_up = 0;
	}
	else if(KEY1_READ() == KEY_PRESSED || KEY2_READ() == KEY_PRESSED || KEY3_READ() == KEY_PRESSED)
	{
		key_up = 1;		// 
	}
	return 0; //
}

// ====================== 循环PID计算 ======================
void Track_PID_Calc(void)
{
	int e=g_track_error;
	float p=TRACK_KP*e;		g_track_integral+=e;
	if(g_track_integral>1000)	g_track_integral=1000;
	if(g_track_integral<-1000)	g_track_integral=-1000;
	float i=TRACK_KI*g_track_integral;	float d=TRACK_KD*(e-g_track_error_last);
	g_track_pid_out=p+i+d;		
	g_track_error_last=e;
}

// ====================== 路径规划（前进和倒退模式） ======================
uint8_t Path_Plan(uint8_t cur,uint8_t tar)
{
	if(g_task_mode==TASK_MODE_1)	
	return DIR_FORWARD;		//
	uint8_t f=(tar-cur+NODE_MAX)%NODE_MAX;
	uint8_t b=(cur-tar+NODE_MAX)%NODE_MAX;
	return(f<=b) ? DIR_FORWARD : DIR_BACKWARD;
}

// ==================== 巡区域边框线-小车中心对齐 ====================
void Car_Align_Center(void)
{
  uint8_t ok=0;uint32_t t=0;
  OpenMV_SetMode(3);
  Arm_MovePoint(POINT_ALIGN);
  while(!ok && t<300)
  {
    if(line_flag)
    {
		line_flag=0;t=0;
		if(fabsf(center_offset)<0.3){ok=1;break;}
		if(center_offset>0.5)
		{
			Motor_Backward(PWM_BACK/6,PWM_BACK/6);
			HAL_Delay(20);
			Motor_Stop();
		}
      	if(center_offset<-0.5)
		{
			Motor_Forward(PWM_BASE/6,PWM_BASE/6);
			HAL_Delay(20);
			Motor_Stop();
		}
    }else t++;
    HAL_Delay(10);
  }
  Arm_MovePoint(POINT_RESET);
  Motor_Stop();
}

// ====================== 视觉模式 ======================
void OpenMV_SetMode(uint8_t m)
{
	char buf[6];
	sprintf(buf,"M,%d\r\n",m);		//主从通信：利用串口打印发送指令“M，x”给从机，从机根据指令切换不同视觉模式
	HAL_UART_Transmit(&huart1,(uint8_t*)buf,strlen(buf),100);
	HAL_Delay(50);
}

// ==================== 机械臂 ====================
void PCA9685_Init(void)
{
	HAL_Delay(100);		//上电等待
	PCA9685_Write(0x00,0x10);
	PCA9685_Write(0xFE,121);
	PCA9685_Write(0x00,0x20);
}
void PCA9685_Write(uint8_t r,uint8_t d)
{
	//向寄存器写入数据函数
	HAL_I2C_Mem_Write(&hi2c1,0x80,r,1,&d,1,100);//参数：I2C句柄，PCA9685舵机驱动模块地址（母），寄存目标器地址（子，具体哪个舵机），寄存器地址长度，数据缓存区地址（来源），数据长度，超时时间
}

void Servo_SetAngle(uint8_t ch,float a)
{
	if(a>180) a=180;
	if(a<0) a=0;	//依旧先限幅，后续可以改成机械臂实际可达范围
	uint16_t p=500+(a/180.0)*2000;  //高电平时间（500μs（0.5ms） 2500μs（2.5ms）进行一个线性映射）
	uint16_t off=(p/20000)*4096;	//PCA9685计数值计算
	uint8_t da[4]={0, 0, off&0xFF, off>>8};		//PWM初始计数值，低位，高位。PWM最终计数值，低八位，高八位
	HAL_I2C_Mem_Write(&hi2c1,0x80,0x06+4*ch,1,da,4,100);	
}

void Arm_MovePoint(Arm_Point_e p)	//p是预设的机械臂位置编号，函数会平滑地将机械臂从当前位置移动到目标位置
{
	float t0=arm_point[p][0], t1=arm_point[p][1], t2=arm_point[p][2];
	static float n0=0,n1=45,n2=-80;		//注意这个地方可调
	for(int i=0;i<=50;i++){				//利用线性插值公式
		float o0=n0+(t0-n0)/50.0*i;		//得到当前转的角度（关节角）
		float o1=n1+(t1-n1)/50.0*i;
		float o2=n2+(t2-n2)/50.0*i;
		Servo_SetAngle(0,o0+90);		//第二个参数考虑实际情况后偏移的实际角
		Servo_SetAngle(1,90-o1);		//偏移参数检验：上电后调用 Arm_MovePoint(POINT_RESET)，看机械臂是不是回到了这个位置
		Servo_SetAngle(2,-o2);
		HAL_Delay(10);		//给运动分成50次移动，10ms延时，500ms平滑完成
  }
  n0=t0; n1=t1; n2=t2;
  HAL_Delay(300);
}

void Arm_Pick_A(void)
{
	Arm_MovePoint(POINT_PICK_A);
	MAGNET_ON();
	HAL_Delay(200);
	Arm_MovePoint(POINT_MOVE);		//到达回收点位
	g_arm_done_flag=1;
}
void Arm_Pick_B(void)
{
	Arm_MovePoint(POINT_PICK_B);
	MAGNET_ON();
	HAL_Delay(200);
	Arm_MovePoint(POINT_MOVE);	
	g_arm_done_flag=1;
}
void Arm_Pick_C(void)
{
	Arm_MovePoint(POINT_PICK_C);
	MAGNET_ON();
	HAL_Delay(200);
	Arm_MovePoint(POINT_MOVE);
	g_arm_done_flag=1;
}

void Arm_Drop_A(void)
{
    Arm_MovePoint(POINT_MOVE);
    MAGNET_ON();
    HAL_Delay(300);

    Arm_MovePoint(POINT_DROP_A);
    HAL_Delay(200);
    MAGNET_OFF();
    HAL_Delay(150);

    for(int k=0;k<2;k++){
        MAGNET_ON(); HAL_Delay(10);
        MAGNET_OFF(); HAL_Delay(30);
    }

    Arm_MovePoint(POINT_RESET);
    g_arm_done_flag = 1;
}
void Arm_Drop_B(void)
{
    Arm_MovePoint(POINT_MOVE);
    MAGNET_ON();
    HAL_Delay(300);

    Arm_MovePoint(POINT_DROP_B);
    HAL_Delay(200);
    MAGNET_OFF();
    HAL_Delay(150);

    for(int k=0;k<2;k++){
        MAGNET_ON(); HAL_Delay(10);
        MAGNET_OFF(); HAL_Delay(30);
    }

    Arm_MovePoint(POINT_RESET);
    g_arm_done_flag = 1;
}
void Arm_Drop_C(void)
{
    Arm_MovePoint(POINT_MOVE);
    MAGNET_ON();
    HAL_Delay(300);

    Arm_MovePoint(POINT_DROP_C);
    HAL_Delay(200);
    MAGNET_OFF();
    HAL_Delay(150);

    for(int k=0;k<2;k++){
        MAGNET_ON(); HAL_Delay(10);
        MAGNET_OFF(); HAL_Delay(30);
    }

    Arm_MovePoint(POINT_RESET);
    g_arm_done_flag = 1;
}


// ====================== 核心状态机 ======================
void Car_State_Machine(void)
{
	g_key_val = Key_Scan();
	uint16_t l, r, dir;

	switch(g_car_state)
		Motor_Stop();
      	OLED_ShowString(0,0,g_task_mode==TASK_MODE_FIXED?"Mode1:Fixed":"Mode2:Custom");
     	OLED_ShowString(0,1,"K1:Start K3:Mode");
		if(g_key_val = 1){
			
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
