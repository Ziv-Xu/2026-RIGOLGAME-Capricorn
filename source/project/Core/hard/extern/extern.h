#ifndef EXTERN_H
#define EXTERN_H

#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "soft_i2c.h"
#include "oled.h"
#include "soft_i2c_track.h"
#include "track.h"
#include <string.h>
#include "color_uart.h"
#include "mpu_soft_i2c.h"
#include "mpu6050.h"
#include "stdio.h"
#include "motor.h"
#include "encoder.h"
#include "BlueSerial.h"
#include "servo.h"
#include "extern.h"

/*========== 陀螺仪相关变量 =============*/
extern float yaw;
extern float pitch;
extern float roll;

/*========= 循迹相关 ===========*/
extern PID_TypeDef PID;
extern PID_TypeDef PID_MV;
extern int IR_Weight[8];
extern uint8_t BASE_SPEED;
extern float ir;

/*======= 舵机相关 ========*/
/* 脉冲宽度范围（比较值），对应 0° ~ 180° */
extern int SERVO_MIN_PULSE;
extern int SERVO_MAX_PULSE;
extern uint8_t ServoAngle1;
extern uint8_t ServoAngle2;
extern uint8_t ServoAngle3;
extern uint8_t ServoAngle4;
extern float servo_angle[4];
extern float deg_per_sec[4];
/*========== 电机，编码器相关 =============*/
extern SpeedCtrl g_speed_left, g_speed_right;//这两个已经在Motor_Init函数中的PID_Init进行了初始化
extern int16_t g_encoder_distance; //后续直接赋值，不用特意初始化
extern uint8_t g_car_dir;
extern int16_t start_distance;//全局变量，用于记录每一次定距离行驶时的开始数值
/*========== OpenMV相关 ==========*/
extern ColorFrame_t g_color_frame;


extern int temp_count;



#endif /* EXTERN_H */
