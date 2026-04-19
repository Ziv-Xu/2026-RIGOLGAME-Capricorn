#ifndef __OLED_H
#define __OLED_H

#include "stm32f1xx_hal.h"
#define OLED_I2C_HANDLE &hi2c1
#define OLED_I2C_ADDR 0x78  // 0.96寸I2C OLED默认地址

// 留待扩展的函数声明
void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowString(uint8_t x, uint8_t y, char *str);
void OLED_ShowNum(uint8_t x, uint8_t y, int32_t num, uint8_t len);


#endif