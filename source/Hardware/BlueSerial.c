/*
 * BlueTooth.c
 *
 *  Created on: Mar 22, 2026
 *      Author: L
 */
#include "main.h"
#include "stm32f1xx_hal_uart.h"
#include "stm32f1xx_it.h"
#include "usart.h"
#include <stdio.h>
#include <stdarg.h>

// 发送环形缓冲区

#define TX_BUFFER_SIZE     256          // 根据需求调整（2的幂方便取模）
static uint8_t tx_buffer[TX_BUFFER_SIZE];
static volatile uint16_t tx_put = 0;    // 写指针
static volatile uint16_t tx_get = 0;    // 读指针
static volatile uint8_t tx_busy = 0;    // 0：空闲，1：正在发送中
static uint8_t tx_current_byte;

// 定义一个单字节接收缓冲区（用于每次接收一个字节）
static uint8_t rx_temp_byte = 0;
#define BLUE_SERIAL_BUFFER_SIZE			100

uint8_t BlueSerial_Buffer[BLUE_SERIAL_BUFFER_SIZE];
uint16_t BlueSerial_PutIndex = 0;
uint16_t BlueSerial_GetIndex = 0;

char BlueSerial_String[100];
char BlueSerial_StringArray[6][20];

void BlueSerial_IRQHandler(uint8_t RxData);
/*
* @brief 蓝牙串口初始化，须配合cubeMX的初始化
*/
void BlueSerial_Init()
{
    HAL_UART_Receive_IT(&huart2, &rx_temp_byte, 1);
}


/**
 * @brief 将单字节放入发送环形缓冲区，如果当前没有发送任务则启动发送
 * @param byte 要发送的数据
 * @retval 0：成功放入缓冲区；1：缓冲区满
 */
uint8_t BlueSerial_SendByte(uint8_t byte)
{
    uint16_t next_put = (tx_put + 1) % TX_BUFFER_SIZE;

    // 环形缓冲区满
    if (next_put == tx_get) {
        return 1;   // 发送失败，缓冲区满
    }

    // 防止中断干扰指针操作（若在中断可能访问，需关中断）
    __disable_irq();
    tx_buffer[tx_put] = byte;
    tx_put = next_put;
    __enable_irq();

    // 如果当前没有正在进行的发送，则启动第一次发送
    if (tx_busy == 0) {
        tx_busy = 1;
        // 从缓冲区取出一个字节并启动中断发送
        __disable_irq();
        tx_current_byte = tx_buffer[tx_get];
        tx_get = (tx_get + 1) % TX_BUFFER_SIZE;
        __enable_irq();
        HAL_UART_Transmit_IT(&huart2, &tx_current_byte, 1);
    }

    return 0;   // 成功
}


uint8_t BlueSerial_Put(uint8_t Byte)
{
    if ((BlueSerial_PutIndex + 1) % BLUE_SERIAL_BUFFER_SIZE == BlueSerial_GetIndex)
    {
        return 1;
    }
	BlueSerial_Buffer[BlueSerial_PutIndex] = Byte;
	if (BlueSerial_PutIndex >= BLUE_SERIAL_BUFFER_SIZE - 1)
	{
		BlueSerial_PutIndex = 0;
	}
	else
	{
		BlueSerial_PutIndex ++;
	}
	return 0;
}

uint8_t BlueSerial_Get(uint8_t *Byte)
{
	if (BlueSerial_GetIndex == BlueSerial_PutIndex)
	{
		*Byte = 0;
	    return 1;
	}
	*Byte = BlueSerial_Buffer[BlueSerial_GetIndex];
//	BlueSerial_Buffer[BlueSerial_GetIndex] = 0x00;
	if (BlueSerial_GetIndex >= BLUE_SERIAL_BUFFER_SIZE - 1)
	{
		BlueSerial_GetIndex = 0;
	}
	else
	{
		BlueSerial_GetIndex ++;
	}
	return 0;
}

uint16_t BlueSerial_Length(void)
{
	return (BlueSerial_PutIndex + BLUE_SERIAL_BUFFER_SIZE - BlueSerial_GetIndex) % BLUE_SERIAL_BUFFER_SIZE;;
}

uint8_t BlueSerial_Read(uint16_t Index)
{
	return BlueSerial_Buffer[(BlueSerial_GetIndex + Index) % BLUE_SERIAL_BUFFER_SIZE];
}

void BlueSerial_ClearBuffer(void)
{
	uint8_t i;
	for (i = 0; i < BLUE_SERIAL_BUFFER_SIZE; i ++)
	{
		BlueSerial_Buffer[i] = 0;
	}
}
/*
* @brief 发送一个数组
* @param Array 数组，Length 数组长度
*/
void BlueSerial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		BlueSerial_SendByte(Array[i]);
	}
}
/*
* @brief 发送一个字符串
* @param String 字符串
*/
void BlueSerial_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		BlueSerial_SendByte(String[i]);
	}
}
/*
*@brief 蓝牙串口printf函数
*/
void BlueSerial_Printf(char *format, ...)
{
	char String[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	BlueSerial_SendString(String);
}
/*
* @brief 串口接收标志位函数
*/
uint8_t BlueSerial_ReceiveFlag(void)
{
	uint8_t Flag = 0;
	uint16_t Length = BlueSerial_Length();
	for (uint16_t i = 0; i < Length; i ++)
	{
		if (BlueSerial_Read(i) == '[')
		{
			Flag = 1;
		}
		else if (BlueSerial_Read(i) == ']' && Flag == 1)
		{
			return 1;
		}
	}
	return 0;
}
/*
* @brief 串口数据接收函数
* @warning 接收的数据只能是“[“+”数据“+”]”的形式，不带“[”“]”的数据无法接收。
*/
void BlueSerial_Receive(void)
{
	uint16_t p = 0, k = 0;
	uint8_t Flag = 0;
	uint16_t Length = BlueSerial_Length();
	for (uint16_t i = 0; i < Length; i ++)
	{
		uint8_t Byte;
		BlueSerial_Get(&Byte);
		if (Byte == '[')
		{
			Flag = 1;
			p = 0;
		}
		else if (Byte == ']' && Flag == 1)
		{
			BlueSerial_String[p] = '\0';
			break;
		}
		else if (Flag == 1)
		{
			BlueSerial_String[p] = Byte;
			p ++;
		}
	}

	p = 0;
	for (uint16_t i = 0; BlueSerial_String[i] != '\0'; i ++)
	{
		if (BlueSerial_String[i] == ',')
		{
			BlueSerial_StringArray[p][k] = '\0';
			p ++;
			k = 0;
		}
		else
		{
			BlueSerial_StringArray[p][k] = BlueSerial_String[i];
			k ++;
		}
	}
	BlueSerial_StringArray[p][k] = '\0';
}

//void BlueSerial_IRQHandler(uint8_t RxData)
//{
//	BlueSerial_Put(RxData);

//}已通过串口接收回调实现


/**
 * @brief 串口发送完成中断回调（HAL 库自动调用）
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)   // 确认是蓝牙串口
    {
        // 检查环形缓冲区中是否还有待发送的数据
        if (tx_put != tx_get)
        {
            // 取出下一个字节并发送
			__disable_irq();
            tx_current_byte = tx_buffer[tx_get];
            tx_get = (tx_get + 1) % TX_BUFFER_SIZE;
			__enable_irq();
            HAL_UART_Transmit_IT(&huart2, &tx_current_byte, 1);
        }
        else
        {
            // 缓冲区已空，清除发送忙标志
            tx_busy = 0;
        }
    }
}


/**
 * @brief 串口接收完成回调（HAL 库自动调用）
 * @param huart 串口句柄
 */
 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        // 将收到的字节存入接收环形缓冲区
        BlueSerial_Put(rx_temp_byte);

        // 重新开启下一次单字节接收
        HAL_UART_Receive_IT(&huart2, &rx_temp_byte, 1);
    }
}