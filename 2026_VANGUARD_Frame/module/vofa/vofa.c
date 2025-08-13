/*
 * @file		vofa.c/h
 * @brief	    vofa+发送数据（调参）
 * @history
 * 	版本			作者			编写日期			内容
 * 	v1.0		冯俊玮		2024/9/10		向上位机vofa+发送数据
 *  v1.1		GUATAI	    2025/8/12		邪恶bsp归一化
 */

#include <stdlib.h>
#include <string.h>
#include "vofa.h"

#define MAX_BUFFER_SIZE 128

uint8_t send_buf[MAX_BUFFER_SIZE];
uint16_t cnt = 0;

USART_t *vofa_usart_instance;

float data_view[20]={0};

void VOFA_Init(UART_HandleTypeDef *vofa_usart_handle)
{
	usart_init_config_t config;
	config.module_callback = NULL; // 该模块不需要接收数据
  config.usart_handle = vofa_usart_handle;
  config.recv_buff_size = 1;

  vofa_usart_instance = USART_Register(&config);
}

/**
***********************************************************************
* @brief:      VOFA_Transmit(uint8_t* buf, uint16_t len)
* @param:		void
* @retval:     void
* @details:    修改通信工具，USART或者USB
***********************************************************************
**/
void VOFA_Transmit(uint8_t* buf, uint16_t len)
{
    USART_Send(vofa_usart_instance, buf, len, USART_TRANSFER_BLOCKING);
	// HAL_UART_Transmit(&huart7, buf, len, 0xff);
}

/**
 * @brief vofa发送不动长数据
 * @param buf 数组指针
 * @param len 数组长度
 */

uint8_t buf_end[4] = {0x00, 0x00, 0x80, 0x7f};
void VOFA_Send_Data(float *buf, uint8_t len)
{
	cnt = len * 4 ;

	memcpy(send_buf , buf, cnt);
	memcpy(send_buf + cnt, buf_end, 4);
	VOFA_Transmit((uint8_t *)send_buf, cnt +4);
}







