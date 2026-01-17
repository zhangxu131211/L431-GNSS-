/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART1_UART_Init(uint32_t bound);
void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */
// 缓冲区大小宏定义（按需修改）
#define USART1_MAX_SEND_LEN    400  // 最大发送字节数
#define USART1_MAX_RECV_LEN    400  // 最大接收字节数
//#define USART2_RX_TIMEOUT_MS   10    // 接收超时阈值（10ms）

// 串口发送缓冲区（8字节对齐，HAL库兼容）
extern uint8_t USART1_TX_BUF[USART1_MAX_SEND_LEN];  // HAL库推荐用__ALIGN_BEGIN/END
// 串口接收缓冲区
extern uint8_t USART1_RX_BUF[USART1_MAX_RECV_LEN];
extern volatile uint16_t USART1_RX_STA;    


/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

