//
// Created by Legion on 2026/10/14.
//

#ifndef TRIPOD_HEAD_BSP_UART_H
#define TRIPOD_HEAD_BSP_UART_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

//启动 USART3 的 DMA + 空闲中断接收
void BSP_UART_StartReceive(void);

//取一帧完整数据，取到返回 1 并清标志，没取到返回 0
uint8_t BSP_UART_GetFrame(uint8_t *out);

#ifdef __cplusplus
}
#endif

#endif //TRIPOD_HEAD_BSP_UART_H
