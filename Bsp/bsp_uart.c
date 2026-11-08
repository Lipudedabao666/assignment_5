//
// Created by Legion on 2026/10/14.
//
#include "bsp_uart.h"
#include <string.h>

#define UART_RECEIVE_SIZE   18

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef  hdma_usart3_rx;

static uint8_t uart_receive_buffer[UART_RECEIVE_SIZE];
static uint8_t uart_frame_buffer[UART_RECEIVE_SIZE];
static uint8_t uart_frame_ready;

//启动 USART3 的 DMA + 空闲中断接收，先清掉残留的错误标志再开
void BSP_UART_StartReceive(void)
{
    HAL_UART_AbortReceive(&huart3);

    __HAL_UART_CLEAR_OREFLAG(&huart3);
    __HAL_UART_CLEAR_PEFLAG(&huart3);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart_receive_buffer, UART_RECEIVE_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
}

//取一帧完整数据，取到返回 1 并清标志，没取到返回 0
uint8_t BSP_UART_GetFrame(uint8_t *out)
{
    if (uart_frame_ready == 0)
    {
        return 0;
    }

    memcpy(out, uart_frame_buffer, UART_RECEIVE_SIZE);
    uart_frame_ready = 0;
    return 1;
}

//接收事件回调：空闲中断路线，长度对就拷出来并置标志，然后重启接收
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART3 && Size == UART_RECEIVE_SIZE)
    {
        memcpy(uart_frame_buffer, uart_receive_buffer, UART_RECEIVE_SIZE);
        uart_frame_ready = 1;
    }

    BSP_UART_StartReceive();
}

//串口出错回调：HAL 出错时会关掉接收，这里重启一次
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        BSP_UART_StartReceive();
    }
}
