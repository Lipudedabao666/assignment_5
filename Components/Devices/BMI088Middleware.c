/**
 * @file    BMI088Middleware.c
 * @brief   BMI088 的"中间层"：只负责做两件最底层的事 —— 拉片选、SPI 收发一个字节
 *
 * ===================== 教学注释（后来加的，代码没动） =====================
 *
 * 【这个文件在整条链路里的位置】
 *
 *   SPI 硬件
 *     │  ← 本文件（最底层：一个字节一个字节地收/发）
 *     ▼
 *   BMI088driver.c（上一层：告诉它"读哪个寄存器、读几个字节"，再把原始字节拼成物理量）
 *     │
 *     ▼
 *   ins_task.c（再上一层：每 1ms 调用一次，把数据送进卡尔曼滤波）
 *
 * 【为什么要单独分一层？】
 *   因为"怎么收发字节"和"读哪个寄存器"是两件不同的事。
 *   分开了以后，如果以后把传感器改接到 I2C 上，只要重写这一层，上层驱动完全不用动。
 *
 * 【SPI 通信的最小知识（读懂本文件只需要这些）】
 *   · SPI 是"主从"结构，MCU 是主，芯片是从。MCU 给时钟(SCK)，数据才走。
 *   · 片选(CS/NS)就是"点名"：拉低 = "我在跟你说话"，拉高 = "说完了/不理你"。
 *     BMI088 内部其实有两个独立的芯片：加速度计 和 陀螺仪，各自一个片选脚，
 *     所以下面有 4 个函数（2 个器件 × 拉低/拉高）。
 *   · SPI 是"全双工"的：你每发出去一个字节，同时就会收回来一个字节 ——
 *     这就是为什么下面只有一个 BMI088_read_write_byte()，读和写共用它。
 *     想读寄存器：先发"寄存器地址|0x80"，再发一个 0x55（占位，为了产生时钟），
 *                 收回来的那个字节就是寄存器内容。
 *     想写寄存器：先发地址，再发要写的值。
 *     （这两个动作在 BMI088driver.c 里封装好了）
 * =========================================================================
 */
#include "BMI088Middleware.h"
#include "main.h"

// SPI 句柄的指针。初始化时由 BMI088driver.c 传进来（本工程就是 &hspi1）。
SPI_HandleTypeDef *BMI088_SPI;

/**
 * @brief 把加速度计的片选拉低 —— 表示"接下来要跟加速度计说话"
 *        对应的引脚宏 CS1_ACCEL_Pin / CS1_ACCEL_GPIO_Port 是 CubeMX 生成的（PA4）
 */
void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);   // RESET = 低电平
}
/**
 * @brief 把加速度计的片选拉高 —— 表示"跟加速度计说完了"（释放总线）
 */
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);     // SET = 高电平
}

/**
 * @brief 把陀螺仪的片选拉低 —— 表示"接下来要跟陀螺仪说话"（对应引脚 PB0）
 */
void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}
/**
 * @brief 把陀螺仪的片选拉高 —— 表示"跟陀螺仪说完了"
 */
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}

/**
 * @brief SPI 收发一个字节（读和写都靠它）
 * @param txdata  要发出去的字节
 * @return        同一时刻收回来的字节
 *
 * 说明：HAL_SPI_TransmitReceive 是"发一个、同时收一个"的阻塞式函数。
 *   最后一个参数 1000 是超时时间（毫秒）—— 万一芯片没接好，最多等 1 秒就返回，
 *   不会把程序永远卡死。
 */
uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(BMI088_SPI, &txdata, &rx_data, 1, 1000);
    return rx_data;
}
