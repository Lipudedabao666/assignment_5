/**
  ******************************************************************************
  * @file	 bsp_PWM.c
  * @author  Wang Hongxi
  * @version V1.0.0
  * @date    2020/3/1
  * @brief   一个极简函数：直接往定时器的比较寄存器写值，改变 PWM 占空比
  ******************************************************************************
  * @attention
  *
  * ===================== 教学注释（后来加的，代码没动） =====================
  *
  * 【PWM 是什么？】
  *   PWM 就是"方波"，由定时器硬件自动输出，CPU 只要设好参数就不用管了。
  *   两个关键数字（都在定时器里）：
  *     ARR（自动重装载值）= 一个周期要数多少个数   → 决定【频率】
  *     CCR（比较值）      = 一个周期里高电平数多少 → 决定【占空比】
  *
  *   本工程给 IMU 加热用的 TIM10 是这么配的：
  *     时钟 2MHz，ARR = 2000-1
  *     → 频率 = 2MHz ÷ 2000 = 1kHz
  *     → 占空比 = CCR ÷ 2000    （CCR=200 就是 10%，CCR=2000 就是 100%）
  *
  * 【为什么不用 HAL 的函数？】
  *   HAL 里其实有个宏 __HAL_TIM_SET_COMPARE() 干的是同一件事。
  *   这里手写是为了更直白（少一层包装），本质上就是往 CCRx 寄存器里写一个数。
  *
  * 【谁在用这个函数？】
  *   ins_task.c 里的温度控制：
  *     TIM_Set_PWM(&htim10, TIM_CHANNEL_1, PID算出来的加热功率(0~2000));
  *
  * ⚠️ 前提：这个定时器的 PWM 必须已经被 HAL_TIM_PWM_Start() 启动了，
  *    否则你在这儿写什么数都不会有波形。
  * =========================================================================
  */
#include "bsp_PWM.h"

/**
 * @brief  设置某个定时器、某个通道的 PWM 占空比
 * @param  tim_pwmHandle  定时器句柄（例如 &htim10）
 * @param  Channel        通道号（TIM_CHANNEL_1 ~ TIM_CHANNEL_4）
 * @param  value          要写入的比较值，范围 0 ~ ARR：
 *                          0   → 一直低电平（占空比 0%）
 *                          ARR → 一直高电平（占空比 100%）
 */
void TIM_Set_PWM(TIM_HandleTypeDef *tim_pwmHandle, uint8_t Channel, uint16_t value)
{
    // 保护：万一传进来的值比 ARR 还大（算错了、或者 PID 超调了），
    // 就把它夹到 ARR，避免出现"占空比 150%"这种没有意义的值。
    if (value > tim_pwmHandle->Instance->ARR)
        value = tim_pwmHandle->Instance->ARR;

    // 按通道号，把值写进对应的比较寄存器 CCRx。
    // 写完之后，定时器硬件会在每个周期自己拿 CCR 和计数值比较、自动翻转输出电平，
    // 也就是说：这一句之后 CPU 就不用再管了。
    switch (Channel)
    {
    case TIM_CHANNEL_1:
        tim_pwmHandle->Instance->CCR1 = value;   // 通道 1 → 写 CCR1
        break;
    case TIM_CHANNEL_2:
        tim_pwmHandle->Instance->CCR2 = value;   // 通道 2 → 写 CCR2
        break;
    case TIM_CHANNEL_3:
        tim_pwmHandle->Instance->CCR3 = value;   // 通道 3 → 写 CCR3
        break;
    case TIM_CHANNEL_4:
        tim_pwmHandle->Instance->CCR4 = value;   // 通道 4 → 写 CCR4
        break;
    }
}
