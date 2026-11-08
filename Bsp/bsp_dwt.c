/**
 ******************************************************************************
 * @file	bsp_dwt.c
 * @author  Wang Hongxi
 * @version V1.1.0
 * @date    2022/3/8
 * @brief   用内核自带的 DWT 计数器做一个"高精度秒表"
 ******************************************************************************
 * @attention
 *
 * ===================== 教学注释（后来加的，代码没动） =====================
 *
 * 【这个文件在整个陀螺仪算法里是什么角色？】
 *   它是"计时员"。卡尔曼滤波每一步都要知道"距上次运算过去了多久"（也就是 dt）。
 *   如果偷懒写死成 0.001f（假设每 1ms 一定跑一次），一旦循环偶尔慢了或快了，
 *   滤波就相当于"用错了模型"，姿态会抖、会慢慢漂。
 *   所以这里用内核自带的计数器，每一轮都【实测】间隔。
 *
 * 【DWT 是什么？为什么不用 HAL_GetTick()？】
 *   DWT = Data Watchpoint and Trace，是 Cortex-M 内核里的一个调试单元。
 *   它里面有个寄存器叫 CYCCNT，每来一个 CPU 时钟周期就自动 +1。
 *   我们主频 168MHz → 分辨率约 6 纳秒；
 *   而 HAL_GetTick() 靠 SysTick 中断累加，只能精确到 1 毫秒，对 EKF 来说太粗了。
 *
 * 【这个文件里的函数都在哪儿被调用？】
 *   DWT_Init(168)          ← main() 里，排在其它初始化之前
 *   DWT_GetDeltaT(&cnt)    ← ins_task.c 的 INS_Task() 里，每 1ms 一次（最关键的一个）
 *   DWT_GetTimeline_s()    ← BMI088driver.c 的静止标定里（用来判断有没有超时）
 *   DWT_Delay()            ← 标定过程中的短延时
 *
 * =========================================================================
 */
#include "bsp_dwt.h"

/* ---------------------------- 全局 / 静态变量 ---------------------------- */

DWT_Time_t SysTime;   // 存放"开机到现在"的 秒/毫秒/微秒（给下面 DWT_GetTimeline_* 用）

// 下面这一行的三个变量分别是：CPU 主频(Hz)、每毫秒的周期数、每微秒的周期数
// （在 DWT_Init 里赋值，本工程 = 168000000 / 168000 / 168）
static uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;
static uint32_t CYCCNT_RountCount; // CYCCNT 一共"绕圈"了几次（32位计数器约 25.6 秒绕一圈）
static uint32_t CYCCNT_LAST;       // 上一次读到的计数值（用来判断有没有绕圈）
uint64_t CYCCNT64;                 // 圈数 + 当前值拼起来的 64 位总计数

static void DWT_CNT_Update(void);  // 内部小函数，定义在文件末尾

/**
 * @brief 初始化这个"秒表"
 * @param CPU_Freq_mHz  CPU 主频，单位 MHz。本工程 168MHz 就传 168
 *
 * 整个函数就四件事：打开总开关 → 清零 → 开始数数 → 把主频记下来备用。
 */
void DWT_Init(uint32_t CPU_Freq_mHz)
{
    /* 使能DWT外设 */
    // DEMCR 寄存器里的 TRCENA 位 = 整个"调试跟踪单元"的总电源。
    // 不先打开它，下面写 DWT 的寄存器全部无效（很多人第一次用 DWT 失败就是漏了这句）。
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* DWT CYCCNT寄存器计数清0 */
    // 把手表归零，从这一刻开始计时。
    DWT->CYCCNT = (uint32_t)0u;

    /* 使能Cortex-M DWT CYCCNT寄存器 */
    // CTRL 里的 CYCCNTENA 位 = "开始数数"。置 1 之后，每个 CPU 周期 CYCCNT 自动 +1。
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // 把主频换算成各种单位存起来。后面每次算时间都要用，存一次省得反复乘。
    CPU_FREQ_Hz = CPU_Freq_mHz * 1000000;      // 168 → 168000000
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;       // 168000（每毫秒多少个周期）
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;    // 168（每微秒多少个周期）

    CYCCNT_RountCount = 0;                     // 绕圈计数清零
}

/**
 * @brief  ★核心★ 量一下"距离上次调用过了多久"
 * @param cnt_last  上一次的计数值。第一次调用前随便给个 0 就行，函数会自己更新它
 * @retval 时间间隔，单位【秒】
 *
 * 为什么它这么重要？
 *   卡尔曼滤波每一步都需要知道"这两次采样之间隔了多久"。它不假设是 1ms，
 *   而是每一轮真去量。所以哪怕你的循环偶尔卡了 0.3ms，滤波依然算得对。
 */
float DWT_GetDeltaT(uint32_t *cnt_last)
{
    // volatile：告诉编译器"这个值随时可能自己变"，每次都老老实实去读寄存器，不许优化掉。
    volatile uint32_t cnt_now = DWT->CYCCNT;   // 读"现在"的计数

    // (cnt_now - *cnt_last) = 这段时间经过了多少个 CPU 周期
    // 再除以 CPU_FREQ_Hz 就换算成"秒"。
    // 注意先强转成 uint32_t 再相减：计数器是 32 位，约 25 秒就会绕回 0，
    // 用"无符号数相减"自带的溢出特性，绕圈时算出来的差值依然正确（小技巧）。
    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(CPU_FREQ_Hz));

    *cnt_last = cnt_now;   // 记住这一次的值，作为下一次的起点

    DWT_CNT_Update();      // 顺便看看有没有绕圈（给 DWT_GetTimeline_* 用）

    return dt;             // 出去：本次间隔（秒），正常应该在 0.001 附近
}

/**
 * @brief 和 DWT_GetDeltaT 一模一样，只是用 double（精度更高，适合长时间积分）
 *        本工程主路径没用它，知道有这个东西就行。
 */
double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}

/**
 * @brief 把"32位计数器 + 绕圈次数"拼成 64 位总计数，再拆成 秒/毫秒/微秒 存进 SysTime
 *        （下面的 DWT_GetTimeline_* 三个函数都靠它）
 */
void DWT_SysTimeUpdate(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    static uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3;

    DWT_CNT_Update();   // 先更新绕圈次数

    // 总周期数 = 绕了几圈 × 一圈的大小 + 当前值
    // ⚠️ 严格说一圈应该是 (UINT32_MAX + 1)，这里写的是 UINT32_MAX，
    //    所以每绕一圈会少算 1 个周期（约 6 纳秒）。对机器人来说完全无所谓，不用改。
    CYCCNT64 = (uint64_t)CYCCNT_RountCount * (uint64_t)UINT32_MAX + (uint64_t)cnt_now;

    CNT_TEMP1 = CYCCNT64 / CPU_FREQ_Hz;                        // 算出整秒数
    CNT_TEMP2 = CYCCNT64 - CNT_TEMP1 * CPU_FREQ_Hz;            // 去掉整秒后剩下的周期数
    SysTime.s = CNT_TEMP1;                                     // 存秒
    SysTime.ms = CNT_TEMP2 / CPU_FREQ_Hz_ms;                   // 剩下部分里的整毫秒数
    CNT_TEMP3 = CNT_TEMP2 - SysTime.ms * CPU_FREQ_Hz_ms;       // 再去掉毫秒，剩下的周期数
    SysTime.us = CNT_TEMP3 / CPU_FREQ_Hz_us;                   // 换算成微秒
}

/**
 * @brief 返回"开机到现在过了多少秒"（带小数）
 *        典型用途：判断某个动作有没有超时（标定里就是这么用的）
 */
float DWT_GetTimeline_s(void)
{
    DWT_SysTimeUpdate();   // 先刷新 SysTime 里的 秒/毫秒/微秒

    // 拼成一个小数：秒 + 毫秒×0.001 + 微秒×0.000001
    float DWT_Timelinef32 = SysTime.s + SysTime.ms * 0.001f + SysTime.us * 0.000001f;

    return DWT_Timelinef32;
}

/**
 * @brief 同上，但返回的单位是【毫秒】
 */
float DWT_GetTimeline_ms(void)
{
    DWT_SysTimeUpdate();

    // 秒×1000 + 毫秒 + 微秒×0.001
    float DWT_Timelinef32 = SysTime.s * 1000 + SysTime.ms + SysTime.us * 0.001f;

    return DWT_Timelinef32;
}

/**
 * @brief 同上，但返回的单位是【微秒】，而且用整数返回
 */
uint64_t DWT_GetTimeline_us(void)
{
    DWT_SysTimeUpdate();

    uint64_t DWT_Timelinef32 = SysTime.s * 1000000 + SysTime.ms * 1000 + SysTime.us;

    return DWT_Timelinef32;
}

/**
 * @brief 检查计数器有没有"绕圈"
 *        32 位计数器数到最大后会绕回 0（168MHz 下约 25.6 秒一次）。
 *        办法很简单：这次读到的值如果比上次还小，说明它绕回去了 → 圈数 +1。
 */
static void DWT_CNT_Update(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;

    if (cnt_now < CYCCNT_LAST)   // 变小了 = 绕圈了
        CYCCNT_RountCount++;

    CYCCNT_LAST = cnt_now;       // 更新"上次的值"
}

/**
 * @brief 忙等延时
 * @param Delay 要等多少【秒】（例如 0.0005 就是 0.5 毫秒）
 *
 * ⚠️ 这是"忙等"：CPU 会卡在这个空循环里，别的什么都干不了。
 *    所以只能用在很短的延时（标定里用 0.5ms / 5ms）。长时间延时千万别用它。
 *    好处是它不依赖任何中断，比 HAL_Delay 更可靠（HAL_Delay 依赖 SysTick 中断）。
 */
void DWT_Delay(float Delay)
{
    uint32_t tickstart = DWT->CYCCNT;   // 记下开始时刻
    float wait = Delay;

    // 死循环，直到"已经流逝的周期数" ≥ "要等的秒数 × 每秒周期数"
    while ((DWT->CYCCNT - tickstart) < wait * (float)CPU_FREQ_Hz)
    {
    }
}
