/**
 ******************************************************************************
 * @file    ins_task.c
 * @author  Wang Hongxi
 * @version V2.0.0
 * @date    2022/2/23
 * @brief   姿态解算的"总调度"：每 1ms 被调用一次，把传感器、滤波、温控串起来
 ******************************************************************************
 * @attention
 *
 * ===================== 教学注释（后来加的，代码没动） =====================
 *
 * 【这个文件是干什么的？—— 一句话】
 *   它是"工头"：每 1 毫秒被叫醒一次，指挥别人干活（读传感器 → 算姿态 → 控温），
 *   并把算好的姿态放进全局变量 INS 里，供全车使用。
 *
 * 【整个陀螺仪算法的数据流】
 *
 *   ① SPI 硬件  ──► BMI088driver.c 的 BMI088_Read()
 *                      把陀螺仪/加速度计的原始数字变成物理量：
 *                      BMI088.Gyro[3] (rad/s)、BMI088.Accel[3] (m/s²)、BMI088.Temperature (℃)
 *                              │
 *   ②                         ▼
 *                      INS_Task()  【本文件】  每 1ms 一次
 *                              │
 *                              ├─ 量 dt（用 bsp_dwt.c 的秒表）
 *                              ├─ 把传感器数据搬到 INS 结构体
 *                              ├─ IMU_QuaternionEKF_Update()  ← ★核心★（在 QuaternionEKF.c）
 *                              │      输入：角速度、加速度、dt
 *                              │      输出：四元数 + 零偏估计（放在全局 QEKF_INS 里）
 *                              ├─ 四元数 → Yaw / Pitch / Roll（度）
 *                              ├─ 算"去掉重力后的运动加速度"
 *                              └─ 每 2ms 调一次温度控制（加热 IMU）
 *                              │
 *   ③                         ▼
 *                      全局变量 INS   ← 全车都读它（云台 yaw 闭环、小陀螺…）
 *
 * 【你要记住的三件事】
 *   1. 外面只需要每 1ms 调一次 INS_Task()，别的什么都不用管。
 *   2. 姿态数据从 INS.Yaw / INS.Pitch / INS.Roll 读（单位：度）；
 *      角速度从 INS.Gyro[] 读（单位：rad/s ← 注意不是度！）。
 *   3. Yaw 没有绝对参考（没用磁力计），上电时是 0，之后会缓慢漂；Pitch/Roll 靠重力修正，不漂。
 *
 * =========================================================================
 */
#include "ins_task.h"
#include "controller.h"
#include "QuaternionEKF.h"
#include "bsp_PWM.h"

/* ------------------------ 全局变量（外面就靠读这些） ------------------------ */

INS_t INS;                 // ★"对外输出的数据仓库"：姿态、角速度、加速度都放这里面
IMU_Param_t IMU_Param;     // 用于修正"IMU 装歪了"的参数（本 demo 没用，见文件末尾的说明）
PID_t TempCtrl = {0};      // 温度控制用的 PID 结构体（就是加热那一路）

// 机体坐标系的三个单位向量（用它们可以观察"车的三个轴在世界里指向哪"）
const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

uint32_t INS_DWT_Count = 0; // 秒表用：记上一次的周期计数值（传给 DWT_GetDeltaT 的）
static float dt = 0, t = 0; // dt = 本次循环的真实间隔（秒）；t = 累计运行时间
uint8_t ins_debug_mode = 0; // 调试开关，本 demo 没用
float RefTemp = 40;         // ★IMU 加热的目标温度：40℃（陀螺零偏随温度变，所以要恒温）

static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3]);

/**
 * @brief 姿态模块初始化 —— 开机调一次
 *
 * 干三件事：
 *   ① 把 IMU 修正参数设成"什么都不改"（scale=1、角度=0）
 *   ② 建立四元数卡尔曼滤波器
 *   ③ 建立温度 PID，并把加热 PWM(TIM10) 启动起来
 */
void INS_Init(void)
{
    // ---- ① 安装误差修正参数：先全设成"不修正" ----
    IMU_Param.scale[X] = 1;    // 三个轴的标度系数 = 1（就是不缩放）
    IMU_Param.scale[Y] = 1;
    IMU_Param.scale[Z] = 1;
    IMU_Param.Yaw = 0;         // 三个安装偏角 = 0（就是不旋转）
    IMU_Param.Pitch = 0;
    IMU_Param.Roll = 0;
    IMU_Param.flag = 1;        // 这个标志=1 表示"参数变了，下次调用时请重算旋转矩阵"

    // ---- ② 建立卡尔曼滤波器（参数含义见下面那行注释）----
    IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0);
    //                      │    │       │      │  └─ lpf：加速度低通滤波系数，0 = 不用
    //                      │    │       │      └──── lambda：渐消因子，1 = 不渐消（原版注释写 0.9996）
    //                      │    │       └─────────── R：加速度计的"量测噪声"，给得越大=越不信任加速度计
    //                      │    └─────────────────── Q2：陀螺零偏的过程噪声（越小=零偏越稳）
    //                      └──────────────────────── Q1：四元数的过程噪声（越大=越信陀螺）

    // imu heat init
    // ---- ③ 温度控制初始化 ----
    // ⚠️ PID_Init 的参数顺序很反直觉：上限、积分限幅、死区 在前，Kp/Ki/Kd 在后
    PID_Init(&TempCtrl, 2000, 300, 0, 1000, 20, 0, 0, 0, 0, 0, 0, 0);
    //                 │     │    │   │    │  └─ Kd = 0（温控一般不用微分）
    //                 │     │    │   │    └──── Ki = 20
    //                 │     │    │   └───────── Kp = 1000
    //                 │     │    └───────────── deadband 死区 = 0
    //                 │     └────────────────── integral_limit 积分限幅 = 300
    //                 └──────────────────────── max_out 输出上限 = 2000（正好等于 TIM10 的 ARR）
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);   // ★启动加热 PWM（不启动的话写 CCR 也没波形）

    INS.AccelLPF = 0.0085;     // 算"运动加速度"时用的低通滤波系数（越小越平滑、越迟钝）
}

/**
 * @brief ★★★ 姿态更新 —— 每 1 毫秒调一次（本项目放在 TIM6 中断里）
 *
 * 注意开头的 static count：这个函数靠它来数"第几次调用"，
 * 从而实现"每 1 次读传感器、每 2 次做一次温控"这种分频。
 */
void INS_Task(void)
{
    static uint32_t count = 0;                       // 调用次数计数器（用来做分频）
    const float gravity[3] = {0, 0, 9.81f};          // 世界坐标系里的重力向量：Z 轴向下 9.81
    dt = DWT_GetDeltaT(&INS_DWT_Count);              // ★量出"距上次调用过了多久"（秒）
    t += dt;                                         // 累计运行时间

    // ins update
    if ((count % 1) == 0)   // % 1 = 0 恒成立 → 也就是"每次调用都干"。想降频可以把 1 改成 2、5…
    {
        // ---- 第1步：读传感器（SPI 读一次，拿到原始物理量）----
        BMI088_Read(&BMI088);

        // ---- 第2步：把传感器数据搬进 INS 结构体（方便全车统一从 INS 取数据）----
        INS.Accel[X] = BMI088.Accel[X];
        INS.Accel[Y] = BMI088.Accel[Y];
        INS.Accel[Z] = BMI088.Accel[Z];
        INS.Gyro[X] = BMI088.Gyro[X];       // 单位 rad/s
        INS.Gyro[Y] = BMI088.Gyro[Y];
        INS.Gyro[Z] = BMI088.Gyro[Z];

        // demo function,用于修正安装误差,可以不管,本demo暂时没用
        // （下面这个函数本 demo 是"空操作"：参数都是默认值，等于什么都没做。详见文件末尾的说明）
        IMU_Param_Correction(&IMU_Param, INS.Gyro, INS.Accel);

        // 计算重力加速度矢量和b系的XY两轴的夹角,可用作功能扩展,本demo暂时没用
        // atan2f 是"反正切"，能算出重力方向和 X、Z 轴的夹角 → 也就是车歪了多少度。
        // 乘以 180/PI 是把弧度换成角度。这里算出来只是放着，没参与控制。
        INS.atanxz = -atan2f(INS.Accel[X], INS.Accel[Z]) * 180 / PI;
        INS.atanyz = atan2f(INS.Accel[Y], INS.Accel[Z]) * 180 / PI;

        // 核心函数,EKF更新四元数
        // ★★★ 整个算法的心脏：把陀螺仪和加速度计融合，得到"车现在的姿态"
        // 参数依次是：角速度(rad/s)×3、加速度(m/s²)×3、时间间隔(秒)
        // 它没有返回值 —— 结果放在全局变量 QEKF_INS 里（调试时直接看这个变量就行）
        IMU_QuaternionEKF_Update(INS.Gyro[X], INS.Gyro[Y], INS.Gyro[Z], INS.Accel[X], INS.Accel[Y], INS.Accel[Z], dt);

        // 把 EKF 算出来的四元数拷到 INS 里（q[0]=实部，q[1..3]=虚部）
        memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

        // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
        // 白话：把"车的 X 轴"这个箭头，换算成"在世界里指向哪个方向"，存进 INS.xn。
        //       yn、zn 同理。这三个向量完整描述了车的朝向（调试时看它们很直观）。
        BodyFrameToEarthFrame(xb, INS.xn, INS.q);
        BodyFrameToEarthFrame(yb, INS.yn, INS.q);
        BodyFrameToEarthFrame(zb, INS.zn, INS.q);

        // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
        // 白话：加速度计测到的是"重力 + 车自身运动"，想得到"车自己的加速度"，
        //      就得先把重力分量减掉。这一步先把世界系的重力转到车体系，才能对应着减。
        float gravity_b[3];
        EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
        for (uint8_t i = 0; i < 3; i++) // 同样过一个低通滤波
        {
            // 一阶低通滤波（让数据更平滑，抑制撞击造成的尖峰）：
            // 新值 = 本次测量值×系数 + 上次输出×系数，两个系数加起来等于 1
            INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
        }
        BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n
        // 到这里 INS.MotionAccel_n 就是"世界坐标系下、车自己的加速度"（本 demo 只是算出来放着，
        // 你可以用它做撞击检测、功率控制里的加速度前馈等）

        // 获取最终数据
        // ★★★ 全车要用的姿态角，就在这里诞生 ★★★
        INS.Yaw = QEKF_INS.Yaw;                       // 偏航（度，±180 会回绕）
        INS.Pitch = QEKF_INS.Pitch;                   // 俯仰（度）
        INS.Roll = QEKF_INS.Roll;                     // 横滚（度）
        INS.YawTotalAngle = QEKF_INS.YawTotalAngle;   // 累计偏航（可超过 360°，小陀螺用这个）
    }

    // temperature control
    if ((count % 2) == 0)   // 每 2 次调用做一次 → 1kHz÷2 = 500Hz
    {
        // 500hz
        IMU_Temperature_Ctrl();   // 读一次温度、算一次 PID、更新一次加热功率
    }

    if ((count % 1000) == 0)   // 每 1000 次（= 每 1 秒）一次
    {
        // 200hz     ← 原作者的注释写错了，实际是每秒一次。这里什么都没干，留给用户加"低频任务"。
    }

    count++;   // 计数器 +1（配合上面的 % 做分频）
}


/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 *                 把三维向量「从车体坐标系」转到「世界坐标系」
 * @param[1]       vector in BodyFrame   输入的机体系向量
 * @param[2]       vector in EarthFrame  输出的世界系向量
 * @param[3]       quaternion            表示当前姿态的四元数 q[4]
 *
 * 【怎么理解这一长串公式？】
 *   这是"用四元数旋转一个向量"的标准公式（等价于 q·v·q⁻¹），写成展开形式就是这样。
 *   你不需要背它，只需要知道：给一个四元数和一个向量，它就能把向量转到另一个坐标系。
 *   三个输出分量分别对应 x、y、z。系数里出现 (0.5 - q2² - q3²) 这类项是四元数展开后的结果。
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 *                 反方向：把三维向量「从世界坐标系」转到「车体坐标系」
 * @param[1]       vector in EarthFrame   输入的世界系向量
 * @param[2]       vector in BodyFrame    输出的机体系向量
 * @param[3]       quaternion             当前姿态四元数
 *
 * 用途举例：世界系的重力是 (0,0,9.81)，想把它转到车体上看"重力的哪个方向压着车"，
 *          就用这个函数（上面 INS_Task 里算运动加速度时正是这么用的）。
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
 * @brief reserved.用于修正IMU安装误差与标度因数误差,即陀螺仪轴和云台轴的安装偏移
 *
 * 【这个函数在干什么（白话）？】
 *   现实中 IMU 装上云台时，很难做到"芯片的 X 轴正好和云台的 X 轴平行"，
 *   可能歪了 1~2 度，也可能三个轴的灵敏度略有差异。
 *   这个函数就是来补偿这些误差的：
 *     ① 用 Yaw/Pitch/Roll 三个安装偏角算出旋转矩阵 c_11...c_33（就是坐标变换矩阵）
 *     ② 用 scale[3] 修正三个轴的标度（灵敏度）
 *     ③ 把角速度和加速度都乘上这个矩阵 + 标度，得到"已经摆正"的数据
 *
 * 【为什么说本 demo 没用它？】
 *   因为 INS_Init 里把 scale 全设成 1、三个偏角全设成 0，
 *   此时 c_11=1、c_22=1、c_33=1，其余为 0（单位矩阵），乘上去等于没变。
 *   所以你看到它被调用了，但实际上什么都没改。要真用它，得先做一次"安装标定"再填参数。
 *
 * @param param IMU参数（标度 + 三个安装偏角）
 * @param gyro  角速度（会被就地修改）
 * @param accel 加速度（会被就地修改）
 */
static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3])
{
    // 静态变量：记住"上一次用的偏角"，用来判断参数有没有被改过（改了才需要重算矩阵）
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    // 如果三个偏角变了（变化超过 0.001 度），或者 flag 被置 1 → 重算旋转矩阵
    if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
        fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag)
    {
        // 角度转弧度（除以 57.295...），再用硬件三角函数算 sin/cos
        cosYaw = arm_cos_f32(param->Yaw / 57.295779513f);
        cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
        cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
        sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
        sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
        sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        // 下面 9 个就是"按 Z-Y-X 顺序旋转"得到的 3×3 旋转矩阵（照公式写，不用背）
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        param->flag = 0;   // 矩阵算好了，标志清 0（下次不用再算）
    }

    // ---- 用标度系数修正三个轴的角速度 ----
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        gyro_temp[i] = gyro[i] * param->scale[i];

    // ---- 再把修正后的角速度乘上旋转矩阵（矩阵 × 向量），就地写回 gyro ----
    gyro[X] = c_11 * gyro_temp[X] +
              c_12 * gyro_temp[Y] +
              c_13 * gyro_temp[Z];
    gyro[Y] = c_21 * gyro_temp[X] +
              c_22 * gyro_temp[Y] +
              c_23 * gyro_temp[Z];
    gyro[Z] = c_31 * gyro_temp[X] +
              c_32 * gyro_temp[Y] +
              c_33 * gyro_temp[Z];

    // ---- 加速度同样做一次旋转修正（加速度没有标度系数，直接转）----
    float accel_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        accel_temp[i] = accel[i];

    accel[X] = c_11 * accel_temp[X] +
               c_12 * accel_temp[Y] +
               c_13 * accel_temp[Z];
    accel[Y] = c_21 * accel_temp[X] +
               c_22 * accel_temp[Y] +
               c_23 * accel_temp[Z];
    accel[Z] = c_31 * accel_temp[X] +
               c_32 * accel_temp[Y] +
               c_33 * accel_temp[Z];

    // 记住这次用的偏角，下次好判断有没有变化
    lastYawOffset = param->Yaw;
    lastPitchOffset = param->Pitch;
    lastRollOffset = param->Roll;
}

/**
 * @brief 温度控制（给 IMU 加热，让它恒温在 40℃）
 *
 * 为什么需要恒温？陀螺仪的零偏会随温度变化。刚上电芯片是凉的，
 * 慢慢热起来，零偏就一直在变，之前标定好的值就不准了。
 * 所以板上有个小加热电阻，把它烤到 40℃ 并保持住，零偏就稳定了。
 *
 * 流程：读当前温度 → 和目标 40℃ 比较 → PID 算出该给多少加热功率 → 写进 PWM 的比较寄存器
 */
void IMU_Temperature_Ctrl(void)
{
    // ⚠️ PID_Calculate 的参数顺序是：(pid, 测量值, 目标值) —— 先测量后目标，别搞反
    PID_Calculate(&TempCtrl, BMI088.Temperature, RefTemp);

    // 把 PID 输出写进 TIM10 的比较寄存器（0~2000 对应 0~100% 占空比）。
    // float_rounding 把浮点四舍五入成整数；float_constrain 把它夹在 0 ~ UINT32_MAX 之间，防止负数。
    TIM_Set_PWM(&htim10, TIM_CHANNEL_1, float_constrain(float_rounding(TempCtrl.Output), 0, UINT32_MAX));
}

//------------------------------------functions below are not used in this demo-------------------------------------------------
//----------------------------------you can read them for learning or programming-----------------------------------------------
//----------------------------------they could also be helpful for further design-----------------------------------------------
// （下面这几个函数本 demo 都没有用到，属于"备用工具箱"，你要自己扩展功能时可以用）

/**
 * @brief        Update quaternion
 *               纯陀螺积分的四元数更新（不用加速度计修正）
 *
 * 输入：四元数指针 q、三轴角速度 gx/gy/gz、时间间隔 dt
 * 输出：直接就地修改 q
 * 用途：这是最"朴素"的姿态更新方式 —— 只靠陀螺积分，短期准、长期一定漂。
 *      EKF 里用的是它的"线性化版本"，这个函数留给简单场景或做对比。
 */
void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt)
{
    float qa, qb, qc;

    // 先算 0.5*ω*dt，后面反复用
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = q[0];   // 存一份旧的实部
    qb = q[1];
    qc = q[2];
    // 四元数微分方程的展开式（乘完就得到新的四元数），照公式写，不用背
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);
}

/**
 * @brief        Convert quaternion to eular angle
 *               四元数 → 欧拉角（Yaw/Pitch/Roll）
 *
 * 输入：q[4]；输出：三个角度指针（单位：度）
 * 注意：这里的 Yaw 范围是 ±180°，需要连续角度的话看 YawTotalAngle 那套逻辑。
 */
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll)
{
    // 三个公式都是四元数转欧拉角的标准形式（atan2f/asinf 是反三角，57.2957…是 180/π）
    *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}

/**
 * @brief        Convert eular angle to quaternion
 *               欧拉角 → 四元数（上面那个的反向操作）
 *
 * 输入：Yaw/Pitch/Roll（度）；输出：q[4]
 * 用途：比如你想"把姿态初始化成某个角度"，就用它算出对应的四元数。
 */
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q)
{
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    // 先统一把"度"换成"弧度"
    Yaw /= 57.295779513f;
    Pitch /= 57.295779513f;
    Roll /= 57.295779513f;
    // 三角函数的"半角"（注意除以 2，这是欧拉角转四元数的公式要求）
    cosPitch = arm_cos_f32(Pitch / 2);
    cosYaw = arm_cos_f32(Yaw / 2);
    cosRoll = arm_cos_f32(Roll / 2);
    sinPitch = arm_sin_f32(Pitch / 2);
    sinYaw = arm_sin_f32(Yaw / 2);
    sinRoll = arm_sin_f32(Roll / 2);
    // 四个分量按公式组合（照抄即可，不用背）
    q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
    q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
    q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
    q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}
