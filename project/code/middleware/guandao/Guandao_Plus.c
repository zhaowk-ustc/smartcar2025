/*
 * Guandao_Plus.c
 *
 *  Created on: 2025年3月26日
 *      Author: 林林林
 */
#include "zf_common_headfile.h"
#include <stdbool.h> // 引入 stdbool.h
#include "control.h"
#include "flash.h"
#include "imu.h"
#include "motor.h"
#include "PID.h"
#include "Display.h"
#include "Guandao_Plus.h"
#include "key.h"
#include "filter.h"
/**
 * ======= 纯跟踪算法可调参数区域 =======
 * 以下参数影响Pure Pursuit算法的性能和效率，可根据实际场景调整
 */
// 最近点查找范围（前后各多少个点）
// - 增大此值可以提高在大弯道或车辆偏离路径时的鲁棒性
// - 减小此值可以提高计算效率，适合路径简单或车辆偏离较小的场景
// - 推荐范围：10~50
#define LOOKAHEAD_SEARCH_RANGE 30

// 预瞄点最大搜索步数（从最近点向前搜索的最大点数）
// - 增大此值可以处理较大的预瞄距离，但会增加计算量
// - 减小此值可以提高效率，适合预瞄距离较小或路径点密集的场景
// - 推荐范围：10~30
#define LOOKAHEAD_MAX_STEPS 20

// 跳跃步长估算的平均点间距（单位：cm）
// - 此值应等于或接近路径点的实际间距
// - 若路径点每1cm记录一次，则设为1.0f
// - 若路径点间距不确定，建议保守设置为1.0f
// - 此值直接影响跳跃搜索的准确性和效率
#define LOOKAHEAD_AVG_DIST 1.0f

// 输出差速的经验比例系数
#define SCALE_FACTOR 80

/**
 * ===================================
 */

// 小车物理参数
#define distance (1.0) // 每间隔1cm记录一次点位
#define WHEELBASE 17   // 轮距（左右轮距离，单位cm）

float X_Memery_Plus[FLASH_PAGE_LENGTH * 6] = {0};
float Y_Memery_Plus[FLASH_PAGE_LENGTH * 6] = {0};
float X_Memery_Store_Plus[FLASH_PAGE_LENGTH * 6] = {0}; // 小车记忆路径（东天北坐标系）单位cm
float Y_Memery_Store_Plus[FLASH_PAGE_LENGTH * 6] = {0}; // 小车每走1cm记录一个路径点
volatile float X = 0, Y = 0;                            // 小车实时位置（东天北坐标系）单位cm

uint8_t road_memery_finish_Plus_flag = 0; // 路径记忆完成标志位
uint8_t road_memery_start_Plus_flag = 0;  // 路径记忆开始标志位
uint8_t road_recurrent_Plus_flag = 0;     // 路径复现标志位
uint16_t NUM_L_Plus = 0, NUM_R_Plus = 0;  // 每走过1cm,NUM+1
uint16_t road_destination = 0;            // 路径终点

/*           */
volatile int16 err_guandao_plus = 0; // 惯导差速
volatile float e_lat = 0;            // 惯导横向误差
volatile int32_t speed_left = 0;     // 左轮速度（脉冲/5ms）
volatile int32_t speed_right = 0;    // 右轮速度（脉冲/5ms）
// 小车实时状态
volatile int locate_index = 0;           // 小车定位点
static int target_index = 0;             // 小车目标点
const int32_t MAX_PHYSICAL_DELTA = 1000; // 最大允许物理增量，用于防御性检查

volatile SlipFlags wheelSlipFlags = SLIP_FLAG_NONE;
volatile float leftWheelSpeedCmps = 0;
volatile float rightWheelSpeedCmps = 0;
volatile float currentAvgSpeedCmps = 0;

/**
 * @brief 轮子数据结构体，封装单个轮子的编码器相关数据
 *
 * @成员 total_pulses     累计脉冲数，用于虚拟清零功能
 * @成员 encoder_last     上一次读取的编码器值，用于计算增量
 * @成员 encoder_prev_speed 速度计算基准值，保存上一次中断时的编码器值
 * @成员 finish_flag;     达到设定阈值标志位
 * @成员 target_pulses    目标脉冲数，达到此值后触发虚拟清零（应设为正数）
 */

// 初始化左轮数据结构体
// - total_pulses: 初始累计脉冲数为0
// - last_total_pulses: 初始上一次脉冲数为0
// - encoder_last: 初始编码器值为0
// - encoder_prev_speed: 初始速度基准值为0
// - target_pulses: 虚拟清零阈值为366脉冲（需根据实际调整）
WheelData wheel_left = {
    .total_pulses = 0,
    .last_total_pulses = 0,
    .encoder_last = 0,
    .encoder_prev_speed = 0,
    .target_pulses = 366};

// 初始化右轮数据结构体（配置同左轮）
WheelData wheel_right = {
    .total_pulses = 0,
    .last_total_pulses = 0,
    .encoder_last = 0,
    .encoder_prev_speed = 0,
    .target_pulses = 366};

/**
 * @brief 计算编码器增量，自动处理int16溢出
 *
 * @param current 当前编码器值（int16类型）
 * @param last 上一次编码器值（int16类型）
 * @return int32_t 修正后的增量（实际物理脉冲变化）
 *
 * @说明
 * - 编码器计数器为16位有符号数（-32768~32767），
 *   当计数器溢出时（如从32767变为-32768），直接相减会得到错误增量。
 * - 此函数通过判断差值范围，修正正向和反向溢出。
 */
static inline int32_t calculate_delta(int16_t current, int16_t last)
{
    int32_t delta = (int32_t)current - (int32_t)last;

    // 处理正向溢出（例如从32767到-32768，实际增量为+1）
    if (delta > 32767)
    {
        delta -= 65536; // 65536 = 2^16
    }
    // 处理反向溢出（例如从-32768到32767，实际增量为-1）
    else if (delta < -32768)
    {
        delta += 65536;
    }

    return delta;
}

/**
 * @brief 更新轮子脉冲累计值并处理虚拟清零
 *
 * @param wheel 轮子数据指针（指向左轮或右轮的结构体）
 * @param current_encoder 当前编码器值
 *
 * @说明
 * - 此函数应在主循环中定期调用，确保实时更新脉冲累计值。
 * - 防御性检查过滤异常脉冲跳变（如信号干扰）。
 * - 使用"跨阈值检测法"同时支持正向和反向累计，解决前进后退交替时计数卡住问题。
 * - 添加智能delta处理，解决正反转切换时编码器值跳变导致的计数卡住问题。
 */
int32_t Update_Wheel_Pulses(WheelData *wheel, int16_t current_encoder)
{
    // 1. 计算增量 (包含溢出处理)
    int32_t delta = calculate_delta(current_encoder, wheel->encoder_last);
    int32_t forward_crossings = 0; // 在函数开始时声明并初始化

    // 2. 智能跳变检测逻辑
    //    阈值需要根据实际情况调整，目标是捕获硬件跳变，但不误伤高速脉冲。
    if (abs(delta) > 1000)
    {
        // printf("Jump detected! Delta: %d, LastEnc: %d, CurrEnc: %d\n",
        //        (int)delta, (int)wheel->encoder_last, (int)current_encoder); // 调试输出

        // 重置累计脉冲，避免跳变干扰后续计数
        wheel->total_pulses = 0;
        // 本次调用不计算位移，因为发生了跳变重置
        forward_crossings = 0; // 确保跳变时返回0
    }
    else
    {
        // --- 正常脉冲累加和计数值计算 ---
        // 保存上一次的累计脉冲值，用于检测跨阈值 (这个变量在新的逻辑中实际未使用，但保留也无害)
        wheel->last_total_pulses = wheel->total_pulses;

        // 更新累计脉冲数（支持正反转，负值表示反向转动）
        wheel->total_pulses += delta;

        // 计算前向阈值穿越次数
        if (wheel->total_pulses >= wheel->target_pulses)
        {
            forward_crossings = wheel->total_pulses / wheel->target_pulses; // 计算穿越了多少次阈值
            wheel->total_pulses %= wheel->target_pulses;                    // 保留余数，用于下一次累计
        }
        // 处理反向穿越 (如果需要反向计数逻辑的话)
        else if (wheel->total_pulses <= -wheel->target_pulses)
        {
            wheel->total_pulses %= wheel->target_pulses; // 模拟取模，保持在阈值范围内
                                                         // 确保如果刚好是 -target_pulses，模运算后变为 0
            if (wheel->total_pulses == -wheel->target_pulses)
            {
                wheel->total_pulses = 0;
            }
        }
    }

    // 3. 保存当前编码器值供下次使用
    wheel->encoder_last = current_encoder;

    // 4. 返回本次计算出的前进厘米数
    return forward_crossings;
}

/**
 * @brief 获取轮子在固定时间窗口5ms内的速度（脉冲增量）
 *
 * @param wheel 轮子数据指针
 * @param current_encoder 当前编码器值
 * @return int32_t 时间窗口内的脉冲增量（正值为正向，负值为反向）
 *
 * @说明
 * - 此函数应在定时中断（如5ms）中调用，确保固定时间间隔。
 * - 每次调用会更新encoder_prev_speed为当前值，供下次计算使用。
 */
int32_t Get_Wheel_Speed(WheelData *wheel, int16_t current_encoder)
{
    // 计算当前编码器值与上一次中断时的基准值的增量
    int32_t delta = calculate_delta(current_encoder, wheel->encoder_prev_speed);

    // 更新基准值为当前值，用于下一次速度计算
    wheel->encoder_prev_speed = current_encoder;

    if (abs(delta) > 1000)
        delta = 0;
    return delta;
}

/*---------------------------------------------
 * 1. 路径记忆函数
 *---------------------------------------------*/
void Distance_Get_Plus(void)
{
    // 确保不会越界访问数组
    if (NUM_L_Plus >= FLASH_PAGE_LENGTH * 6 - 2)
    {
        road_memery_finish_Plus_flag = 1; // 路径记忆完成标志位
        return;                           // 直接返回，不再记录新的点
    }

    NUM_L_Plus++;
    X += distance * (-sinf(yaw_plus));
    Y += distance * cosf(yaw_plus);
    X_Memery_Plus[NUM_L_Plus] = X;
    Y_Memery_Plus[NUM_L_Plus] = Y;
}

/*---------------------------------------------
 * 2. 预瞄点查找函数
 *---------------------------------------------*/
/**
 * @brief 查找距离当前位姿L厘米的预瞄点索引
 * @param L 预瞄距离（单位：厘米）
 * @return 预瞄点在路径数组中的索引
 *
 * @说明 此函数使用"距离法"查找预瞄点，确保预瞄点始终在车辆前方固定物理距离处
 * 算法分三步：
 * 1. 先找到距离当前位置最近的前方路径点作为参考点（定位点）
 * 2. 使用跳跃式搜索策略，快速接近可能的预瞄点位置
 * 3. 从跳跃位置精确搜索，找到第一个距离当前位置大于等于预瞄距离L的点
 *
 * 优势：
 * - 相比简单的"索引偏移法"更加精确，能够适应路径点分布不均匀的情况
 * - 使用"只选前方点"策略，避免小车回头跟踪
 * - 跳跃式搜索大幅减少计算量，尤其适合长路径
 * - 严格的边界保护，防止数组越界
 */
void find_lookahead_index(float L)
{
    /**
     * 步骤1：找到距离当前位置最近的前方路径点
     */
    // 初始化最小距离为最大浮点值，确保第一次比较时一定会更新
    float min_distance = FLT_MAX;
    int closest_index = 0;

    // 搜索范围控制：以当前目标点为中心，向前后各搜索LOOKAHEAD_SEARCH_RANGE个点
    // 这样可以适应车辆偏离路径或转弯的情况，同时限制计算量
    int search_start = target_index - LOOKAHEAD_SEARCH_RANGE;
    int search_end = target_index + LOOKAHEAD_SEARCH_RANGE;

    // 边界检查：防止索引越界，确保安全访问数组
    if (search_start < 0)
        search_start = 0;
    if (search_end >= FLASH_PAGE_LENGTH * 6)
        search_end = FLASH_PAGE_LENGTH * 6 - 1;

    // 在设定范围内寻找距离当前位置最近的前方路径点
    for (int i = search_start; i <= search_end; i++)
    {
        // 计算当前点到路径点的欧氏距离
        float dx = X_Memery_Store_Plus[i] - X; // X方向差值
        float dy = Y_Memery_Store_Plus[i] - Y; // Y方向差值
        float dist = sqrtf(dx * dx + dy * dy); // 欧氏距离

        // 更新最近点：若当前点距离更小，则更新最近点索引
        if (dist < min_distance)
        {
            min_distance = dist;
            closest_index = i;
        }
    }

    // 更新定位点为找到的最近点（这是小车在路径上的"投影点"）
    locate_index = closest_index;

    /**
     * 步骤2：跳跃式搜索策略
     */
    // 限制预瞄点搜索范围：最多向前搜索LOOKAHEAD_MAX_STEPS个点
    // 这样可以避免在长路径时搜索到终点，提高效率
    int search_limit = closest_index + LOOKAHEAD_MAX_STEPS;
    if (search_limit >= FLASH_PAGE_LENGTH * 6)
        search_limit = FLASH_PAGE_LENGTH * 6 - 1;

    // 估算大致的跳跃步数：预瞄距离÷平均点间距
    // 这样可以快速接近可能的预瞄点位置，减少后续精确搜索的范围
    int jump_steps = (int)(L / LOOKAHEAD_AVG_DIST);
    if (jump_steps < 1)
        jump_steps = 1; // 确保至少跳跃1步

    // 计算跳跃目标点：在最近点基础上向前跳跃
    int jump_target = closest_index + jump_steps;
    if (jump_target > search_limit)
        jump_target = search_limit; // 确保不超过搜索范围

    // 计算跳跃点到当前位置的实际距离
    float jump_dx = X_Memery_Store_Plus[jump_target] - X;
    float jump_dy = Y_Memery_Store_Plus[jump_target] - Y;
    float jump_dist = sqrtf(jump_dx * jump_dx + jump_dy * jump_dy);

    /**
     * 步骤3：确定精确搜索范围
     */
    // 根据跳跃结果，确定需要精确搜索的区间
    int fine_search_start, fine_search_end;

    if (jump_dist < L)
    {
        // 情况1：跳跃后距离仍小于预瞄距离
        // 说明预瞄点在跳跃点之后，从跳跃点开始向前搜索
        fine_search_start = jump_target;
        fine_search_end = search_limit;
    }
    else
    {
        // 情况2：跳跃后距离已大于预瞄距离
        // 说明预瞄点在最近点和跳跃点之间，缩小搜索范围
        fine_search_start = closest_index;
        fine_search_end = jump_target;
    }

    /**
     * 步骤4：精确搜索预瞄点
     */
    // 在确定的精确搜索区间内，找到第一个距离大于等于L的点
    for (int i = fine_search_start; i <= fine_search_end; i++)
    {
        float dx = X_Memery_Store_Plus[i] - X;
        float dy = Y_Memery_Store_Plus[i] - Y;
        float dist = sqrtf(dx * dx + dy * dy);

        // 找到第一个距离大于等于预瞄距离L的点作为预瞄点
        if (dist >= L)
        {
            target_index = i; // 更新目标点（预瞄点）
            return;           // 找到合适点后立即返回，避免多余搜索
        }
    }

    // 边界处理：如果搜索区间内没有找到合适点（如已接近路径终点）
    // 则使用搜索区间的最后一个点作为预瞄点
    target_index = fine_search_end;
}

/*---------------------------------------------
 * 3. 横向误差计算（右前上坐标系）
 *---------------------------------------------*/
/**
 * @brief 计算预瞄点的横向误差（右前上坐标系）
 * @param lookahead_index 预瞄点索引
 * @return 横向误差e_lat（厘米，正值表示预瞄点在右侧）
 */
float calculate_e_lat(int lookahead_index)
{
    float dx = X_Memery_Store_Plus[lookahead_index] - X;
    float dy = Y_Memery_Store_Plus[lookahead_index] - Y;
    // 直接使用航向角yaw_plus（顺时针为负，逆时针为正）
    float rotated_x = dx * cosf(yaw_plus) + dy * sinf(yaw_plus);
    return rotated_x; // 正值表示预瞄点在小车右侧
}

/*---------------------------------------------
 * 4. Pure Pursuit核心算法
 *---------------------------------------------*/

/**
 * @brief 纯跟踪控制算法主函数
 * @return 差速控制量（负值表示左转，正值表示右转）
 *
 * @说明 纯跟踪算法的核心思想是：
 * 1. 根据当前位置和预瞄距离L，在路径上找到一个预瞄点
 * 2. 计算从当前位置到预瞄点的曲率
 * 3. 根据曲率计算差速控制量
 *
 * 优点：算法简单直观，计算量小，适合低速场景
 * 参数调优：预瞄距离L是最关键参数，L越小控制越灵敏但容易震荡，L越大控制越平滑但跟踪精度降低
 */
int16_t pure_pursuit_control()
{

    // 设置预瞄距离（单位：厘米）
    // 注：低速场景使用较小的预瞄距离可获得更精确的跟踪效果
    // 可选优化：根据速度动态调整 L_distance = K1 * speed + K2
    static float L_distance = 20.0f; // 预瞄距离，单位厘米，低速场景使用较小值

    // 使用"距离法"查找预瞄点
    find_lookahead_index(L_distance);

    // 计算横向误差（车体坐标系下，预瞄点相对车身的横向偏差）
    e_lat = calculate_e_lat(target_index);

    // 计算预瞄点到当前位置的实际距离
    float dx = X_Memery_Store_Plus[target_index] - X;
    float dy = Y_Memery_Store_Plus[target_index] - Y;
    float L_actual = sqrtf(dx * dx + dy * dy); // 实际预瞄距离，单位厘米

    // 防止除零（实际工程中不大可能出现，但为防御性编程）
    if (L_actual < 1e-3f)
        L_actual = 1e-3f;

    // 计算曲率 - 纯跟踪控制核心公式
    // 曲率k = 2 * 横向误差 / 预瞄距离的平方
    float curvature = 2.0f * e_lat / (L_actual * L_actual);

    // 将曲率转换为差速控制量
    // delta_v = 曲率 * 轮距 * 比例系数
    // 注：比例系数可根据实际控制效果调整
    int16_t delta_v = (int16_t)(curvature * WHEELBASE * SCALE_FACTOR);

    // 差速限制，防止电机过载和控制量突变
    if (delta_v > 200)
        delta_v = 200;
    if (delta_v < -200)
        delta_v = -200;

    return delta_v; // 返回差速控制量
}

// 打滑检测
/*-----------------------------------------------------------------------------
 * 函数名称：UpdateWheelSpeeds
 * 功能描述：将speed_left,speed_right转化为实际轮速物理量
 * 输入参数：leftPulses  - 左轮脉冲增量（脉冲数/采样周期）即Speed_left
 *          rightPulses - 右轮脉冲增量（脉冲数/采样周期）即speed_right
 * 输出参数：无
 * 注    意：需在固定时间间隔（如5ms）调用以保证速度计算准确性
 *----------------------------------------------------------------------------*/
void UpdateWheelSpeeds(int32_t leftPulses, int32_t rightPulses)
{

    // 脉冲转线速度：脉冲数 → 厘米/秒
    // 公式：速度 = (固定时间间隔内脉冲增量/每cm脉冲数) /采样间隔(单位秒)
    static float scale = (1000.0f / SAMPLING_INTERVAL_MS);

    leftWheelSpeedCmps = (float)leftPulses / ENCODER_PULSES_PER_REVOLUTION * scale;
    rightWheelSpeedCmps = (float)rightPulses / ENCODER_PULSES_PER_REVOLUTION * scale;
}

/*-----------------------------------------------------------------------------
 * 函数名称：DetectLateralSlip
 * 功能描述：检测横向滑移（差速转向不匹配）
 * 输入参数：无
 * 输出参数：更新wheelSlipFlags中的横向滑移标志位
 * 核心逻辑：比较理论偏航率（基于轮速差）与IMU实际偏航率的偏差
 *----------------------------------------------------------------------------*/
void DetectLateralSlip(void)
{
    // 计算理论偏航率（单位：弧度/秒）
    // 公式：理论偏航率 = (右轮速度 - 左轮速度) / 轮距
    float theoreticalYawRate = (rightWheelSpeedCmps - leftWheelSpeedCmps) / WHEEL_BASE_CM;

    // 计算偏航率偏差（绝对值）
    float yawRateDeviation = fabsf(theoreticalYawRate - gyro_param.gyro_z); // IMU测量的偏航角速度（单位：弧度/秒）

    // 计算平均速度（判断是否满足检测条件）
    float avgSpeedCmps = 0.5f * (fabsf(leftWheelSpeedCmps) + fabsf(rightWheelSpeedCmps));

    // 检测逻辑
    if (avgSpeedCmps > MIN_LATERAL_DETECT_SPEED_CMPS &&
        yawRateDeviation > LATERAL_SLIP_YAW_RATE_THRESHOLD)
    {
        // 判断滑移方向
        if (theoreticalYawRate > gyro_param.gyro_z)
        {
            wheelSlipFlags |= SLIP_FLAG_LATERAL_RIGHT; // 右轮打滑导致转向不足
        }
        else
        {
            wheelSlipFlags |= SLIP_FLAG_LATERAL_LEFT; // 左轮打滑导致转向过度
        }
    }
}

/*-----------------------------------------------------------------------------
 * 函数名称：DetectLongitudinalSlip
 * 功能描述：检测纵向滑移（驱动轮空转或抱死）
 * 输入参数：无
 * 输出参数：更新wheelSlipFlags中的纵向滑移标志位
 * 核心逻辑：比较轮速计算的理论加速度与IMU实际加速度的差异
 *----------------------------------------------------------------------------*/
void DetectLongitudinalSlip(void)
{

    // 计算当前平均轮速（单位：米/秒）
    currentAvgSpeedCmps = 0.5f * (leftWheelSpeedCmps + rightWheelSpeedCmps) / 100.0f;

    float accelDiff = fabsf(currentAvgSpeedCmps - acc_y_speed); // 编码器速度与加速度计速度差（单位：米/秒）

    // printf("%f,%f\n", accelDiff, currentAvgSpeedCmps); // 打印调试
    //  检测逻辑
    if (currentAvgSpeedCmps > MIN_LONGITUDINAL_DETECT_SPEED_CMPS)
    {
        if (accelDiff > LONGITUDINAL_ACCEL_DIFF_THRESHOLD)
        {
            wheelSlipFlags |= SLIP_FLAG_LONGITUDINAL_ACCEL; // 滑移
        }
    }

    // static float prevAvgSpeedCmps = 0.0f; // 上一次的平均速度

    // // 计算当前平均轮速（单位：厘米/秒）
    // float currentAvgSpeedCmps = 0.5f * (leftWheelSpeedCmps + rightWheelSpeedCmps);

    // // 计算理论加速度（单位：米/秒）
    // // 公式：加速度 = (当前速度 - 上次速度) / 采样时间
    // float timeIntervalSec = 1000.0f / SAMPLING_INTERVAL_MS;
    // float theoreticalAccelMps2 = (currentAvgSpeedCmps - prevAvgSpeedCmps) / 100.0f * timeIntervalSec;

    // // 更新历史速度
    // prevAvgSpeedCmps = currentAvgSpeedCmps;

    // // 计算加速度差异（绝对值）
    // float accelDiff = fabsf(theoreticalAccelMps2 - acc_param.acc_y); // IMU测量的前向加速度（单位：米/秒?）

    // printf("%f,%f\n", accelDiff, currentAvgSpeedCmps); // 打印调试
    // // 检测逻辑
    // if (currentAvgSpeedCmps > MIN_LONGITUDINAL_DETECT_SPEED_CMPS)
    // {
    //     if (theoreticalAccelMps2 > 0.1f && accelDiff > LONGITUDINAL_ACCEL_DIFF_THRESHOLD)
    //     {
    //         wheelSlipFlags |= SLIP_FLAG_LONGITUDINAL_ACCEL; // 加速滑移（驱动轮空转）
    //     }
    //     else if (theoreticalAccelMps2 < -0.1f && accelDiff > LONGITUDINAL_ACCEL_DIFF_THRESHOLD)
    //     {
    //         wheelSlipFlags |= SLIP_FLAG_LONGITUDINAL_DECEL; // 减速滑移（轮胎抱死）
    //     }
    // }
}

void Slip_Check(void)
{

    UpdateWheelSpeeds(speed_left, speed_right);
    DetectLongitudinalSlip();
    DetectLateralSlip();
}