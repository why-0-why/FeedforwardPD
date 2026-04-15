/**
 ******************************************************************************
 * @file           : alg_inertia.c
 * @author         : WHY
 * @date           : 2026-4-14
 * @brief          : 单关节惯量辨识模块（最小二乘法）
 *
 * 辨识模型（去除已知重力前馈后）：
 *     τ - τ_ff = J·α + b·ω
 *
 * 其中：
 *   τ      —— 电机反馈力矩 (N·m)
 *   τ_ff   —— 重力前馈力矩（由已辨识重力参数计算）
 *   J      —— 待辨识等效转动惯量 (kg·m²)
 *   b      —— 待辨识粘滞摩擦系数 (N·m·s/rad)
 *   α      —— 关节角加速度（由角速度差分估算）
 *   ω      —— 关节角速度（电机反馈）
 *
 * 将 N 帧数据写成矩阵形式：
 *     Y = X · theta
 *     Y[i] = τ[i] - τ_ff[i]
 *     X[i] = [α[i],  ω[i]]
 *     theta = [J,  b]^T
 *
 * 调用 DATA_OLS() 求解正规方程得到 theta。
 ******************************************************************************
 * @attention
 *    - 角加速度由相邻帧角速度差分估算，首帧无加速度信息，跳过不采样
 *    - 激励信号需覆盖足够的角加速度与角速度范围，否则辨识精度下降
 *    - 建议在电机做持续变速运动时（如正弦轨迹）采集数据
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "alg_inertia.h"
#include "alg_OLS.h"
#include <string.h>

/* Private define ------------------------------------------------------------*/
/** 最小求解样本数（2×2 最小二乘至少需要 2 个线性无关行） */
#define INERTIA_SOLVE_MIN_SAMPLES 4

/* Private function prototypes -----------------------------------------------*/

/* Public functions ----------------------------------------------------------*/

/**
 * @brief:      DATA_InertiaInit: 初始化惯量辨识句柄
 * @param[in]:  ident  句柄指针
 * @param[in]:  dt_s   采样周期 (s)
 * @retval:     void
 * @details:    清零缓冲区，设置采样周期，将 vel_prev 初始化为无效标记
 **/
void DATA_InertiaInit(STR_InertiaIdent* ident, float32_t dt_s)
{
    memset(ident, 0, sizeof(STR_InertiaIdent));//memset作用: 将内存块清零
    ident->dt_s     = dt_s;
    ident->vel_prev = 0.0f;
    ident->ready    = 0;
    ident->n_sample = -1; // -1 表示首帧，需跳过（无差分加速度）
}

/**
 * @brief:      DATA_InertiaReset: 重置辨识句柄
 * @param[in]:  ident  句柄指针
 * @retval:     void
 * @details:    清空采样缓冲区与辨识结果，保留 dt_s 采样周期设置
 **/
void DATA_InertiaReset(STR_InertiaIdent* ident)
{
    float32_t dt_bak = ident->dt_s;
    memset(ident, 0, sizeof(STR_InertiaIdent));
    ident->dt_s     = dt_bak;
    ident->n_sample = -1;
}

/**
 * @brief:      DATA_InertiaPushSample: 推送一帧测量数据并累积样本
 * @param[in]:  ident   句柄指针
 * @param[in]:  vel     当前角速度 (rad/s)
 * @param[in]:  tor     当前关节力矩 (N·m)
 * @param[in]:  tor_ff  重力前馈力矩 (N·m)
 * @retval:     当前已采集样本数；若已触发求解则返回 INERTIA_SAMPLE_MAX
 * @details:    首帧仅记录 vel_prev，不计入样本；
 *              此后每帧由差分估算 α，填入 X/Y 缓冲区；
 *              缓冲区满时自动调用 DATA_InertiaSolve()
 **/
int8_t DATA_InertiaPushSample(STR_InertiaIdent* ident,
                              float32_t vel,
                              float32_t tor,
                              float32_t tor_ff)
{
    /* 首帧：仅保存 vel_prev，跳过采样 */
    if (ident->n_sample < 0)
    {
        ident->vel_prev = vel;
        ident->n_sample = 0;
        return 0;
    }

    /* 缓冲区已满，不再接受新样本 */
    if (ident->n_sample >= INERTIA_SAMPLE_MAX)
    {
        return INERTIA_SAMPLE_MAX;
    }

    /* 差分估算角加速度 */
    float32_t acc = (vel - ident->vel_prev) / ident->dt_s;
    ident->vel_prev = vel;

    /* 填充回归矩阵行：[α, ω] */
    int8_t idx = ident->n_sample;
    ident->X[idx * 2 + 0] = acc;
    ident->X[idx * 2 + 1] = vel;

    /* 填充目标向量：τ - τ_ff */
    ident->Y[idx] = tor - tor_ff;

    ident->n_sample++;

    /* 缓冲区满，自动触发求解 */
    if (ident->n_sample >= INERTIA_SAMPLE_MAX)
    {
        DATA_InertiaSolve(ident);
    }

    return ident->n_sample;
}

/**
 * @brief:      DATA_InertiaSolve: 对已采集样本执行最小二乘辨识
 * @param[in]:  ident  句柄指针
 * @retval:      0: 成功，-1: 失败（样本不足或矩阵奇异）
 * @details:    调用 DATA_OLS() 求解 Y = X * [J, b]^T，
 *              结果写入 ident->J_est 与 ident->b_est，
 *              并置位 ident->ready = 1
 **/
int8_t DATA_InertiaSolve(STR_InertiaIdent* ident)
{
    if (ident->n_sample < INERTIA_SOLVE_MIN_SAMPLES)
    {
        return -1;
    }

    float32_t theta[2] = {0.0f, 0.0f};

    int8_t ret = DATA_OLS(ident->n_sample, 2, ident->X, ident->Y, theta);

    if (ret == 0)
    {
        ident->J_est = theta[0];
        ident->b_est = theta[1];
        ident->ready = 1;
    }

    return ret;
}
