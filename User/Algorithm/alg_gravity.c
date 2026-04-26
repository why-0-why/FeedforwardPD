/**
 ******************************************************************************
 * @file           : alg_gravity.c
 * @author         : WHY
 * @date           : 2026-4-26
 * @brief          : 单关节重力辨识模块（最小二乘法）
 *
 * 辨识模型：
 *     τ = θ₁·cos(pos) + θ₂·sin(pos)
 *
 * 其中：
 *   τ    —— 电机反馈力矩 (N·m)，在静止状态下采集
 *   pos  —— 关节角度 (rad)
 *   θ₁   —— 重力参数（余弦分量）
 *   θ₂   —— 重力参数（正弦分量）
 *
 * 将 N 帧数据写成矩阵形式：
 *     Y = X · theta
 *     Y[i] = τ[i]
 *     X[i] = [cos(pos[i]),  sin(pos[i])]
 *     theta = [θ₁,  θ₂]^T
 *
 * 调用 DATA_OLS() 求解正规方程得到 theta。
 ******************************************************************************
 * @attention
 *    - 采样时电机须处于静止（或低速）状态，以排除惯量和摩擦的影响
 *    - 采样点应覆盖足够的角度范围，否则辨识精度下降
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "alg_gravity.h"
#include "alg_OLS.h"
#include <string.h>
#include <math.h>

/* Private define ------------------------------------------------------------*/
/** 最小求解样本数（2 参数最小二乘至少需要 3 个线性无关行） */
#define GRAVITY_SOLVE_MIN_SAMPLES 3

/* Public functions ----------------------------------------------------------*/

/**
 * @brief:      DATA_GravityInit: 初始化重力辨识句柄
 * @param[in]:  ident  句柄指针
 * @retval:     void
 * @details:    清零缓冲区与辨识结果
 **/
void DATA_GravityInit(STR_GravityIdent* ident)
{
    memset(ident, 0, sizeof(STR_GravityIdent));
}

/**
 * @brief:      DATA_GravityReset: 重置辨识句柄
 * @param[in]:  ident  句柄指针
 * @retval:     void
 * @details:    清空采样缓冲区与辨识结果
 **/
void DATA_GravityReset(STR_GravityIdent* ident)
{
    memset(ident, 0, sizeof(STR_GravityIdent));
}

/**
 * @brief:      DATA_GravityPushSample: 推送一帧静止测量数据
 * @param[in]:  ident  句柄指针
 * @param[in]:  pos    当前关节角度 (rad)
 * @param[in]:  tor    当前关节力矩 (N·m)
 * @retval:     当前已采集样本数；缓冲区满时返回 GRAVITY_SAMPLE_MAX
 **/
int8_t DATA_GravityPushSample(STR_GravityIdent* ident, float32_t pos, float32_t tor)
{
    if (ident->n_sample >= GRAVITY_SAMPLE_MAX)
    {
        return GRAVITY_SAMPLE_MAX;
    }

    int8_t idx = ident->n_sample;
    ident->pos[idx] = pos;
    ident->tor[idx] = tor;
    ident->n_sample++;

    return ident->n_sample;
}

/**
 * @brief:      DATA_GravitySolve: 对已采集样本执行最小二乘辨识
 * @param[in]:  ident  句柄指针
 * @retval:      0: 成功，-1: 失败（样本不足或矩阵奇异）
 * @details:    构建回归矩阵 X = [cos(pos), sin(pos)]，
 *              调用 DATA_OLS() 求解 Y = X * [θ₁, θ₂]^T，
 *              结果写入 ident->theta，并置位 ident->ready = 1
 **/
int8_t DATA_GravitySolve(STR_GravityIdent* ident)
{
    if (ident->n_sample < GRAVITY_SOLVE_MIN_SAMPLES)
    {
        return -1;
    }

    float32_t X[GRAVITY_SAMPLE_MAX * 2];
    for (int8_t i = 0; i < ident->n_sample; i++)
    {
        X[i * 2 + 0] = cosf(ident->pos[i]);
        X[i * 2 + 1] = sinf(ident->pos[i]);
    }

    int8_t ret = DATA_OLS(ident->n_sample, 2, X, ident->tor, ident->theta);

    if (ret == 0)
    {
        ident->ready = 1;
    }

    return ret;
}

/**
 * @brief:      DATA_GravityCalcFF: 计算当前位置的重力前馈力矩
 * @param[in]:  theta    辨识参数数组 [θ₁, θ₂]
 * @param[in]:  pos_rad  当前关节角度 (rad)
 * @retval:     重力前馈力矩 (N·m)
 **/
float32_t DATA_GravityCalcFF(const float32_t* theta, float32_t pos_rad)
{
    return theta[0] * cosf(pos_rad) + theta[1] * sinf(pos_rad);
}
