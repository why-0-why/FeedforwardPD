/**
 ******************************************************************************
 * @file           : alg_gravity.h
 * @author         : WHY
 * @date           : 2026-4-26
 * @brief          : alg_gravity.c 的头文件
 *                   包含单关节重力辨识（最小二乘法）的结构体与接口声明
 *
 * 辨识模型：τ = θ₁·cos(pos) + θ₂·sin(pos)
 *   τ    : 关节力矩测量值（电机反馈，静止状态）
 *   pos  : 关节角度 (rad)
 *   θ₁   : 待辨识重力参数（余弦分量）
 *   θ₂   : 待辨识重力参数（正弦分量）
 *
 * 前馈力矩计算：τ_ff = θ₁·cos(pos) + θ₂·sin(pos)
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ALG_GRAVITY_H
#define ALG_GRAVITY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "arm_math.h"

/* Exported defines ----------------------------------------------------------*/

/** 辨识样本最大容量；按需修改以平衡内存与精度 */
#define GRAVITY_SAMPLE_MAX 16

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 单关节重力辨识句柄
 *
 * 维护采样缓冲区，并在调用 DATA_GravitySolve() 时执行最小二乘求解。
 * 所有字段均由 DATA_GravityInit() 初始化后内部管理，外部只读辨识结果。
 */
typedef struct
{
    /* ---- 采样缓冲区 ---- */
    float32_t pos[GRAVITY_SAMPLE_MAX]; /**< 各采样点关节角度 (rad)       */
    float32_t tor[GRAVITY_SAMPLE_MAX]; /**< 各采样点关节力矩 (N·m)       */
    int8_t    n_sample;                /**< 当前已采集样本数              */

    /* ---- 辨识结果 ---- */
    float32_t theta[2]; /**< [θ₁, θ₂]：τ = θ₁·cos + θ₂·sin */
    int8_t    ready;    /**< 0: 未就绪，1: 辨识完成          */
} STR_GravityIdent;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  初始化重力辨识句柄
 * @param  ident  句柄指针
 * @retval void
 */
void DATA_GravityInit(STR_GravityIdent* ident);

/**
 * @brief  重置辨识句柄（清空缓冲区与结果）
 * @param  ident  句柄指针
 * @retval void
 */
void DATA_GravityReset(STR_GravityIdent* ident);

/**
 * @brief  推送一帧静止测量数据
 * @param  ident  句柄指针
 * @param  pos    当前关节角度 (rad)
 * @param  tor    当前关节力矩 (N·m)，电机静止时反馈
 * @retval 当前已采集样本数；缓冲区满时返回 GRAVITY_SAMPLE_MAX
 */
int8_t DATA_GravityPushSample(STR_GravityIdent* ident, float32_t pos, float32_t tor);

/**
 * @brief  对已采集的样本执行最小二乘辨识
 * @param  ident  句柄指针
 * @retval  0: 成功，-1: 失败（样本不足或矩阵奇异）
 * @note   样本数 ≥ 3 时方可求解
 */
int8_t DATA_GravitySolve(STR_GravityIdent* ident);

/**
 * @brief  根据辨识参数计算当前位置的重力前馈力矩
 * @param  theta    辨识参数数组 [θ₁, θ₂]
 * @param  pos_rad  当前关节角度 (rad)
 * @retval 重力前馈力矩 (N·m)
 */
float32_t DATA_GravityCalcFF(const float32_t* theta, float32_t pos_rad);

#ifdef __cplusplus
}
#endif

#endif /* ALG_GRAVITY_H */
