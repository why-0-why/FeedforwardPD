/**
******************************************************************************
 * @file           : alg_inertia.h
 * @author         : WHY
 * @date           : 2026-4-14
 * @brief          : alg_inertia.c 的头文件
 *                   包含单关节惯量辨识（最小二乘法）的结构体与接口声明
 *
 * 辨识模型：τ - τ_ff = J·α + b·ω
 *   τ      : 关节力矩测量值（电机反馈）
 *   τ_ff   : 已知重力前馈力矩
 *   J      : 待辨识等效转动惯量 (kg·m²)
 *   b      : 待辨识粘滞摩擦系数 (N·m·s/rad)
 *   α      : 关节角加速度（微分估算）
 *   ω      : 关节角速度（电机反馈）
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ALG_INERTIA_H
#define ALG_INERTIA_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "arm_math.h"
/* Exported defines ----------------------------------------------------------*/

/** 辨识样本最大容量；按需修改以平衡内存与精度 */
#define INERTIA_SAMPLE_MAX  64

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 单关节惯量辨识句柄
 *
 * 维护采样缓冲区，并在缓冲区满时调用最小二乘求解辨识结果。
 * 所有字段均由 DATA_InertiaInit() 初始化后内部管理，外部只读辨识结果。
 */
typedef struct
{
    /* ---- 采样缓冲区 ---- */
    float32_t X[INERTIA_SAMPLE_MAX * 2]; /**< 回归矩阵 [α₀ ω₀; α₁ ω₁; …]，行优先 */
    float32_t Y[INERTIA_SAMPLE_MAX];     /**< 目标向量 τ - τ_ff                      */
    int8_t    n_sample;                  /**< 当前已采集样本数                        */

    /* ---- 辨识结果 ---- */
    float32_t J_est; /**< 估计等效转动惯量 (kg·m²)            */
    float32_t b_est; /**< 估计粘滞摩擦系数 (N·m·s/rad)        */
    int8_t    ready; /**< 0: 未就绪，1: 辨识完成              */

    /* ---- 微分估算内部状态 ---- */
    float32_t vel_prev;  /**< 上一拍角速度，用于差分估算角加速度  */
    float32_t dt_s;      /**< 采样周期 (s)                        */
} STR_InertiaIdent;

/* Exported variables --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  初始化惯量辨识句柄
 * @param  ident  句柄指针
 * @param  dt_s   采样周期 (s)，用于差分估算角加速度
 * @retval void
 */
void DATA_InertiaInit(STR_InertiaIdent* ident, float32_t dt_s);

/**
 * @brief  重置辨识句柄（清空缓冲区，保留采样周期设置）
 * @param  ident  句柄指针
 * @retval void
 */
void DATA_InertiaReset(STR_InertiaIdent* ident);

/**
 * @brief  推送一帧测量数据并累积样本
 * @param  ident   句柄指针
 * @param  vel     当前角速度 (rad/s)，由电机反馈获取
 * @param  tor     当前关节力矩 (N·m)，由电机反馈获取
 * @param  tor_ff  重力前馈力矩 (N·m)，由已辨识重力参数计算
 * @retval 当前已采集样本数；达到 INERTIA_SAMPLE_MAX 时触发求解
 */
int8_t DATA_InertiaPushSample(STR_InertiaIdent* ident,
                              float32_t vel,
                              float32_t tor,
                              float32_t tor_ff);

/**
 * @brief  对已采集的样本执行最小二乘辨识
 * @param  ident  句柄指针
 * @retval  0: 成功，-1: 失败（样本不足或矩阵奇异）
 * @note   样本数 ≥ 2 时方可求解；通常在 DATA_InertiaPushSample() 内部自动调用
 */
int8_t DATA_InertiaSolve(STR_InertiaIdent* ident);

#ifdef __cplusplus
}
#endif

#endif /* ALG_INERTIA_H */
