/**
 ******************************************************************************
 * @file           : CubicFit.h
 * @author         : WHY
 * @date           : 2026-04-10
 * @brief          : 三次多项式拟合模块的头文件
 *                   包含拟合系数结构体声明及插值函数声明
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef CUBICFIT_H
#define CUBICFIT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/**
 * @brief 三次多项式系数结构体
 *        q(t) = a0 + a1*t + a2*t^2 + a3*t^3
 */
typedef struct
{
    float f_a0;  /**< 常数项系数 */
    float f_a1;  /**< 一次项系数 */
    float f_a2;  /**< 二次项系数 */
    float f_a3;  /**< 三次项系数 */
} STR_CubicCoeffs;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported defines ----------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief          数据：根据边界条件计算三次多项式系数
 * @param[in]      f_q0       起点位置
 * @param[in]      f_q1       终点位置
 * @param[in]      f_v0       起点速度
 * @param[in]      f_v1       终点速度
 * @param[in]      i_T_ms     总时间（单位：ms）
 * @param[out]     p_coeffs   指向存储系数的结构体指针
 * @return         void
 */
void DATA_CalcCubicCoeffs(float f_q0, float f_q1, float f_v0, float f_v1,
                          int i_T_ms, STR_CubicCoeffs *p_coeffs);

/**
 * @brief          数据：根据三次多项式系数计算给定时刻的位置
 * @param[in]      p_coeffs   指向系数结构体的指针
 * @param[in]      i_t_ms     插值时刻（单位：ms，范围 [0, i_T_ms]）
 * @return         float      当前位置
 */
float DATA_EvalCubicPosition(const STR_CubicCoeffs *p_coeffs, int i_t_ms);

/**
 * @brief          数据：一步完成三次多项式插值（计算系数并求值）
 * @param[in]      f_q0       起点位置
 * @param[in]      f_q1       终点位置
 * @param[in]      f_v0       起点速度
 * @param[in]      f_v1       终点速度
 * @param[in]      i_T_ms     总时间（单位：ms）
 * @param[in]      i_t_ms     插值时刻（单位：ms，范围 [0, i_T_ms]）
 * @return         float      当前位置
 */
float DATA_CubicInterpolate(float f_q0, float f_q1, float f_v0, float f_v1,
                            int i_T_ms, int i_t_ms);

#ifdef __cplusplus
}
#endif

#endif // CUBICFIT_H