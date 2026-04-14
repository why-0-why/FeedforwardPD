/**
 ******************************************************************************
 * @file           : CubicFit.c
 * @author         : WHY
 * @date           : 2026-04-10
 * @brief          : 三次多项式拟合驱动模块
 *
 * 本文件提供了三次多项式插值（Hermite插值）相关函数：
 *     - DATA_CalcCubicCoeffs(): 根据起点/终点位置和速度计算多项式系数
 *     - DATA_EvalCubicPosition(): 根据系数计算给定时刻的位置
 *     - DATA_CubicInterpolate(): 组合上述两步，直接返回插值结果
 ******************************************************************************
 * @attention
 * 时间单位统一为毫秒(ms)，速度单位应为位置/毫秒，以保证量纲一致。
 * 若速度单位为位置/秒，请先将时间转换为秒后再调用。
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "alg_cubicfit.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void calc_coeffs(float f_q0, float f_q1, float f_v0, float f_v1,
                        float f_T, STR_CubicCoeffs *p_coeffs);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief          内部计算三次多项式系数（使用浮点时间）
 * @param[in]      f_q0       起点位置
 * @param[in]      f_q1       终点位置
 * @param[in]      f_v0       起点速度
 * @param[in]      f_v1       终点速度
 * @param[in]      f_T        总时间（浮点数）
 * @param[out]     p_coeffs   系数结构体指针
 * @return         void
 */
static void calc_coeffs(float f_q0, float f_q1, float f_v0, float f_v1,
                        float f_T, STR_CubicCoeffs *p_coeffs)
{
    float f_T2 = f_T * f_T;      // T^2
    float f_T3 = f_T2 * f_T;     // T^3

    // 解边界条件方程组
    // a0 = q0
    // a1 = v0
    // a2 = (3*(q1 - q0 - v0*T) - (v1 - v0)*T) / T^2
    // a3 = ((v1 - v0)*T - 2*(q1 - q0 - v0*T)) / T^3

    float f_dq = f_q1 - f_q0;           // 位置差
    float f_dv = f_v1 - f_v0;           // 速度差

    p_coeffs->f_a0 = f_q0;
    p_coeffs->f_a1 = f_v0;
    p_coeffs->f_a2 = (3.0f * (f_dq - f_v0 * f_T) - f_dv * f_T) / f_T2;
    p_coeffs->f_a3 = (f_dv * f_T - 2.0f * (f_dq - f_v0 * f_T)) / f_T3;
}

/* Exported functions --------------------------------------------------------*/

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
                          int i_T_ms, STR_CubicCoeffs *p_coeffs)
{
    // 将整数毫秒时间转换为浮点数，用于系数计算
    float f_T = (float)i_T_ms;
    calc_coeffs(f_q0, f_q1, f_v0, f_v1, f_T, p_coeffs);
}

/**
 * @brief          数据：根据三次多项式系数计算给定时刻的位置
 * @param[in]      p_coeffs   指向系数结构体的指针
 * @param[in]      i_t_ms     插值时刻（单位：ms，范围 [0, i_T_ms]）
 * @return         float      当前位置
 */
float DATA_EvalCubicPosition(const STR_CubicCoeffs *p_coeffs, int i_t_ms)
{
    float f_t = (float)i_t_ms;
    float f_t2 = f_t * f_t;
    float f_t3 = f_t2 * f_t;

    // q(t) = a0 + a1*t + a2*t^2 + a3*t^3
    return p_coeffs->f_a0 + p_coeffs->f_a1 * f_t +
           p_coeffs->f_a2 * f_t2 + p_coeffs->f_a3 * f_t3;
}

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
                            int i_T_ms, int i_t_ms)
{
    STR_CubicCoeffs s_coeffs;
    DATA_CalcCubicCoeffs(f_q0, f_q1, f_v0, f_v1, i_T_ms, &s_coeffs);
    return DATA_EvalCubicPosition(&s_coeffs, i_t_ms);
}