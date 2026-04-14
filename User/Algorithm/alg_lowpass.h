/**
******************************************************************************
 * @file           : alg_lowpass.h
 * @author         : WHY
 * @date           : 2026-4-14
 * @brief          : alg_lowpass.c 的头文件
 *                   包含低通滤波器的结构体
 *                   低通滤波器的函数声明
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ALG_LOWPASS_H
#define ALG_LOWPASS_H

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/

typedef struct {
    float alpha;        // 滤波系数
    float y_prev;       // 上一次的输出值
} STR_LowPass;

/**
 * @brief 初始化低通滤波器
 * @param filter 滤波器结构体指针
 * @param fc 截止频率 (Hz)
 * @param fs 采样频率 (Hz)
 */
void DATA_LowpassInit(STR_LowPass *filter, float fc, float fs);

/**
 * @brief 对输入值进行滤波
 * @param filter 滤波器结构体指针
 * @param x 当前输入值
 * @return 滤波后的输出值
 */
float DATA_LowpassUpdate(STR_LowPass *filter, float x);
#ifdef __cplusplus
}
#endif
#endif // ALG_LOWPASS_H