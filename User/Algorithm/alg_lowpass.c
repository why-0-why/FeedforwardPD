
/**
 ******************************************************************************
 * @file           : alg_lowpass.c
 * @author         : WHY
 * @date           : 2026-4-14
 * @brief          : XXX的驱动模块
 *
 * 本文件提供了离散低通滤波器的初始化函数和更新函数
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "alg_lowpass.h"

/* Private includes ----------------------------------------------------------*/
#include <math.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

void DATA_LowpassInit(STR_LowPass *filter, float fc, float fs) {
    // 计算归一化截止角频率
    float wc = 2.0f * M_PI * fc / fs;
    // 一阶低通滤波器的系数（指数平滑法）
    filter->alpha = 1.0f - expf(-wc);
    filter->y_prev = 0.0f;
}

void DATA_LowpassSet(STR_LowPass *filter, float alpha,float y_prev)
{
    filter->alpha  = alpha;
    filter->y_prev = y_prev;
}

float DATA_LowpassUpdate(STR_LowPass *filter, float x) {
    // y[n] = α * x[n] + (1-α) * y[n-1]
    float y = filter->alpha * x + (1.0f - filter->alpha) * filter->y_prev;
    filter->y_prev = y;
    return y;
}