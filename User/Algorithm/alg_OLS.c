/**
 ******************************************************************************
 * @file           : alg_OLS.c
 * @author         : WHY
 * @date           : 2026-4-3
 * @brief          : 最小二乘求解模块（正规方程 + 高斯消元）
 *
 * 提供接口 DATA_OLS() 用于求解线性最小二乘问题 Y = X * theta
 ******************************************************************************
 * @attention
 *    - 要求 X 列满秩，否则矩阵奇异返回 -1
 *    - 使用正规方程 X^T X theta = X^T Y，内部采用列主元高斯消元
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "alg_OLS.h"
#include <stdlib.h>

#include <string.h>

/* Private function prototypes -----------------------------------------------*/
static void compute_xtx_xty(int8_t n, int8_t m, const float32_t* X, const float32_t* Y,
                            float32_t* XtX, float32_t* XtY);
static int8_t gauss_elimination(int8_t m, float32_t* A, float32_t* b, float32_t* x);
static int8_t solve_normal_equations(int8_t m, const float32_t* XtX, const float32_t* XtY,
                                  float32_t* theta);

/* Public functions ----------------------------------------------------------*/

/**
 * @brief 最小二乘求解主函数
 * @param n     样本数量（X 的行数）
 * @param m     自变量个数（X 的列数，theta 的长度）
 * @param X     输入矩阵，大小为 n×m，按行存储 (X[i*m + j])
 * @param Y     观测向量，长度为 n
 * @param theta 输出参数，求解得到的系数向量，长度为 m（调用者分配空间）
 * @return      0 成功，-1 失败（内存不足或矩阵奇异）
 */
int8_t DATA_OLS(int8_t n, int8_t m, const float32_t* X, const float32_t* Y, float32_t* theta)
{
    // 分配正规方程系数矩阵 XtX (m×m) 和右端向量 XtY (m×1)
    float32_t* XtX = (float32_t*)malloc(m * m * sizeof(float32_t));
    float32_t* XtY = (float32_t*)malloc(m * sizeof(float32_t));
    float32_t* A = (float32_t*)malloc(m * sizeof(float32_t));

    if (!XtX || !XtY || !A)
    {
        // 释放已分配的内存，避免泄漏
        free(XtX);
        free(XtY);
        free(A);
        return -1;
    }

    // 计算正规方程系数和右端项
    compute_xtx_xty(n, m, X, Y, XtX, XtY);

    // 求解正规方程 (XtX) * theta = XtY
    int8_t ret = solve_normal_equations(m, XtX, XtY, A);

    if (ret == 0)
    {
        // 成功：复制结果到输出参数 theta
        memcpy(theta, A, m * sizeof(float32_t));
    }

    // 释放临时内存
    free(XtX);
    free(XtY);
    free(A);
    return ret;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 计算正规方程的系数矩阵 X^T X 和右端向量 X^T Y
 * @param n   样本数
 * @param m   变量数
 * @param X   输入矩阵 (n×m)
 * @param Y   观测向量 (n)
 * @param XtX 输出矩阵 (m×m)，调用者需保证已分配空间，函数内会清零
 * @param XtY 输出向量 (m)，调用者需保证已分配空间，函数内会清零
 */
static void compute_xtx_xty(int8_t n, int8_t m, const float32_t* X, const float32_t* Y,
                            float32_t* XtX, float32_t* XtY)
{
    // 重要：先将输出数组清零，避免残留垃圾值
    for (int8_t i = 0; i < m * m; ++i) XtX[i] = 0.0;
    for (int8_t i = 0; i < m; ++i) XtY[i] = 0.0;

    // 遍历每个样本，累加外积
    for (int8_t i = 0; i < n; ++i)
    {
        const float32_t* row = X + i * m; // 第 i 个样本的自变量向量
        float32_t yi = Y[i];

        // 更新 XtY: XtY[j] += X[i][j] * Y[i]
        for (int8_t j = 0; j < m; ++j)
        {
            XtY[j] += row[j] * yi;
        }

        // 更新 XtX: XtX[j][k] += X[i][j] * X[i][k]
        for (int8_t j = 0; j < m; ++j)
        {
            float32_t xj = row[j];
            for (int8_t k = 0; k < m; ++k)
            {
                XtX[j * m + k] += xj * row[k];
            }
        }
    }
}

/**
 * @brief 列主元高斯消元法求解线性方程组 A x = b
 * @param m   方程阶数
 * @param A   系数矩阵 (m×m)，会被修改
 * @param b   右端向量 (m)，会被修改
 * @param x   解向量 (m)，调用者分配空间
 * @return    0 成功，-1 矩阵奇异
 */
static int8_t gauss_elimination(int8_t m, float32_t* A, float32_t* b, float32_t* x)
{
    int8_t* perm = (int8_t*)malloc(m * sizeof(int8_t));
    if (!perm) return -1;

    for (int8_t i = 0; i < m; ++i) perm[i] = i;

    // 前向消元（选主元）
    for (int8_t k = 0; k < m; ++k)
    {
        // 选主元：第 k 列绝对值最大的行
        int8_t max_row = k;
        float32_t max_val = fabs(A[k * m + k]);
        for (int8_t i = k + 1; i < m; ++i)
        {
            float32_t val = fabs(A[i * m + k]);
            if (val > max_val)
            {
                max_val = val;
                max_row = i;
            }
        }
        if (max_val < 1e-12)
        {
            // 奇异矩阵
            free(perm);
            return -1;
        }

        // 交换当前行与主元行（系数矩阵和右端向量）
        if (max_row != k)
        {
            for (int8_t j = 0; j < m; ++j)
            {
                float32_t tmp = A[k * m + j];
                A[k * m + j] = A[max_row * m + j];
                A[max_row * m + j] = tmp;
            }
            float32_t tmp = b[k];
            b[k] = b[max_row];
            b[max_row] = tmp;
            // 置换记录（本例未使用，但保留）
            int8_t tmp_p = perm[k];
            perm[k] = perm[max_row];
            perm[max_row] = tmp_p;
        }

        // 消元
        float32_t pivot = A[k * m + k];
        for (int8_t i = k + 1; i < m; ++i)
        {
            float32_t factor = A[i * m + k] / pivot;
            for (int8_t j = k; j < m; ++j)
            {
                A[i * m + j] -= factor * A[k * m + j];
            }
            b[i] -= factor * b[k];
        }
    }

    // 回代求解
    for (int8_t i = m - 1; i >= 0; --i)
    {
        float32_t sum = 0.0;
        for (int8_t j = i + 1; j < m; ++j)
        {
            sum += A[i * m + j] * x[j];
        }
        x[i] = (b[i] - sum) / A[i * m + i];
    }

    free(perm);
    return 0;
}

/**
 * @brief 求解正规方程 (X^T X) * theta = X^T Y
 * @param m     变量个数
 * @param XtX   系数矩阵 X^T X (m×m)，不会被修改
 * @param XtY   右端向量 X^T Y (m)
 * @param theta 输出解向量 (m)
 * @return      0 成功，-1 求解失败（矩阵奇异或内存不足）
 */
static int8_t solve_normal_equations(int8_t m, const float32_t* XtX, const float32_t* XtY,
                                  float32_t* theta)
{
    // 拷贝一份数据，因为高斯消元会修改原矩阵和向量
    float32_t* A_copy = (float32_t*)malloc(m * m * sizeof(float32_t));
    float32_t* b_copy = (float32_t*)malloc(m * sizeof(float32_t));
    if (!A_copy || !b_copy)
    {
        free(A_copy);
        free(b_copy);
        return -1;
    }

    for (int8_t i = 0; i < m * m; ++i) A_copy[i] = XtX[i];
    for (int8_t i = 0; i < m; ++i) b_copy[i] = XtY[i];

    int8_t ret = gauss_elimination(m, A_copy, b_copy, theta);

    free(A_copy);
    free(b_copy);
    return ret;
}
