/**
 ******************************************************************************
 * @file           : task_identify.cpp
 * @author         : WHY
 * @date           : 2026-4-26
 * @brief          : 任务层：辨识任务
 *
 * 辨识流程（顺序执行）：
 *   1. 重力辨识（G_Identifying → G_Ready）
 *      在 NUM_G_SAMPLES+1 个静止位置依次采集 (pos, tor)，
 *      调用 alg_gravity 最小二乘求解 τ = θ₁·cos(pos) + θ₂·sin(pos)
 *
 *   2. 惯量辨识（J_Identifying → J_Ready）
 *      施加正弦激励，去除重力前馈后调用 alg_inertia 最小二乘求解
 *      τ - τ_ff = J·α + b·ω
 *
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "task_identify.h"
/* Private includes ----------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "dvc_dmMotor.h"
#include "task_start.h"
#include "task_ctrl.h"
#include "alg_gravity.h"
#include "alg_inertia.h"
#include "alg_lowpass.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define NUM_G_SAMPLES    10                  /**< 重力辨识采样点数（实际采 NUM_G_SAMPLES+1 个点） */
#define NUM_J_SAMPLES    INERTIA_SAMPLE_MAX  /**< 惯量辨识采样点数 */
#define G_SETTLE_MS      1500u               /**< 每个重力采样点的稳定等待时间 (ms) */
#define J_SAMPLE_PERIOD  75u                 /**< 惯量采样间隔 (ms，以 1ms 循环计) */

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
osThreadId h_TaskIdentify;
ENU_IdentifyState e_IdentifyState = G_Identifying;

static STR_GravityIdent s_gravityIdent;
static STR_InertiaIdent s_inertiaIdent;
static STR_LowPass      s_posLowpass;

static const float32_t f_startPos = -60.0f * M_PI / 180.0f;
static const float32_t f_endPos   =  60.0f * M_PI / 180.0f;

static float32_t a_sinPos[2] = {0}; /**< 正弦激励：[pos(rad), vel(rad/s)] */

/* Exported variables --------------------------------------------------------*/
extern STR_dmMotor s_dmMotor1;
extern ENU_Mode    e_mode;
extern float32_t   a_estG[2];          /**< 重力辨识参数，写入 task_ctrl.cpp */
extern float32_t   f_estJ;
extern float32_t   f_estB;
extern float32_t   f_feedforwardTor;   /**< 当前重力前馈力矩，来自 task_ctrl.cpp */

/* Private function prototypes -----------------------------------------------*/
static void CALC_SinCurve(uint32_t t);

/* Public functions ----------------------------------------------------------*/

void TASK_IdentifyInit()
{
    DATA_GravityInit(&s_gravityIdent);
    DATA_InertiaInit(&s_inertiaIdent, 0.001f);
    DATA_LowpassInit(&s_posLowpass, 1, 1000);

    if (!s_dmMotor1.enable_flag)
    {
        CTRL_dmMotorEnable(&hcan1, &s_dmMotor1);
    }
    osDelay(200);

    /* 移动到重力辨识起始位置 */
    s_dmMotor1.ctrl.pos_set = f_endPos - NUM_G_SAMPLES * (f_endPos - f_startPos) / NUM_G_SAMPLES;
    DATA_LowpassSet(&s_posLowpass, s_posLowpass.alpha,f_endPos);
    CTRL_dmMotorCtrl(&hcan1, &s_dmMotor1);

    osThreadDef(TaskIdentify, TASK_Identify, osPriorityNormal, 0, 256);
    h_TaskIdentify = osThreadCreate(osThread(TaskIdentify), NULL);
}

/**
 * @brief  任务-辨识（顺序执行：重力辨识 → 惯量辨识）
 */
void TASK_Identify(void const* argument)
{
    /* ------------------------------------------------------------------ */
    /* 阶段一：重力辨识                                                    */
    /* 依次移动到 NUM_G_SAMPLES+1 个位置，每点稳定后采集 (pos, tor)       */
    /* ------------------------------------------------------------------ */
    e_IdentifyState = G_Identifying;

    for (int8_t i = NUM_G_SAMPLES; i >= 0; i--)
    {
        /* 更新目标位置（低通滤波平顺过渡） */
        s_dmMotor1.ctrl.pos_set=DATA_LowpassUpdate(
                &s_posLowpass,
                f_endPos - i * (f_endPos - f_startPos) / NUM_G_SAMPLES);
        /* 持续发送指令，等待电机稳定 */
        for (uint32_t t = 0; t < G_SETTLE_MS; t++)
        {
            s_dmMotor1.ctrl.pos_set = DATA_LowpassUpdate(
                &s_posLowpass,
                f_endPos - i * (f_endPos - f_startPos) / NUM_G_SAMPLES);
            CTRL_dmMotorCtrl(&hcan1, &s_dmMotor1);
            osDelay(1);
        }

        /* 采集当前位置与力矩 */
        DATA_GravityPushSample(&s_gravityIdent, s_dmMotor1.para.pos_rad, s_dmMotor1.para.tor);
    }

    /* 最小二乘求解重力参数 */
    if (DATA_GravitySolve(&s_gravityIdent) == 0)
    {
        a_estG[0] = s_gravityIdent.theta[0];
        a_estG[1] = s_gravityIdent.theta[1];
        e_IdentifyState = G_Ready;
    }
    else
    {
        e_IdentifyState = Identify_Failure;
    }

    /* ------------------------------------------------------------------ */
    /* 阶段二：惯量辨识（仅在重力辨识成功后执行）                         */
    /* 施加正弦激励，按固定间隔采集 (vel, tor, tor_ff)                    */
    /* ------------------------------------------------------------------ */
    if (e_IdentifyState == G_Ready)
    {
        TASK_CTRLInit(); /* 启动控制任务，开始输出重力前馈 */

        s_dmMotor1.ctrl.pos_set = 0.0f;
        CTRL_dmMotorCtrl(&hcan1, &s_dmMotor1);
        osDelay(1000);

        e_IdentifyState = J_Identifying;

        for (uint32_t t = 0, n = 0; n < NUM_J_SAMPLES; t++)
        {
            CALC_SinCurve(t);
            s_dmMotor1.ctrl.pos_set = a_sinPos[0];
            CTRL_dmMotorCtrl(&hcan1, &s_dmMotor1);

            /* 每隔 J_SAMPLE_PERIOD ms 采集一帧 */
            if (t % J_SAMPLE_PERIOD == 0)
            {
                DATA_InertiaPushSample(&s_inertiaIdent,
                                       s_dmMotor1.para.vel / 3.0f,
                                       s_dmMotor1.para.tor,
                                       f_feedforwardTor);
                n++;
            }
            osDelay(1);
        }

        /* 最小二乘求解惯量参数 */
        if (DATA_InertiaSolve(&s_inertiaIdent) == 0)
        {
            f_estJ = s_inertiaIdent.J_est;
            f_estB = s_inertiaIdent.b_est;
            e_IdentifyState = J_Ready;
        }
        else
        {
            e_IdentifyState = Identify_Failure;
        }
    }

    /* ------------------------------------------------------------------ */
    /* 收尾                                                                */
    /* ------------------------------------------------------------------ */
    if (e_IdentifyState == Identify_Failure)
    {
        a_estG[0] = 0.0f;
        a_estG[1] = 0.0f;
    }

    osDelay(100);
    e_mode = NORMAL;
    osThreadTerminate(h_TaskIdentify);
}

/**
 * @brief  生成正弦激励曲线，结果写入 a_sinPos
 * @param  t  时间，单位 ms
 */
static void CALC_SinCurve(uint32_t t)
{
    static const float32_t f_omega = 2.0f * M_PI / 3.0f;
    static const float32_t f_amp   = M_PI;
    a_sinPos[0] = f_amp * sinf(f_omega * t / 1000.0f);
    a_sinPos[1] = f_amp * f_omega * cosf(f_omega * t / 1000.0f);
}
