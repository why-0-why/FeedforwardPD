/**
 ******************************************************************************
 * @file           : task_ctrl.c
 * @author         : WHY
 * @date           : 2026-4-3
 * @brief          : 任务层：控制任务，周期执行硬件控制
 *
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "task_ctrl.h"
/* Private includes ----------------------------------------------------------*/
#include "main.h"
#include "arm_math.h"
#include "cmsis_os.h"
#include "dvc_dmMotor.h"
#include "task_start.h"
#include "alg_lowpass.h"
#include "alg_cubicfit.h"
#include "alg_gravity.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
osThreadId h_TaskCTRL;
static STR_LowPass    s_lowPassFilter;
static STR_CubicCoeffs s_cubicCoeffs;
static uint32_t u_followPeriod = 4000;
static float32_t a_sinPos[3] = {0}; /**< 正弦曲线：[pos(rad), vel(rad/s), acc(rad/s²)] */

/* Exported variables --------------------------------------------------------*/
extern STR_dmMotor s_dmMotor1;
extern ENU_Mode    e_mode;
float32_t a_estG[2]       = {0.0971329957f, -0.318323165f}; /**< 重力辨识参数 [θ₁, θ₂] */
float32_t f_estJ          = 0.000139336596f;                 /**< 辨识等效转动惯量 (kg·m²) */
float32_t f_estB          = 0.0662943497f;                   /**< 辨识粘滞摩擦系数 (N·m·s/rad) */
float32_t f_feedforwardTor = 0.0f;                           /**< 当前前馈力矩 (N·m) */
uint8_t   u_ffMode        = 0;                               /**< 前馈模式：0=无, 1=重力, 2=重力+惯量 */

/* Private function prototypes -----------------------------------------------*/
static void TASK_ModeSwitch();
static void TASK_SafetyMode();
static void TASK_NormalMode();
static void TASK_FollowMode();
static float32_t CALC_RandCurve();
static void CALC_SinCurve(float32_t t);

/* Private user code ---------------------------------------------------------*/

void TASK_CTRLInit()
{
    CTRL_dmMotorEnable(&hcan1, &s_dmMotor1);
    osThreadDef(TaskCTRL, TASK_CTRL, osPriorityNormal, 0, 256);
    h_TaskCTRL = osThreadCreate(osThread(TaskCTRL), NULL);
}

/**
 * @brief  任务-控制（周期执行）
 */
void TASK_CTRL(void const* argument)
{
    uint32_t period = osKernelSysTick();
    DATA_LowpassInit(&s_lowPassFilter, 1, 100);
    for (;;)
    {
        TASK_ModeSwitch();
        switch (e_mode)
        {
        case SAFETY:
            TASK_SafetyMode();
            break;
        case NORMAL:
            TASK_NormalMode();
            break;
        case FOLLOW:
            TASK_FollowMode();
            break;
        case IDENTIFY:
            f_feedforwardTor = DATA_GravityCalcFF(a_estG, s_dmMotor1.para.pos_rad);
            break;
        }
        if (e_mode == NORMAL || e_mode == FOLLOW)
            osDelayUntil(&period, 10); /* 100 Hz */
        else if (e_mode == IDENTIFY)
            osDelayUntil(&period, 1);  /* 1 kHz  */
    }
}

static void TASK_ModeSwitch()
{
    static ENU_Mode e_modeBuf = SAFETY;
    if (e_modeBuf != e_mode)
    {
        e_modeBuf = e_mode;
        switch (e_modeBuf)
        {
        case SAFETY:
            DATA_dmMotorInitCmdCtrl(&s_dmMotor1);
            CTRL_dmMotorDisable(&hcan1, &s_dmMotor1);
            break;
        case NORMAL:
            CTRL_dmMotorEnable(&hcan1, &s_dmMotor1);
            break;
        case FOLLOW:
            DATA_CalcCubicCoeffs(0, M_PI, 0, 0, u_followPeriod / 2, &s_cubicCoeffs);
            CTRL_dmMotorEnable(&hcan1, &s_dmMotor1);
            break;
        default:
            break;
        }
    }
}

static void TASK_SafetyMode()
{
    DATA_dmMotorInitCmdCtrl(&s_dmMotor1);
    CTRL_dmMotorDisable(&hcan1, &s_dmMotor1);
}

static void TASK_NormalMode()
{
    static uint8_t u_keyBuf = 1;
    if (u_keyBuf != HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
    {
        u_keyBuf = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
        if (!u_keyBuf)
        {
            if (s_dmMotor1.ctrl.kp_set > 0)
            {
                s_dmMotor1.ctrl.kp_set  = 0;
                s_dmMotor1.ctrl.kd_set  = 0;
            }
            else
            {
                s_dmMotor1.ctrl.kp_set  = 100;
                s_dmMotor1.ctrl.kd_set  = 0.5;
            }
            s_dmMotor1.ctrl.pos_set = s_dmMotor1.para.pos_rad;
            CTRL_dmMotorEnable(&hcan1, &s_dmMotor1);
        }
    }
    f_feedforwardTor  = DATA_GravityCalcFF(a_estG, s_dmMotor1.para.pos_rad);
    f_feedforwardTor += f_estB * s_dmMotor1.para.vel / 3.0f ; /* 速度补偿 */
    s_dmMotor1.ctrl.tor_set = f_feedforwardTor;
    CTRL_dmMotorCtrl(&hcan1, &s_dmMotor1);
}

static void TASK_FollowMode()
{
    CALC_SinCurve(HAL_GetTick() / 1000.0f);
    s_dmMotor1.ctrl.pos_set = a_sinPos[0];
    s_dmMotor1.ctrl.vel_set = 3.0f * a_sinPos[1];

    if (u_ffMode == 2)
    {
        f_feedforwardTor  = DATA_GravityCalcFF(a_estG, s_dmMotor1.para.pos_rad);
        f_feedforwardTor += f_estJ * a_sinPos[2] + f_estB * a_sinPos[1]; /* 惯量补偿 */
    }
    else if (u_ffMode == 1)
    {
        f_feedforwardTor = DATA_GravityCalcFF(a_estG, s_dmMotor1.para.pos_rad);
    }
    else
    {
        f_feedforwardTor = 0.0f;
    }
    s_dmMotor1.ctrl.tor_set = f_feedforwardTor;
    CTRL_dmMotorCtrl(&hcan1, &s_dmMotor1);
}

/**
 * @brief  生成随机平滑位置曲线
 * @retval 当前目标位置 (rad)
 */
static float32_t CALC_RandCurve()
{
    static float32_t f_pos = 0.0f;
    float32_t f_inc = ((float32_t)rand() / RAND_MAX) * 0.02f - 0.01f;
    f_pos += f_inc;
    if (f_pos >  2.0f * M_PI) f_pos =  2.0f * M_PI;
    if (f_pos < -2.0f * M_PI) f_pos = -2.0f * M_PI;
    return DATA_LowpassUpdate(&s_lowPassFilter, f_pos);
}

/**
 * @brief  生成正弦激励曲线，结果写入 a_sinPos
 * @param  t  时间 (s)
 */
static void CALC_SinCurve(float32_t t)
{
    static const float32_t f_omega = 2.0f * M_PI / 3.0f;
    static const float32_t f_amp   = M_PI;
    a_sinPos[0] =  f_amp * sinf(f_omega * t);
    a_sinPos[1] =  f_amp * f_omega * cosf(f_omega * t);
    a_sinPos[2] = -f_amp * f_omega * f_omega * sinf(f_omega * t);
}
