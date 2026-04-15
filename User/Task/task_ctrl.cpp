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
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
osThreadId h_TaskCTRL;
static STR_LowPass s_lowPassFilter;
static STR_CubicCoeffs s_cubicCoeffs;
static uint32_t i_period=4000;
float sinpos[3]={0};//单位rad,rad/s,rad/s^2
/* Exported variables --------------------------------------------------------*/
extern STR_dmMotor s_dmMotor1;
extern ENU_Mode e_mode;
float32_t a_theta[2] = {0.0971329957, -0.318323165}; //测量得到的参数
float32_t J=0.000139336596;
float32_t B=0.0662943497;
float32_t ff;
uint8_t i_ffable=0;
/* Private function prototypes -----------------------------------------------*/
static void TASK_ModeSwitch();
static void TASK_SafetyMode();
static void TASK_NormalMode();
static void TASK_FollowMode();
static float rand_curve();
static void sin_curve(float t);
/* Private user code ---------------------------------------------------------*/


void TASK_CTRLInit()
{
    /* 定义检测任务属性 */
    osThreadDef(TaskCTRL, TASK_CTRL, osPriorityNormal, 0, 256);
    /* 创建检测任务 */
    h_TaskCTRL = osThreadCreate(osThread(TaskCTRL), NULL);
}

/**
 * @brief          任务-初始化硬件和启动其他任务
 * @param[in]      void
 * @return         void
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
            float32_t f_posSin = 0;
            float32_t f_posCos = 0;
            f_posCos = cosf(s_dmMotor1.para.pos_rad);
            f_posSin = sinf(s_dmMotor1.para.pos_rad);

            ff = a_theta[0] * f_posCos + a_theta[1] * f_posSin;//重力补偿
            break;
        }
        if (e_mode==NORMAL||e_mode==FOLLOW)
            osDelayUntil(&period, 10); //100hz
        else if (e_mode==IDENTIFY)
            osDelayUntil(&period, 1); //1khz
    }
}

void TASK_ModeSwitch()
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
            DATA_CalcCubicCoeffs(0,M_PI, 0, 0, i_period/2, &s_cubicCoeffs);
            CTRL_dmMotorEnable(&hcan1, &s_dmMotor1);
            break;
        }
    }
}


void TASK_SafetyMode()
{
    DATA_dmMotorInitCmdCtrl(&s_dmMotor1);
    CTRL_dmMotorDisable(&hcan1, &s_dmMotor1);
}

void TASK_NormalMode()
{
    static uint8_t iKeyBuf = 1;
    if (iKeyBuf != HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)) //按键切换失能使能
    {
        iKeyBuf = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
        if (!iKeyBuf)
        {
            if (s_dmMotor1.ctrl.kp_set > 0)
            {
                s_dmMotor1.ctrl.kp_set = 0;
                s_dmMotor1.ctrl.kd_set = 0;
                s_dmMotor1.ctrl.pos_set = s_dmMotor1.para.pos_rad;
                CTRL_dmMotorEnable(&hcan1, &s_dmMotor1);
            }
            else
            {
                s_dmMotor1.ctrl.kp_set = 100;
                s_dmMotor1.ctrl.kd_set = 0.5;
                s_dmMotor1.ctrl.pos_set = s_dmMotor1.para.pos_rad;
                CTRL_dmMotorEnable(&hcan1, &s_dmMotor1);
            }
        }
    }
    CTRL_dmMotorCtrl(&hcan1, &s_dmMotor1);
}

void TASK_FollowMode()
{
    sin_curve(HAL_GetTick()/1000.0);
    s_dmMotor1.ctrl.pos_set = sinpos[0];
    s_dmMotor1.ctrl.vel_set = 3*sinpos[1];
    float32_t f_posSin = 0;
    float32_t f_posCos = 0;
    f_posCos = cosf(s_dmMotor1.para.pos_rad);
    f_posSin = sinf(s_dmMotor1.para.pos_rad);

    if (i_ffable==2)
    {
        ff = a_theta[0] * f_posCos + a_theta[1] * f_posSin;//重力补偿
        ff +=J*sinpos[2]+B*sinpos[1];//惯量补偿
    }else if (i_ffable==1)
    {
        ff = a_theta[0] * f_posCos + a_theta[1] * f_posSin;//重力补偿
    }
    else
    {
        ff=0;
    }
    s_dmMotor1.ctrl.tor_set = ff;

    CTRL_dmMotorCtrl(&hcan1, &s_dmMotor1);
}

/**
 *
 * @return 随机平滑变换位置曲线
 */
float rand_curve()
{
    static float pos = 0;

    float increment = ((float)rand() / RAND_MAX) * (0.01 - (-0.01)) - 0.01; // 生成(-0.5, 0.5)的随机增量
    pos += increment;

    // 限幅到 [-2π, 2π]
    if (pos > 2.0 * M_PI)
        pos = 2.0 * M_PI;
    else if (pos < -2.0 * M_PI)
        pos = -2.0 * M_PI;
    return DATA_LowpassUpdate(&s_lowPassFilter, pos);
}

/**
 * @param t 时间
 * @return 按时间运行的正弦曲线
 */
void sin_curve(float t)
{
    static float omega=2*M_PI/3;//sin曲线角速度
    static float amp=M_PI;//幅值

    sinpos[0]=amp*sinf(omega*t);
    sinpos[1]=amp*omega*cosf(omega*t);
    sinpos[2]=-amp*omega*omega*sinf(omega*t);
}