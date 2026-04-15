/**
 ******************************************************************************
 * @file           : task_identify.cpp
 * @author         : WHY
 * @date           : 2026-4-14
 * @brief          : 任务层：惯量辨识任务
 *
 * 辨识模型（去除已知重力前馈后）：
 *     τ - τ_ff = J·α + b·ω
 *
 * 在 IDENTIFY 模式下，每毫秒推送一帧电机反馈数据；
 * 缓冲区满后自动触发最小二乘求解，结果写入 s_inertiaIdent。
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
#include "alg_inertia.h"
#include "alg_lowpass.h"
#include  "alg_OLS.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
osThreadId h_TaskIdentify;
STR_InertiaIdent s_inertiaIdent;
ENU_IdentifyState e_IdentifyState = G_Identifying;

#define NumGdata 10
int8_t i_dataCount = NumGdata;
uint64_t i_timeCount = 0;

float32_t f_startPos = -60.0f * M_PI / 180.0f;
float32_t f_endPos = 60.0f * M_PI / 180.0f;
STR_LowPass s_posLowpass;

#define NumJdata INERTIA_SAMPLE_MAX
static float sinpos[2]={0};
/* Exported variables --------------------------------------------------------*/
extern STR_dmMotor s_dmMotor1;
extern ENU_Mode e_mode;
extern float32_t a_theta[2]; //重力辨识参数，来自 task_ctrl.cpp
extern float32_t J;
extern float32_t B;
extern float32_t ff; //当前重力前馈力矩，来自 task_ctrl.cpp
/* Private function prototypes -----------------------------------------------*/
static void sin_curve(uint32_t t);

void TASK_IdentifyInit()
{
    /* 采样周期 1 ms，与 osDelay(1) 一致 */
    DATA_InertiaInit(&s_inertiaIdent, 0.001f);//创建惯量模型
    DATA_LowpassInit(&s_posLowpass,1,1000);//创建位置低通滤波器，重力辨识平顺
    if (!s_dmMotor1.enable_flag)
    {
        CTRL_dmMotorEnable(&hcan1, &s_dmMotor1);
    }
    osDelay(200); //等待使能完成
    s_dmMotor1.ctrl.pos_set = f_endPos - i_dataCount * (f_endPos - f_startPos) / NumGdata;
    CTRL_dmMotorCtrl(&hcan1, &s_dmMotor1); //回启始位置
    osDelay(1000); //起始位置多给的时间
    i_timeCount = HAL_GetTick();
    /* 定义辨识任务属性 */
    osThreadDef(TaskIdentify, TASK_Identify, osPriorityNormal, 0, 256);
    /* 创建辨识任务 */
    h_TaskIdentify = osThreadCreate(osThread(TaskIdentify), NULL);
}

/**
 * @brief          任务-惯量辨识
 * @param[in]      void
 * @return         void
 */
void TASK_Identify(void const* argument)
{
    float32_t a_posMeas_rad[NumGdata + 1] = {0};
    float32_t a_torMeas_Nm[NumGdata + 1] = {0}; //X*theta=Y中的Y
    float32_t a_X[(NumGdata + 1) * 2] = {0}; //X*theta=Y中的X
    while (e_IdentifyState != G_Ready && e_IdentifyState != Identify_Failure)
    {
        //电机目标递增，并记录位置与力
        s_dmMotor1.ctrl.pos_set = DATA_LowpassUpdate(&s_posLowpass,f_endPos - i_dataCount * (f_endPos - f_startPos) / NumGdata);
        CTRL_dmMotorCtrl(&hcan1, &s_dmMotor1);

        if (HAL_GetTick() - i_timeCount > 1500)
        {
            i_timeCount = HAL_GetTick();

            a_posMeas_rad[i_dataCount] = s_dmMotor1.para.pos_rad; //记录位置
            a_torMeas_Nm[i_dataCount] = s_dmMotor1.para.tor; //记录力
            i_dataCount--;

            if (i_dataCount < 0) //满足退出条件，最小二乘计算模型
            {
                float32_t a_posSin[NumGdata + 1] = {0};
                float32_t a_posCos[NumGdata + 1] = {0};
                for (int i = 0; i < NumGdata + 1; i++)
                {
                    a_posSin[i] = sinf(a_posMeas_rad[i]);
                    a_posCos[i] = cosf(a_posMeas_rad[i]);
                }

                for (int i = 0; i < NumGdata + 1; i++)
                {
                    a_X[i * 2] = a_posCos[i];
                    a_X[2 * i + 1] = a_posSin[i];
                }
                if (!DATA_OLS(NumGdata + 1, 2, a_X, a_torMeas_Nm, a_theta))//返回0成功
                {
                    e_IdentifyState = G_Ready;
                }
                else
                {
                    e_IdentifyState = Identify_Failure;
                }
                //反算
                // f_mglEst_Nm = sqrt(a_theta[0] * a_theta[0] + a_theta[1] * a_theta[1]);
                // f_alphaEst_rad = atan2(a_theta[1], a_theta[0]);  // 注意顺序: atan2(b, a)
                // f_alphaEst_deg = f_alphaEst_rad * 180.0 / M_PI;
            }
        }
        osDelay(1);
    }
    TASK_CTRLInit();; //前馈力矩开始
    i_dataCount = 0;
    //回到0点等待惯量辨识
    s_dmMotor1.ctrl.pos_set = 0.0f;
    CTRL_dmMotorCtrl(&hcan1, &s_dmMotor1);
    osDelay(1000);
    while (e_IdentifyState != J_Ready && e_IdentifyState != Identify_Failure) //惯量辨识
    {
        //播放逻辑
        static uint32_t i_postime = 0;
        sin_curve(i_postime);
        s_dmMotor1.ctrl.pos_set = sinpos[0];
        CTRL_dmMotorCtrl(&hcan1, &s_dmMotor1);
        i_postime++;
        //记录数据逻辑
        if (i_postime%75==0)
        {
            float32_t vel = s_dmMotor1.para.vel/3.0;//有三倍关系
            float32_t tor = s_dmMotor1.para.tor;
            float32_t tor_ff = ff; //使用 task_feedforward 实时计算的重力前馈

            DATA_InertiaPushSample(&s_inertiaIdent, vel, tor, tor_ff);
            i_dataCount++;
        }
        if (i_dataCount>=NumJdata&&!DATA_InertiaSolve(&s_inertiaIdent))
        {
            e_IdentifyState = J_Ready;
            J=s_inertiaIdent.J_est;
            B=s_inertiaIdent.b_est;
        }
        else if (i_dataCount>=NumJdata)
        {
            e_IdentifyState = Identify_Failure;
        }

        osDelay(1);
    }

    //辨识失败数据清除
    if (e_IdentifyState == Identify_Failure)
    {
        a_theta[0]=0;
        a_theta[1]=0;
    }
    osDelay(100);
    e_mode = NORMAL; //进入正常模式
    osThreadTerminate(h_TaskIdentify);//任务完成,删除本任务
}

/**
 * @param t 时间，单位ms
 * @return 按时间运行的正弦曲线
 */
void sin_curve(uint32_t t)
{
    static float omega1=2*M_PI/3;//sin曲线角速度
    static float amp1=M_PI;//幅值

    sinpos[0]=amp1*sinf(omega1*t/1000.0f);
    sinpos[1]=amp1*omega1*cosf(omega1*t/1000.0f);
}