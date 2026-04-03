/**
 ******************************************************************************
 * @file           : task_start.c
 * @author         : WHY
 * @date           : 2026-4-1
 * @brief          : 任务层：启动任务，执行初始化硬件和启动任务
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
#include "alg_OLS.h"
#include "arm_math.h"
#include "drv_dmMotor.h"
#include "task_start.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
osThreadId h_TaskIdentify;
/* Exported variables --------------------------------------------------------*/
extern STR_dmMotor s_dmMotor1;
extern ENU_Mode e_mode;
/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/


void TASK_IdentifyInit()
{
    /* 定义检测任务属性 */
    osThreadDef(TaskIdentify, TASK_Identify, osPriorityNormal, 0, 256);
    /* 创建检测任务 */
    h_TaskIdentify = osThreadCreate(osThread(TaskIdentify), NULL);
}
float32_t RadMeas[6]={
    -0.503356934,
    -0.576981544,
    -0.631151199,
    0.63000679,
    0.560578346,
    0.501449585};
float32_t sindata[6]={0};
float32_t cosdata[6]={0};
float32_t cossin[6*2]={0};
float32_t t[6]={
    0.0219783783,
    0.173381805,
    0.109889984,
    -0.11965847,
    -0.0170936584,
    -0.0268621445};
float32_t mgl[2]={0,0};
float32_t mgl_est =0;
float32_t alpha_est_rad=0;
float32_t alpha_est_deg=0;

float32_t motorposSin=0;
float32_t motorposCos=0;

float32_t ff=0;
/**
 * @brief          任务-初始化硬件和启动其他任务
 * @param[in]      void
 * @return         void
 */
void TASK_Identify(void const* argument)
{
    uint32_t period = osKernelSysTick();

    for (int i=0;i<6;i++)
    {
        sindata[i]=sinf(RadMeas[i]);
        cosdata[i]=cosf(RadMeas[i]);
    }

    for (int i=0;i<6;i++)
    {
            cossin[i*2]=cosdata[i];
            cossin[2*i+1]=sindata[i];
    }
    DATA_OLS(6,2,cossin,t,mgl);

    //反算
    mgl_est = sqrt(mgl[0] * mgl[0] + mgl[1] * mgl[1]);
    alpha_est_rad = atan2(mgl[1], mgl[0]);  // 注意顺序: atan2(b, a)
    alpha_est_deg = alpha_est_rad * 180.0 / M_PI;
    for (;;)
    {
        switch (e_mode)
        {
        case SAFETY:
            s_dmMotor1.ctrl.tor_set=0;
            break;
        case NORMAL:
            motorposCos=cosf(s_dmMotor1.ctrl.pos_set);
            motorposSin=sinf(s_dmMotor1.ctrl.pos_set);

            ff=mgl[0]*motorposCos+mgl[1]*motorposSin;
            s_dmMotor1.ctrl.tor_set=ff;
        }
        osDelayUntil(&period, 10); //100hz
    }
}

