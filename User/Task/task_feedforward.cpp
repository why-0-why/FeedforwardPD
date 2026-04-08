/**
 ******************************************************************************
 * @file           : task_feedforward.c
 * @author         : WHY
 * @date           : 2026-4-8
 * @brief          : 任务层：启动前馈力矩计算任务，执行初始化数据及前馈计算
 *
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "task_feedforward.h"
/* Private includes ----------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "alg_OLS.h"
#include "arm_math.h"
#include "dvc_dmMotor.h"
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


void TASK_FeedforwardInit()
{
    /* 定义检测任务属性 */
    osThreadDef(TaskIdentify, TASK_Feedforward, osPriorityAboveNormal, 0, 256);//有空就算，不得打断周期控制
    /* 创建检测任务 */
    h_TaskIdentify = osThreadCreate(osThread(TaskIdentify), NULL);
}

float32_t a_theta[2]={0,0};//测量得到的参数
// float32_t f_mglEst_Nm =0;
// float32_t f_alphaEst_rad=0;
// float32_t f_alphaEst_deg=0;


float32_t ff=0;
/**
 * @brief          任务-初始化硬件和启动其他任务
 * @param[in]      void
 * @return         void
 */
void TASK_Feedforward(void const* argument)
{
    for (;;)
    {
        switch (e_mode)
        {
        case SAFETY:
            s_dmMotor1.ctrl.tor_set=0;
            break;
        case NORMAL:
            float32_t f_posSin=0;
            float32_t f_posCos=0;
            f_posCos=cosf(s_dmMotor1.ctrl.pos_set);
            f_posSin=sinf(s_dmMotor1.ctrl.pos_set);

            ff=a_theta[0]*f_posCos+a_theta[1]*f_posSin;
            s_dmMotor1.ctrl.tor_set=ff;
        }
        //全速运算
        osDelay(1);
    }
}

