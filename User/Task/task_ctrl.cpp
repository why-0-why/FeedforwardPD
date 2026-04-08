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
#include "cmsis_os.h"
#include "drv_dmMotor.h"
#include "task_start.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
osThreadId h_TaskCTRL;
/* Exported variables --------------------------------------------------------*/
extern STR_dmMotor s_dmMotor1;
extern ENU_Mode e_mode;
/* Private function prototypes -----------------------------------------------*/
static void TASK_ModeSwitch();
static void TASK_SafetyMode();
static void TASK_NormalMode();
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
        default:
            break;
        }
        osDelayUntil(&period, 10); //100hz
    }
}

void TASK_ModeSwitch()
{
    static ENU_Mode e_modeBuf = SAFETY;
    if (e_modeBuf!=e_mode)
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