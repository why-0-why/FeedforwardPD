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
#include "task_start.h"
/* Private includes ----------------------------------------------------------*/
#include "main.h"
#include "bsp_can.h"
#include "cmsis_os.h"
#include "drv_dmMotor.h"
#include "task_identify.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
ENU_Mode e_mode = SAFETY;

OBJ_CAN o_can1;
uint8_t a_can1TxFIFOBuff[CAN_TX_FIFO_SIZE]; ///< CAN1发送FIFO缓冲区
STR_dmMotor s_dmMotor1;
/* Exported variables --------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void DATA_CAN1CallBack(uint32_t std_id, uint8_t* data, uint32_t dlc);
static void DATA_MotorInit();
static void TASK_SafetyMode();
static void TASK_NormalMode();
/* Private user code ---------------------------------------------------------*/

/**
 * @brief          任务-初始化硬件和启动其他任务
 * @param[in]      void
 * @return         void
 */
void TASK_Start(void const* argument)
{
    CTRL_CANNewObject(&o_can1, &hcan1, a_can1TxFIFOBuff, DATA_CAN1CallBack);
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);

    DATA_MotorInit();

    TASK_IdentifyInit();

    uint32_t period = osKernelSysTick();
    for (;;)
    {
        switch (e_mode)
        {
        case SAFETY:
            TASK_SafetyMode();
            break;
        case NORMAL:
            TASK_NormalMode();
            break;
        }

        osDelayUntil(&period, 10); //100hz
    }
}

/**
 * @brief 回调：CAN1
 *
 *
 */
void DATA_CAN1CallBack(uint32_t std_id, uint8_t* data, uint32_t dlc)
{
    switch (std_id)
    {
    case MST_ID:
        DATA_dmMortorFbdata(&s_dmMotor1, data);
        break;
    default:
        break;
    }
}

/**
 * @brief 数据：电机初始化
 *
 */
void DATA_MotorInit()
{
    s_dmMotor1.id = 1;
    s_dmMotor1.ctrl.mode = MIT_MODE;
    DATA_dmMotorInitCmdCtrl(&s_dmMotor1);
    s_dmMotor1.ctrl.kp_set = 30;
    s_dmMotor1.ctrl.kd_set = 0.5;
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
                s_dmMotor1.ctrl.pos_set = s_dmMotor1.para.pos;
                CTRL_dmMotorEnable(&hcan1, &s_dmMotor1);
            }
            else
            {
                s_dmMotor1.ctrl.kp_set = 30;
                s_dmMotor1.ctrl.kd_set = 0.5;
                s_dmMotor1.ctrl.pos_set = s_dmMotor1.para.pos;
                CTRL_dmMotorEnable(&hcan1, &s_dmMotor1);
            }
        }
    }
    CTRL_dmMotorCtrl(&hcan1, &s_dmMotor1);
}
