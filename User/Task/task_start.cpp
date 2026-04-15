/**
 ******************************************************************************
 * @file           : task_start.c
 * @author         : WHY
 * @date           : 2026-4-6
 * @brief          : 任务层：启动任务，执行初始化硬件和启动任务;辨识模式辨识参数
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
#include "alg_OLS.h"
#include "arm_math.h"
#include "bsp_can.h"
#include "cmsis_os.h"
#include "dvc_dmMotor.h"
#include "task_ctrl.h"
#include "task_identify.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
extern osThreadId TaskStartHandle;

/* Private variables ---------------------------------------------------------*/
ENU_Mode e_mode = SAFETY;

OBJ_CAN o_can1;
uint8_t a_can1TxFIFOBuff[CAN_TX_FIFO_SIZE]; ///< CAN1发送FIFO缓冲区
STR_dmMotor s_dmMotor1;
/* Exported variables --------------------------------------------------------*/
extern float32_t a_theta[2];
/* Private function prototypes -----------------------------------------------*/
static void DATA_CAN1CallBack(uint32_t std_id, uint8_t* data, uint32_t dlc);
static void DATA_MotorInit();
//static void TASK_Identify();
/* Private user code ---------------------------------------------------------*/

/**
 * @brief          任务-初始化硬件和启动其他任务
 * @param[in]      void
 * @return         void
 */
void TASK_Start(void const* argument)
{
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
    CTRL_CANNewObject(&o_can1, &hcan1, a_can1TxFIFOBuff, DATA_CAN1CallBack);

    while (e_mode == SAFETY)
    {
        osDelay(100); //等待模式切换
    }
    DATA_MotorInit(); //初始化电机控制
    if (e_mode == IDENTIFY)
        TASK_IdentifyInit();
    else
        TASK_CTRLInit();
    while (e_mode==IDENTIFY)//等待辨识解释
    {
        osDelay(1000);
    }

    osThreadTerminate(TaskStartHandle);//任务完成,删除本任务
}

/**
 * @brief 回调：CAN1
 *
 */
void DATA_CAN1CallBack(uint32_t std_id, uint8_t* data, uint32_t dlc)
{
    switch (std_id)
    {
    case MST_ID:
        //int8_t id =(data[0]) & 0x0F;
        DATA_dmMortorFbdata(&s_dmMotor1, data);
        break;
    default:
        break;
    }
}

/**
 * @brief 数据：电机数据初始化
 *
 */
void DATA_MotorInit()
{
    s_dmMotor1.id = 1;
    s_dmMotor1.ctrl.mode = MIT_MODE;
    DATA_dmMotorInitCmdCtrl(&s_dmMotor1);
    s_dmMotor1.ctrl.kp_set = 50;
    s_dmMotor1.ctrl.kd_set = 0.5;
}
