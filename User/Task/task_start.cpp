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
#include "task_feedforward.h"
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
static void TASK_Identify();
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
    TASK_Identify();
    TASK_FeedforwardInit();
    TASK_CTRLInit();

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
    s_dmMotor1.ctrl.kp_set = 30;
    s_dmMotor1.ctrl.kd_set = 0.5;
}

#define NumofData 10
uint8_t i_stepCount = 0;
int8_t i_dataCount = NumofData;
uint64_t i_timeCount = 0;
float32_t f_startPos = -60.0f * M_PI / 180.0f;
float32_t f_endPos = 60.0f * M_PI / 180.0f;
float32_t a_posMeas_rad[NumofData+1] = {0};
float32_t a_torMeas_Nm[NumofData+1] = {0}; //X*theta=Y中的Y
float32_t a_X[(NumofData+1) * 2] = {0}; //X*theta=Y中的X
/**
 * @brief 任务：辨识模式,周期更新目标位置,稳定后获取位置和力;最后用最小二乘法计算模型
 *
 */
void TASK_Identify()
{
    while (e_mode == IDENTIFY)
    {
        switch (i_stepCount)
        {
        case 0://电机使能
            if (!s_dmMotor1.enable_flag)
            {
                i_dataCount=NumofData;
                i_timeCount=HAL_GetTick();//启动时间
                CTRL_dmMotorEnable(&hcan1, &s_dmMotor1);
            }
            else
            {
                i_stepCount = 1;
            }
            break;
        case 1://电机目标递增，并记录位置与力
            if (s_dmMotor1.enable_flag)
            {
                s_dmMotor1.ctrl.pos_set = f_endPos - i_dataCount * (f_endPos - f_startPos)/NumofData;
                CTRL_dmMotorCtrl(&hcan1,&s_dmMotor1);

                if (HAL_GetTick()-i_timeCount>2500)
                {
                    i_timeCount=HAL_GetTick();

                    a_posMeas_rad[i_dataCount]=s_dmMotor1.para.pos_rad;//记录位置
                    a_torMeas_Nm[i_dataCount]=s_dmMotor1.para.tor;//记录力
                    i_dataCount--;

                    if (i_dataCount<0)
                    {
                        i_stepCount=2;//下一步
                    }
                }
            }
            break;
        case 2://最小二乘计算模型
            float32_t a_posSin[NumofData+1] = {0};
            float32_t a_posCos[NumofData+1] = {0};
            for (int i = 0; i < NumofData+1; i++)
            {
                a_posSin[i] = sinf(a_posMeas_rad[i]);
                a_posCos[i] = cosf(a_posMeas_rad[i]);
            }

            for (int i = 0; i < NumofData+1; i++)
            {
                a_X[i * 2] = a_posCos[i];
                a_X[2 * i + 1] = a_posSin[i];
            }
            DATA_OLS(NumofData+1, 2, a_X, a_torMeas_Nm, a_theta);
            e_mode=NORMAL;
            //反算
            // f_mglEst_Nm = sqrt(a_theta[0] * a_theta[0] + a_theta[1] * a_theta[1]);
            // f_alphaEst_rad = atan2(a_theta[1], a_theta[0]);  // 注意顺序: atan2(b, a)
            // f_alphaEst_deg = f_alphaEst_rad * 180.0 / M_PI;
            break;
        }
        osDelay(100); //10hz
    }
}
