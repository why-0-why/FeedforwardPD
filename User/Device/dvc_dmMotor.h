/**
******************************************************************************
 * @file           : dvc_dmMotor.h
 * @author         : WHY
 * @date           : 2026-4-8
 * @brief          : dvc_dmMotor.c 的头文件
 *
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DVC_DMMOTOR_H
#define DVC_DMMOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "bsp_can.h"
/* Exported defines ----------------------------------------------------------*/

#define MIT_CMD 		0x000
#define POS_CMD			0x100
#define SPEED_CMD		0x200
#define MST_ID 			0x000 //反馈帧默认000

#define MIT_MODE 			0x000
#define POS_MODE 			0x001
#define SPEED_MODE 		    0x002

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

/* Exported types ------------------------------------------------------------*/
// 电机回传信息结构体
typedef struct
{
    int id;
    int state;
    int p_int;
    int v_int;
    int t_int;
    int kp_int;
    int kd_int;
    float pos_rad;//弧度制
    float pos_deg;//角度制
    float vel;
    float tor;
    float Kp;
    float Kd;
    float Tmos;
    float Tcoil;
} STR_dmMotorFbpara;

// 电机参数设置结构体
typedef struct
{
    int8_t mode;
    float pos_set;
    float vel_set;
    float tor_set;
    float kp_set;
    float kd_set;
} STR_dmMotorCtrl;

typedef struct
{
    int8_t id; //电机id
    uint8_t enable_flag; //运行状态
    STR_dmMotorFbpara para; //电机状态
    STR_dmMotorCtrl ctrl; //电机控制器
    STR_dmMotorCtrl cmd; //电机指令，通过函数将指令更新至控制器
} STR_dmMotor;

/* Exported variables --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void CTRL_dmMotorEnable(CAN_HandleTypeDef* hcan, STR_dmMotor* motor);
void CTRL_dmMotorDisable(CAN_HandleTypeDef* hcan, STR_dmMotor* motor);
void CTRL_dmMotorClearErr(CAN_HandleTypeDef* hcan, STR_dmMotor* motor);
void CTRL_dmMotorCtrl(CAN_HandleTypeDef* hcan, STR_dmMotor* motor);

void DATA_dmMotorInitCmdCtrl(STR_dmMotor* motor);
void DATA_dmMotorCmd2Ctrl(STR_dmMotor* motor);
void DATA_dmMortorFbdata(STR_dmMotor* motor, uint8_t* rx_data);
int DATA_Float2Uint(float x_float, float x_min, float x_max, int bits);
float DATA_Uint2Float(int x_int, float x_min, float x_max, int bits);

void COMM_dmMotorEnable(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id);
void COMM_dmMotorDisable(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id);
void COMM_dmMotorSavePosZero(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id);
void COMM_dmMotorClearErr(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id);
void COMM_dmMotorMitCtrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel, float kp, float kd,
                         float torq);
void COMM_dmMotorPosSpeedCtrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel);
void COMM_dmMotorSpeedCtrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float vel);

#ifdef __cplusplus
}
#endif

#endif //DVC_DMMOTOR_H
