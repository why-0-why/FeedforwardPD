/**
******************************************************************************
 * @file           : XXX.c
 * @author         : WHY
 * @date           : 2025-10-12
 * @brief          : XXX的驱动模块
 *
 * 本文件提供了XXX的初始化函数和控制函数：
 *     - XXX_Init(): 初始化XXX的对象，配置为模式
 *     - XXX_show(): 根据输入的XX值控制XXX显示
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_dmMotor.h"
/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/**
 * @brief:      	CTRL_dmMotorEnable: 启用DM4310电机控制模式函数
 * @param[in]:   hcan:    指向CAN_HandleTypeDef结构的指针
 * @param[in]:   motor:   指向motor_t结构的指针，包含电机相关信息和控制参数
 * @retval:     	void
 * @details:    	根据电机控制模式启用相应的模式，通过CAN总线发送启用命令
 *               支持的控制模式包括位置模式、位置速度控制模式和速度控制模式
 **/
void CTRL_dmMotorEnable(CAN_HandleTypeDef* hcan, STR_dmMotor* motor)
{
    motor->enable_flag = 1;
    switch (motor->ctrl.mode)
    {
    case MIT_MODE:
        COMM_dmMotorEnable(hcan, motor->id, MIT_CMD);
        break;
    case POS_MODE:
        COMM_dmMotorEnable(hcan, motor->id, POS_CMD);
        break;
    case SPEED_MODE:
        COMM_dmMotorEnable(hcan, motor->id, SPEED_CMD);
        break;
    }
}

/**
 * @brief:      	CTRL_dmMotorDisable: 禁用DM4310电机控制模式函数
 * @param[in]:   hcan:    指向CAN_HandleTypeDef结构的指针
 * @param[in]:   motor:   指向motor_t结构的指针，包含电机相关信息和控制参数
 * @retval:     	void
 * @details:    	根据电机控制模式禁用相应的模式，通过CAN总线发送禁用命令
 *               支持的控制模式包括位置模式、位置速度控制模式和速度控制模式
 **/
void CTRL_dmMotorDisable(CAN_HandleTypeDef* hcan, STR_dmMotor* motor)
{
    motor->enable_flag = 0;
    switch (motor->ctrl.mode)
    {
    case MIT_MODE:
        COMM_dmMotorDisable(hcan, motor->id, MIT_CMD);
        break;
    case POS_MODE:
        COMM_dmMotorDisable(hcan, motor->id, POS_CMD);
        break;
    case SPEED_MODE:
        COMM_dmMotorDisable(hcan, motor->id, SPEED_CMD);
        break;
    }
    DATA_dmMotorInitCmdCtrl(motor);
}

/**
 * @brief:      	CTRL_dmMotorClearErr: 清除DM4310电机错误函数
 * @param[in]:   hcan: 	 指向CAN控制结构体的指针
 * @param[in]:  	motor:   指向电机结构体的指针
 * @retval:     	void
 * @details:    	根据电机的控制模式，调用对应模式的清除错误函数
 **/
void CTRL_dmMotorClearErr(CAN_HandleTypeDef* hcan, STR_dmMotor* motor)
{
    switch (motor->ctrl.mode)
    {
    case MIT_MODE:
        COMM_dmMotorClearErr(hcan, motor->id, MIT_CMD);
        break;
    case POS_MODE:
        COMM_dmMotorClearErr(hcan, motor->id, POS_CMD);
        break;
    case SPEED_MODE:
        COMM_dmMotorClearErr(hcan, motor->id, SPEED_CMD);
        break;
    }
}

/**
 * @brief:      	CTRL_dmMotorCtrl: 发送DM4310电机控制命令函数
 * @param[in]:   hcan:    指向CAN_HandleTypeDef结构的指针
 * @param[in]:   motor:   指向motor_t结构的指针，包含电机相关信息和控制参数
 * @retval:     	void
 * @details:    	根据电机控制模式发送相应的命令到DM4310电机
 *               支持的控制模式包括位置模式、位置速度控制模式和速度控制模式
 **/
void CTRL_dmMotorCtrl(CAN_HandleTypeDef* hcan, STR_dmMotor* motor)
{
    switch (motor->ctrl.mode)
    {
    case MIT_MODE:
        COMM_dmMotorMitCtrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set, motor->ctrl.kp_set,
                            motor->ctrl.kd_set, motor->ctrl.tor_set);
        break;
    case POS_MODE:
        COMM_dmMotorPosSpeedCtrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set);
        break;
    case SPEED_MODE:
        COMM_dmMotorSpeedCtrl(hcan, motor->id, motor->ctrl.vel_set);
        break;
    }
}

/**
 * @brief:      	DATA_dmMotorInitCmdCtrl: 清除DM4310电机控制参数函数
 * @param[in]:   motor:   指向motor_t结构的指针，包含电机相关信息和控制参数
 * @retval:     	void
 * @details:    	将DM4310电机的命令参数和控制参数清零，包括位置、速度、
 *               比例增益(KP)、微分增益(KD)和扭矩
 **/
void DATA_dmMotorInitCmdCtrl(STR_dmMotor* motor)
{
    motor->cmd.kd_set = 0;
    motor->cmd.kp_set = 0;
    motor->cmd.pos_set = 0;
    motor->cmd.vel_set = 0;
    motor->cmd.tor_set = 0;

    motor->ctrl.kd_set = 0;
    motor->ctrl.kp_set = 0;
    motor->ctrl.pos_set = 0;
    motor->ctrl.vel_set = 0;
    motor->ctrl.tor_set = 0;
}

/**
 * @brief:      	DATA_dmMotorCmd2Ctrl: 设置DM4310电机控制参数函数
 * @param[in]:   motor:   指向motor_t结构的指针，包含电机相关信息和控制参数
 * @retval:     	void
 * @details:    	根据命令参数设置DM4310电机的控制参数，包括位置、速度、
 *               比例增益(KP)、微分增益(KD)和扭矩
 **/
void DATA_dmMotorCmd2Ctrl(STR_dmMotor* motor)
{
    motor->ctrl.kd_set = motor->cmd.kd_set;
    motor->ctrl.kp_set = motor->cmd.kp_set;
    motor->ctrl.pos_set = motor->cmd.pos_set;
    motor->ctrl.vel_set = motor->cmd.vel_set;
    motor->ctrl.tor_set = motor->cmd.tor_set;
}

/**
 * @brief:      	DATA_dmMortorFbdata: 获取DM4310电机反馈数据函数
 * @param[in]:   motor:    指向motor_t结构的指针，包含电机相关信息和反馈数据
 * @param[in]:   rx_data:  指向包含反馈数据的数组指针
 * @retval:     	void
 * @details:    	从接收到的数据中提取DM4310电机的反馈信息，包括电机ID、
 *               状态、位置、速度、扭矩以及相关温度参数
 **/
void DATA_dmMortorFbdata(STR_dmMotor* motor, uint8_t* rx_data)
{
    motor->para.id = (rx_data[0]) & 0x0F;
    motor->para.state = (rx_data[0]) >> 4;
    motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
    motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    motor->para.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
    motor->para.pos = DATA_Uint2Float(motor->para.p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
    motor->para.vel = DATA_Uint2Float(motor->para.v_int, V_MIN, V_MAX, 12); // (-45.0,45.0)
    motor->para.tor = DATA_Uint2Float(motor->para.t_int, T_MIN, T_MAX, 12); // (-18.0,18.0)
    motor->para.Tmos = (float)(rx_data[6]);
    motor->para.Tcoil = (float)(rx_data[7]);
}

/**
 * @brief:      	DATA_Float2Uint: 浮点数转换为无符号整数函数
 * @param[in]:   x_float:	待转换的浮点数
 * @param[in]:   x_min:		范围最小值
 * @param[in]:   x_max:		范围最大值
 * @param[in]:   bits: 		目标无符号整数的位数
 * @retval:     	无符号整数结果
 * @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
 **/
int DATA_Float2Uint(float x_float, float x_min, float x_max, int bits)
{
    /* Converts a float to an unsigned int, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief:      	DATA_Uint2Float: 无符号整数转换为浮点数函数
 * @param[in]:   x_int: 待转换的无符号整数
 * @param[in]:   x_min: 范围最小值
 * @param[in]:   x_max: 范围最大值
 * @param[in]:   bits:  无符号整数的位数
 * @retval:     	浮点数结果
 * @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
 **/
float DATA_Uint2Float(int x_int, float x_min, float x_max, int bits)
{
    /* converts unsigned int to float, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief:      	COMM_dmMotorEnable: 启用电机模式函数
 * @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
 * @param[in]:   motor_id: 电机ID，指定目标电机
 * @param[in]:   mode_id:  模式ID，指定要开启的模式
 * @retval:     	void
 * @details:    	通过CAN总线向特定电机发送启用特定模式的命令
 **/
void COMM_dmMotorEnable(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
    uint8_t data[8];
    uint16_t id = motor_id + mode_id;

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFC;

    COMM_CANHandleSendData(hcan, id, data, 8);
}

/**
 * @brief:      	COMM_dmMotorDisable: 禁用电机模式函数
 * @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
 * @param[in]:   motor_id: 电机ID，指定目标电机
 * @param[in]:   mode_id:  模式ID，指定要禁用的模式
 * @retval:     	void
 * @details:    	通过CAN总线向特定电机发送禁用特定模式的命令
 **/
void COMM_dmMotorDisable(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
    uint8_t data[8];
    uint16_t id = motor_id + mode_id;

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFD;

    COMM_CANHandleSendData(hcan, id, data, 8);
}


/**
 * @brief:      	COMM_dmMotorSavePosZero: 保存位置零点函数
 * @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
 * @param[in]:   motor_id: 电机ID，指定目标电机
 * @param[in]:   mode_id:  模式ID，指定要保存位置零点的模式
 * @retval:     	void
 * @details:    	通过CAN总线向特定电机发送保存位置零点的命令
 **/
void COMM_dmMotorSavePosZero(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
    uint8_t data[8];
    uint16_t id = motor_id + mode_id;

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFE;

    COMM_CANHandleSendData(hcan, id, data, 8);
}

/**
 * @brief:          COMM_dmMotorClearErr: 清除电机错误函数
 * @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
 * @param[in]:   motor_id: 电机ID，指定目标电机
 * @param[in]:   mode_id:  模式ID，指定要清除错误的模式
 * @retval:     	void
 * @details:    	通过CAN总线向特定电机发送清除错误的命令。
 **/
void COMM_dmMotorClearErr(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
    uint8_t data[8];
    uint16_t id = motor_id + mode_id;

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFB;

    COMM_CANHandleSendData(hcan, id, data, 8);
}

/**
 * @brief:      	COMM_dmMotorMitCtrl: MIT模式下的电机控制函数
 * @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
 * @param[in]:   motor_id:	电机ID，指定目标电机
 * @param[in]:   pos:			位置给定值
 * @param[in]:   vel:			速度给定值
 * @param[in]:   kp:				位置比例系数
 * @param[in]:   kd:				位置微分系数
 * @param[in]:   torq:			转矩给定值
 * @retval:     	void
 * @details:    	通过CAN总线向电机发送MIT模式下的控制帧。
 **/
void COMM_dmMotorMitCtrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel, float kp, float kd,
                         float torq)
{
    uint8_t data[8];
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    uint16_t id = motor_id + MIT_CMD;

    pos_tmp = DATA_Float2Uint(pos, P_MIN, P_MAX, 16);
    vel_tmp = DATA_Float2Uint(vel, V_MIN, V_MAX, 12);
    kp_tmp = DATA_Float2Uint(kp, KP_MIN, KP_MAX, 12);
    kd_tmp = DATA_Float2Uint(kd, KD_MIN, KD_MAX, 12);
    tor_tmp = DATA_Float2Uint(torq, T_MIN, T_MAX, 12);

    data[0] = (pos_tmp >> 8);
    data[1] = pos_tmp;
    data[2] = (vel_tmp >> 4);
    data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    data[4] = kp_tmp;
    data[5] = (kd_tmp >> 4);
    data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    data[7] = tor_tmp;

    COMM_CANHandleSendData(hcan, id, data, 8);
}

/**
 * @brief:      	COMM_dmMotorPosSpeedCtrl: 位置速度控制函数
 * @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
 * @param[in]:   motor_id:	电机ID，指定目标电机
 * @param[in]:   vel:			速度给定值
 * @retval:     	void
 * @details:    	通过CAN总线向电机发送位置速度控制命令
 **/
void COMM_dmMotorPosSpeedCtrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel)
{
    uint16_t id;
    uint8_t *pbuf, *vbuf;
    uint8_t data[8];

    id = motor_id + POS_CMD;
    pbuf = (uint8_t*)&pos;
    vbuf = (uint8_t*)&vel;

    data[0] = *pbuf;
    data[1] = *(pbuf + 1);
    data[2] = *(pbuf + 2);
    data[3] = *(pbuf + 3);

    data[4] = *vbuf;
    data[5] = *(vbuf + 1);
    data[6] = *(vbuf + 2);
    data[7] = *(vbuf + 3);

    COMM_CANHandleSendData(hcan, id, data, 8);
}

/**
 * @brief:      	COMM_dmMotorSpeedCtrl: 速度控制函数
 * @param[in]:   hcan: 		指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
 * @param[in]:   motor_id: 电机ID，指定目标电机
 * @param[in]:   vel: 			速度给定值
 * @retval:     	void
 * @details:    	通过CAN总线向电机发送速度控制命令
 **/
void COMM_dmMotorSpeedCtrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float vel)
{
    uint16_t id;
    uint8_t* vbuf;
    uint8_t data[4];

    id = motor_id + SPEED_CMD;
    vbuf = (uint8_t*)&vel;

    data[0] = *vbuf;
    data[1] = *(vbuf + 1);
    data[2] = *(vbuf + 2);
    data[3] = *(vbuf + 3);

    COMM_CANHandleSendData(hcan, id, data, 4);
}
