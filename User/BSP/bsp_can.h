/*******************************************************************************
 * Robofuture RM Team
 * @file        bsp_can.h
 * @author      Zhb
 * @version     1.0
 * @date        2021/3/12
 * @brief       CAN通信驱动头文件
 *              定义CAN对象结构、数据类型和接口函数声明
 *              提供CAN通信的初始化、发送和接收管理功能
 * @note        支持多CAN设备管理，基于FIFO缓冲实现高效数据收发
*******************************************************************************/

#ifndef BSP_CAN_H
#define BSP_CAN_H
#ifdef __cplusplus
extern "C" {
#endif
/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp_def.h"     // 板级支持包定义，包含BSP_Status_e等类型
#include "drv_fifo.h"
#include "can.h"        // STM32Cube HAL CAN头文件
/* 类型定义 ------------------------------------------------------------------*/

/**
 * @brief CAN数据接收回调函数类型
 * @param std_id CAN标准标识符
 * @param data 接收数据指针
 * @param dlc 数据长度码
 *
 * @note 当CAN接收到数据时，通过此回调函数通知应用程序
 */
typedef void (*CALL_CANRx)(uint32_t std_id, uint8_t* data, uint32_t dlc);

/**
 * @brief CAN对象管理结构体
 *
 * 封装CAN通信所需的所有资源，包括硬件句柄、发送FIFO和接收回调
 */
typedef struct
{
    /* Handle */
    CAN_HandleTypeDef* hcan;           ///< CAN硬件控制器句柄指针

    /* Tx 发送相关成员 */
    STR_FIFO tx_fifo;                    ///< 发送数据FIFO缓冲区
    uint8_t* tx_fifo_buffer;           ///< 发送FIFO存储区指针
    uint8_t is_sending;                ///< 发送状态标志：0-空闲，1-发送中

    /* Rx 接收相关成员 */
    CALL_CANRx rx_callback;  ///< 数据接收回调函数指针
} OBJ_CAN;

/**
 * @brief CAN发送消息结构体
 *
 * 定义CAN数据帧的完整结构，包括标识符、数据长度和载荷数据
 */
typedef struct
{
    uint32_t std_id;    ///< CAN标准标识符（11位）
    uint8_t dlc;        ///< 数据长度码（0-8字节）
    uint8_t data[8];    ///< 数据载荷（最大8字节）
} STR_CANTxMsg;

/* 宏定义 --------------------------------------------------------------------*/

#define CAN_TX_FIFO_UNIT_NUM (256)     ///< 发送FIFO中可存储的CAN消息单元数量
#define CAN_TX_FIFO_SIZE (CAN_TX_FIFO_UNIT_NUM * sizeof(STR_CANTxMsg))  ///< 发送FIFO总大小（字节）

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/

void CTRL_CANInit(CAN_HandleTypeDef* hcan, uint32_t active_it);
ENU_BSPStatus CTRL_CANNewObject(OBJ_CAN* obj, CAN_HandleTypeDef* hcan,
                               uint8_t* tx_fifo_buff, CALL_CANRx fun);
ENU_BSPStatus DATA_CANSetRxCallback(OBJ_CAN* obj, CALL_CANRx fun);
ENU_BSPStatus COMM_CANHandleSendData(CAN_HandleTypeDef* hcan, uint32_t std_id,
                               uint8_t* data, uint16_t len);
ENU_BSPStatus COMM_CANObjSendData(OBJ_CAN* obj, uint32_t std_id,
                                  uint8_t* data, uint16_t len);

#ifdef __cplusplus
}
#endif
#endif  // BSP_CAN_H

