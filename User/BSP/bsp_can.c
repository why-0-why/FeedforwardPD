/*******************************************************************************
 * Robofuture RM Team
 * @file        bsp_can.c
 * @author      Zhb
 * @version     1.0
 * @date        2021/3/12
 * @brief       CAN通信驱动层封装，提供高效的收发管理机制
 *              基于HAL库实现，支持多CAN设备管理和FIFO缓冲
 *              需在CubeMX中配置CAN的接收与发送回调函数
 * @note        该驱动实现了CAN对象的统一管理，支持中断方式的收发处理
*******************************************************************************/

/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp_can.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/
#define CAN_DEVICE  1 ///< 系统支持的CAN设备总数（CAN1和CAN2）

/* 私有变量 ------------------------------------------------------------------*/
/**
 * @brief CAN对象管理数组
 *
 * 保存所有创建的CAN对象指针，用于在中断回调中快速查找对应的CAN对象
 */
static OBJ_CAN* m_objects[CAN_DEVICE];

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/
static void COMM_CAN_TransmitHandler(OBJ_CAN* obj);

/* 函数体 --------------------------------------------------------------------*/

/**
 * @brief 配置CAN接收过滤器
 * @param hcan CAN控制器句柄指针
 * @param filter_bank 过滤器组编号
 *
 * @note 使用32位掩码模式，接收所有标准ID帧，配置失败会触发错误处理
 */
void BSP_CAN_FilterConfig(CAN_HandleTypeDef* hcan, uint32_t filter_bank)
{
    CAN_FilterTypeDef  filter_config;
    /* 配置过滤器参数 */
    filter_config.FilterBank = filter_bank;           // 设置过滤器组编号
    filter_config.FilterMode = CAN_FILTERMODE_IDMASK; // 使用掩码模式
    filter_config.FilterScale = CAN_FILTERSCALE_32BIT; // 32位过滤器尺度
    filter_config.FilterIdHigh = 0x0000;              // 过滤器ID高16位
    filter_config.FilterIdLow = 0x0000;               // 过滤器ID低16位
    filter_config.FilterMaskIdHigh = 0x0000;          // 过滤器掩码高16位
    filter_config.FilterMaskIdLow = 0x0000;           // 过滤器掩码低16位
    filter_config.FilterFIFOAssignment = CAN_RX_FIFO0; // 分配到接收FIFO0
    filter_config.FilterActivation = ENABLE;          // 使能过滤器
    filter_config.SlaveStartFilterBank = 14;          // 从CAN过滤器起始组

    /* 应用过滤器配置 */
    if (HAL_CAN_ConfigFilter(hcan, &filter_config) != HAL_OK)
    {
        Error_Handler(); // 配置失败，进入错误处理
    }
}

/**
 * @brief 初始化CAN设备并启动相关中断
 * @param hcan CAN控制器句柄指针
 * @param active_it 使能的中断类型
 *
 * @note 根据CAN实例自动配置不同的过滤器组，初始化失败会触发错误处理
 */
void CTRL_CANInit(CAN_HandleTypeDef* hcan, uint32_t active_it)
{
    /* 根据CAN实例选择不同的过滤器组配置 */
    if (hcan->Instance == CAN1)
    {
    	BSP_CAN_FilterConfig(hcan, 0); // CAN1使用过滤器组0
    }
#ifdef CAN2
    else if (hcan->Instance == CAN2)
    {
    	BSP_CAN_FilterConfig(hcan, 14); // CAN2使用过滤器组14
    }
#endif

    /* 启动CAN控制器 */
    if (HAL_CAN_Start(hcan) != HAL_OK)
    {
        Error_Handler(); // 启动失败，进入错误处理
    }

    /* 激活CAN中断通知 */
    if (HAL_CAN_ActivateNotification(hcan, active_it) != HAL_OK)
    {
        Error_Handler(); // 中断激活失败，进入错误处理
    }
}

/**
 * @brief 创建并初始化CAN收发管理对象
 * @param obj CAN对象指针
 * @param hcan CAN控制器句柄指针
 * @param tx_fifo_buff 发送FIFO缓冲区指针
 * @param fun 数据接收回调函数
 * @return BSP_Status_e 创建状态
 *
 * @note 对象创建后会自动添加到全局管理数组中，并初始化CAN硬件
 */
ENU_BSPStatus CTRL_CANNewObject(OBJ_CAN* obj, CAN_HandleTypeDef* hcan, uint8_t* tx_fifo_buff, CALL_CANRx fun)
{
    /* 检查对象指针有效性 */
    if (obj == NULL)
		return BSP_ERROR;

    /* 初始化CAN对象成员变量 */
    obj->hcan = hcan;                   // 保存CAN控制器句柄
    obj->is_sending = 0;                // 初始发送状态为未发送
    obj->tx_fifo_buffer = tx_fifo_buff; // 保存发送FIFO缓冲区指针
    obj->rx_callback = fun;             // 保存接收回调函数
    /* 初始化发送FIFO */
    DATA_FIFOInit(&(obj->tx_fifo), tx_fifo_buff, sizeof(STR_CANTxMsg), CAN_TX_FIFO_UNIT_NUM);

    /* 将对象添加到全局管理数组 */
    for (uint8_t i=0; i < CAN_DEVICE; i++)
    {
        if (m_objects[i] == NULL) // 找到空闲位置
        {
        	/* 初始化CAN硬件并启用中断 */
        	CTRL_CANInit(hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY);
            m_objects[i] = obj; // 保存对象指针到全局数组
            return BSP_OK;      // 创建成功
        }
    }
    return BSP_ERROR; // 对象数组已满，创建失败
}

/**
 * @brief 设置CAN数据接收回调函数
 * @param obj CAN对象指针
 * @param fun 接收回调函数指针
 * @return BSP_Status_e 设置状态
 */
ENU_BSPStatus DATA_CANSetRxCallback(OBJ_CAN* obj, CALL_CANRx fun)
{
    /* 参数有效性检查 */
    if (obj == NULL)
        return BSP_ERROR;
    if (fun == NULL)
        return BSP_ERROR;

    obj->rx_callback = fun; // 更新接收回调函数
    return BSP_OK;          // 设置成功
}

/**
 * @brief 通过HAL库CAN句柄发送数据
 * @param hcan CAN控制器句柄指针
 * @param std_id CAN标准标识符
 * @param data 待发送数据指针
 * @param len 待发送数据长度
 * @return BSP_Status_e 发送状态
 *
 * @note 内部通过查找对应的CAN对象来发送数据
 */
ENU_BSPStatus COMM_CANHandleSendData(CAN_HandleTypeDef* hcan, uint32_t std_id, uint8_t* data, uint16_t len)
{
    /* 检查CAN句柄有效性 */
    if(hcan == NULL)
        return BSP_ERROR;

    /* 遍历CAN对象数组查找匹配的CAN实例 */
    for (uint8_t i=0; i < CAN_DEVICE; i++)
    {
        if (m_objects[i]->hcan->Instance == hcan->Instance) // 找到匹配的CAN对象
        {
            /* 调用对象发送函数 */
            return COMM_CANObjSendData(m_objects[i], std_id, data, len);
        }
    }
    return BSP_ERROR; // 未找到对应的CAN对象
}

/**
 * @brief 通过CAN对象发送数据
 * @param obj CAN对象指针
 * @param std_id CAN标准标识符
 * @param data 待发送数据指针
 * @param len 待发送数据长度
 * @return BSP_Status_e 发送状态
 *
 * @note 数据会被分割为多个CAN帧并存入发送FIFO，使用临界区保护确保线程安全
 */
ENU_BSPStatus COMM_CANObjSendData(OBJ_CAN* obj, uint32_t std_id, uint8_t* data, uint16_t len)
{
    ENU_BSPStatus res = BSP_ERROR; // 默认返回错误状态
    uint8_t *send_ptr;            // 发送数据指针
    uint16_t send_num;            // 已发送字节计数
    STR_CANTxMsg msg;              // CAN发送消息结构

    if (obj == NULL)
        return BSP_ERROR;

    /* 初始化发送参数 */
    send_ptr = data;        // 指向待发送数据起始位置
    msg.std_id = std_id;    // 设置CAN标准标识符
    send_num = 0;           // 已发送字节数清零

    /* 进入临界区保护FIFO操作 */
    CRITICAL_SECTION_ENTER();

    /* 循环发送所有数据 */
    while (send_num < len)
    {
        /* 检查发送FIFO是否已满 */
        if (DATA_FIFOIsFull(&(obj->tx_fifo)))
        {
            // FIFO已满，停止发送并重置发送状态
            obj->is_sending = 0;
            break; // 退出发送循环
        }

        /* 计算当前CAN帧的数据长度 */
        if (len - send_num >= 8) // 剩余数据大于等于8字节
        {
            msg.dlc = 8; // 发送完整的一帧（8字节）
        }
        else // 剩余数据不足8字节
        {
            msg.dlc = len - send_num; // 发送剩余的所有数据
        }

        /* 拷贝数据到CAN消息结构 */
        // 使用32位拷贝优化性能，分两次拷贝4字节
        *((uint32_t *)(msg.data)) = *((uint32_t *)(send_ptr));         // 拷贝前4字节
        *((uint32_t *)(msg.data + 4)) = *((uint32_t *)(send_ptr + 4)); // 拷贝后4字节

        /* 更新发送状态 */
        send_ptr += msg.dlc;  // 移动数据指针到下一位置
        send_num += msg.dlc;  // 增加已发送字节计数

        /* 将CAN消息存入发送FIFO */
        DATA_FIFOPutNoProtect(&(obj->tx_fifo), &msg);
        res = BSP_OK; // 标记至少有一帧数据成功写入
    }

    /* 退出临界区 */
    CRITICAL_SECTION_EXIT();

    /* 检查是否需要启动发送处理 */
    if ((obj->is_sending) == 0 && (!(DATA_FIFOIsEmpty(&(obj->tx_fifo)))))
    {
        COMM_CAN_TransmitHandler(obj); // 启动发送处理
    }

    return res; // 返回发送状态
}

/**
 * @brief CAN发送数据处理函数
 * @param obj CAN对象指针
 *
 * @note 从发送FIFO中取出数据并通过CAN邮箱发送，在临界区内执行确保线程安全
 */
static void COMM_CAN_TransmitHandler(OBJ_CAN* obj)
{
    STR_CANTxMsg msg;                // CAN发送消息结构
    CAN_TxHeaderTypeDef   header;   // CAN发送头结构
    uint32_t              mailbox;  // 发送邮箱编号

    /* 检查对象指针有效性 */
    if(obj == NULL)
        return;

    /* 进入临界区保护发送状态和FIFO操作 */
    CRITICAL_SECTION_ENTER();

    /* 检查发送FIFO中是否有待发送数据 */
    if (!DATA_FIFOIsEmpty(&(obj->tx_fifo)))
    {
        /* 循环发送直到邮箱用尽或FIFO为空 */
        while (HAL_CAN_GetTxMailboxesFreeLevel(obj->hcan) && (!(DATA_FIFOIsEmpty(&(obj->tx_fifo)))))
        {
            /* 从FIFO中取出一个CAN消息 */
            DATA_FIFOGetNoProtect(&(obj->tx_fifo), &msg);

            /* 配置CAN发送头参数 */
            header.StdId = msg.std_id;   // 设置标准标识符
            header.IDE = CAN_ID_STD;     // 设置为标准帧
            header.RTR = CAN_RTR_DATA;   // 设置为数据帧
            header.DLC = msg.dlc;        // 设置数据长度

            obj->is_sending = 1; // 标记为发送状态

            /* 将消息添加到CAN发送邮箱 */
            HAL_CAN_AddTxMessage(obj->hcan, &header, msg.data, &mailbox);
        }
    }
    else // FIFO中没有待发送数据
    {
        obj->is_sending = 0; // 标记发送完成
    }

    /* 退出临界区 */
    CRITICAL_SECTION_EXIT();
}

/**
 * @brief CAN发送邮箱0完成中断回调
 * @param hcan CAN控制器句柄指针
 *
 * @note HAL库中断回调函数，在发送完成时自动调用
 */
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef* hcan)
{
    /* 遍历所有CAN对象查找匹配的CAN实例 */
    for (uint8_t i=0; (i < CAN_DEVICE); i++)
    {
        if (m_objects[i]->hcan->Instance == hcan->Instance) // 找到匹配的CAN对象
        {
            COMM_CAN_TransmitHandler(m_objects[i]); // 触发发送处理
        }
    }
}

/**
 * @brief CAN发送邮箱1完成中断回调
 * @param hcan CAN控制器句柄指针
 *
 * @note HAL库中断回调函数，在发送完成时自动调用
 */
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef* hcan)
{
    /* 遍历所有CAN对象查找匹配的CAN实例 */
    for (uint8_t i=0; (i < CAN_DEVICE); i++)
    {
        if (m_objects[i]->hcan->Instance == hcan->Instance) // 找到匹配的CAN对象
        {
            COMM_CAN_TransmitHandler(m_objects[i]); // 触发发送处理
        }
    }
}

/**
 * @brief CAN发送邮箱2完成中断回调
 * @param hcan CAN控制器句柄指针
 *
 * @note HAL库中断回调函数，在发送完成时自动调用
 */
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef* hcan)
{
    /* 遍历所有CAN对象查找匹配的CAN实例 */
    for (uint8_t i=0; (i < CAN_DEVICE); i++)
    {
        if (m_objects[i]->hcan->Instance == hcan->Instance) // 找到匹配的CAN对象
        {
            COMM_CAN_TransmitHandler(m_objects[i]); // 触发发送处理
        }
    }
}

/**
 * @brief CAN异常回调函数
 * @param hcan CAN控制器句柄指针
 *
 * @note HAL库错误回调函数，在CAN通信错误时自动调用
 */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* hcan)
{
    /* 重置CAN错误状态 */
    HAL_CAN_ResetError(hcan);
}

/**
 * @brief CAN接收FIFO0消息挂起回调
 * @param hcan CAN控制器句柄指针
 *
 * @note HAL库中断回调函数，在FIFO0接收到数据时自动调用
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    CAN_RxHeaderTypeDef   rx_header; // 接收消息头
    uint8_t               rx_data[8]; // 接收数据缓冲区

    /* 从CAN接收FIFO0中读取消息 */
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    /* 遍历所有CAN对象查找匹配的CAN实例 */
    for (uint8_t i=0; i < CAN_DEVICE; i++)
    {
        if (m_objects[i]->hcan->Instance == hcan->Instance) // 找到匹配的CAN对象
        {
            /* 调用接收回调函数处理接收到的数据 */
            if (m_objects[i]->rx_callback != NULL)
                m_objects[i]->rx_callback(rx_header.StdId, rx_data, rx_header.DLC);
        }
    }
}

/**
 * @brief CAN接收FIFO1消息挂起回调
 * @param hcan CAN控制器句柄指针
 *
 * @note HAL库中断回调函数，在FIFO1接收到数据时自动调用
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    CAN_RxHeaderTypeDef   rx_header; // 接收消息头
    uint8_t               rx_data[8]; // 接收数据缓冲区

    /* 从CAN接收FIFO0中读取消息 */
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    /* 遍历所有CAN对象查找匹配的CAN实例 */
    for (uint8_t i=0; i < CAN_DEVICE; i++)
    {
        if (m_objects[i]->hcan->Instance == hcan->Instance) // 找到匹配的CAN对象
        {
            /* 调用接收回调函数处理接收到的数据 */
            if (m_objects[i]->rx_callback != NULL)
                m_objects[i]->rx_callback(rx_header.StdId, rx_data, rx_header.DLC);
        }
    }
}