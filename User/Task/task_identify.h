/**
 ******************************************************************************
 * @file           : task_identify.h
 * @author         : WHY
 * @date           : 2026-4-14
 * @brief          : task_identify.cpp 的头文件
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef TASK_IDENTIFY_H
#define TASK_IDENTIFY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    G_Identifying = 0,
    G_Ready,
    J_Identifying,
    J_Ready,
    Identify_Failure,
} ENU_IdentifyState;
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported defines ----------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void TASK_Identify(void const* argument);
void TASK_IdentifyInit();

#ifdef __cplusplus
}
#endif

#endif /* TASK_IDENTIFY_H */
