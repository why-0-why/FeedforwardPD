#ifndef BSP_DEF_H
#define BSP_DEF_H
#ifdef __cplusplus
extern "C" {
#endif
/* 包含头文件 ----------------------------------------------------------------*/
#include "main.h"
/* 类型定义 ------------------------------------------------------------------*/

/**
 * @brief   驱动层状态枚举
 */
typedef enum
{
    BSP_OK       = 0U,
    BSP_ERROR    = 1U,
} ENU_BSPStatus;

/**
 * @brief   驱动层函数功能使能枚举
 */
typedef enum
{
    BSP_DISABLE = 0U,
    BSP_ENABLE = 1U
} ENU_BSPFunctionalStatus;

/* 宏定义 --------------------------------------------------------------------*/
/**
 * @brief   获取主中断状态/中断使能/关闭/恢复
 */
#define INT_STATE_GET() __get_PRIMASK()
#define INT_ENABLE()    do{__enable_irq();  }while(0)
#define INT_DISABLE()   do{__disable_irq(); }while(0)
#define INT_RESTORE(x)  do{__set_PRIMASK(x);}while(0)

/**
 * @brief   进入临界区（即不能被中断打断的操作）
 *
 * @note 必须和CRITICAL_SECTION_EXIT（）配合使用，两函数间不要使用break/continue
 */
#define CRITICAL_SECTION_ENTER()                  \
do                                                \
{                                                 \
uint32_t cpu_state = INT_STATE_GET(); \
INT_DISABLE();

/**
 * @brief   退出临界区
 *
 * @note 必须和CRITICAL_SECTION_ENTER（）配合使用，两函数间不要使用break/continue
 */
#define CRITICAL_SECTION_EXIT()        \
INT_RESTORE(cpu_state); \
}                                  \
while (0)

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif
#endif /* BSP_DEF_H */
