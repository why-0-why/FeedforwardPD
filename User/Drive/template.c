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

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
    int id;
    float val;
} STR_StructName;//结构体用STR开头，枚举用ENU开头，回调函数类型用CALL开头，对象（有变量也有函数的结构体）用OBJ开头
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
float f_variable1;
double f_variable2;
int i_variable3;
int a_variables1[4]={0};
float a_variable2[4]={0};
STR_StructName s_strut1;//全局变量开头用a代表数组，开头用i代表整型，开头用f代表浮点型，开头用c代表字符型，开头用s代表结构体型，开头用p代表指针型
/* Exported variables --------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void do_sth_inside();
/* Private user code ---------------------------------------------------------*/

//函数命名分5种，函数名开头用TASK代表具体应用任务，DATA代表数据处理，CTRL代表控制执行，COM代表通讯相关，内部函数按具体功能用小写下划线命名

/**
 * @brief          数据：数据第一次搬运及处理
 * @param[in]      void
 * @return         void
 */
void DATA_FUN1()
{

}

/**
 * @brief          控制：控制模式转换，控制外设
 * @param[in]      void
 * @return         void
 */
void CTRL_FUN1()
{

}

/**
 * @brief          通讯：数据组装与发送
 * @param[in]      void
 * @return         void
 */
void COM_FUN1()
{

}

/**
 * @brief          内部数据转换
 * @param[in]      void
 * @return         void
 */
void do_sth_inside()
{

}