/**
******************************************************************************
  * @file           : drv_fifo.c
  * @author         : WHY
  * @date           : 2025-10-11
  * @brief          : 软件fifo的驱动模块
  *
* 本文件提供了FIFO的创建、初始化和操作函数：
  *     - fifo_s_create(): 动态创建单字节FIFO实例
  *     - fifo_s_init(): 初始化静态单字节FIFO实例
  *     - fifo_s_put(): 向单字节FIFO写入数据
  *     - fifo_s_get(): 从单字节FIFO读取数据
  *     - fifo_s_puts(): 批量写入数据到单字节FIFO
  *     - fifo_s_gets(): 批量从单字节FIFO读取数据
  *     - fifo_s_isempty(): 检查单字节FIFO是否为空
  *     - fifo_s_isfull(): 检查单字节FIFO是否已满
  *     - fifo_s_flush(): 清空单字节FIFO
  *     - fifo_create(): 动态创建通用FIFO实例
  *     - fifo_init(): 初始化静态通用FIFO实例
  *     - fifo_put(): 向通用FIFO写入数据
  *     - fifo_get(): 从通用FIFO读取数据
  *     - fifo_is_empty(): 检查通用FIFO是否为空
  *     - fifo_is_full(): 检查通用FIFO是否已满
  *     - fifo_flush(): 清空通用FIFO
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "drv_fifo.h"

/* Private includes ----------------------------------------------------------*/
#include "main.h"// 提供hal库
#include <string.h>
#include "bsp_def.h"

/* Private macro -------------------------------------------------------------*/

/* Private user code ---------------------------------------------------------*/
#ifdef USE_DYNAMIC_MEMORY
/**
  * @brief  创建新的FIFO实例（单字节模式）。
  * 此函数为N个块的FIFO元素分配足够空间，然后返回FIFO的指针。
  *
  * @param  [in] uint_cnt FIFO元素的数量。
  * @retval FIFO实例的指针，如果分配内存失败则返回NULL。
  *
  * @note   -# 在使用此函数前，必须启用USE_MEMORY_ALLOC宏并确保系统包含<stdlib.h>头文件。
  * @note   -# FIFO_Create和FIFO_Destory函数必须成对使用。
  */
fifo_s_t *fifo_s_create(int uint_cnt)
{
    fifo_s_t *p_fifo = NULL;  //!< FIFO Pointer
    char *p_base_addr = NULL; //!< Memory Base Address

    //! Check input parameters.
    assert_param(uint_cnt);

    //! Allocate Memory for pointer of new FIFO Control Block
    p_fifo = (fifo_s_t *)malloc(sizeof(fifo_s_t));
    if (NULL == p_fifo)
    {
        //! Allocate Failure, exit now
        return (NULL);
    }
    //! Allocate Memory for pointer of new FIFO
    p_base_addr = malloc(uint_cnt);
    if (NULL == p_base_addr)
    {
        //! Allocate Failure, exit now
        free(p_fifo);
        return (NULL);
    }
    //! Initialize General FIFO Module
    fifo_s_init(p_fifo, p_base_addr, uint_cnt);

    return (p_fifo);
}

/**
  * @brief  销毁FIFO实例（单字节模式）。
  * 此函数释放内存，然后重新初始化FIFO结构体。
  *
  * @param  [in] p_fifo FIFO实例的指针。
  * @retval 无。
  *
  * @note   在使用此函数前，必须启用USE_MEMORY_ALLOC宏并确保系统包含<stdlib.h>头文件。
  */
void fifo_s_destroy(fifo_s_t *p_fifo)
{
    //! Check input parameters.
    assert_param(p_fifo);
    assert_param(p_fifo->p_start_addr);

    //! Free FIFO memory
    free(p_fifo->p_start_addr);
    //! Free FIFO Control Block memory
    free(p_fifo);

    return; //!< Success
}

#endif // USE_DYNAMIC_MEMORY

/**
  * @brief  初始化静态FIFO结构体（单字节模式）。
  * @param  [in] p_fifo 有效FIFO实例的指针。
  * @param  [in] p_base_addr 预分配内存的基地址，例如数组。
  * @param  [in] uint_cnt FIFO元素的数量。
  * @retval 如果初始化成功返回0，否则返回-1。
  */
int DATA_SFIFOInit(STR_SFIFO *p_fifo, void *p_base_addr, int uint_cnt)
{
    //! Check input parameters.
    assert_param(p_fifo);
    assert_param(p_base_addr);
    assert_param(uint_cnt);

    //! Initialize FIFO Control Block.
    p_fifo->p_start_addr = (char *)p_base_addr;
    p_fifo->p_end_addr = (char *)p_base_addr + uint_cnt - 1;
    p_fifo->free_num = uint_cnt;
    p_fifo->used_num = 0;
    p_fifo->p_read_addr = (char *)p_base_addr;
    p_fifo->p_write_addr = (char *)p_base_addr;

    return (0);
}

/**
  * @brief  将一个元素放入FIFO（单字节模式）。
  * @param  [in]  p_fifo 有效FIFO的指针。
  * @param  [in]  element 要放入的数据元素
  * @retval 如果操作成功返回0，否则返回-1。
  */
int DATA_SFIFOPut(STR_SFIFO *p_fifo, char element)
{
    //! Check input parameters.
    assert_param(p_fifo);

    if (0 == p_fifo->free_num)
    {
        //! Error, FIFO is full!
        return (-1);
    }

    INT_DISABLE();

    if (0 == p_fifo->free_num)
    {
        //! Error, FIFO is full!
        INT_ENABLE();
        return (-1);
    }

    if (p_fifo->p_write_addr > p_fifo->p_end_addr)
    {
        p_fifo->p_write_addr = p_fifo->p_start_addr;
    }

    *(p_fifo->p_write_addr) = element;
    p_fifo->p_write_addr++;
    p_fifo->free_num--;
    p_fifo->used_num++;
    INT_ENABLE();

    return (0);
}

/**
  * @brief  将多个元素放入FIFO（单字节模式）。
  * @param  [in]  p_fifo 有效FIFO的指针。
  * @param  [in]  p_source 要放入的数据元素
  * @param  [in]  len 元素数量
  * @retval 实际写入的数据数量，否则返回-1。
  */
int DATA_SFIFOPuts(STR_SFIFO *p_fifo, char *p_source, int len)
{
    int retval;
    int len_to_end;
    int len_from_start;

    assert_param(p_fifo);

    if (NULL == p_source)
    {
        return -1;
    }

    if (0 == p_fifo->free_num)
    {
        return 0;
    }

    INT_DISABLE();

    if (0 == p_fifo->free_num)
    {
        INT_ENABLE();
        return 0;
    }

    if (p_fifo->p_write_addr > p_fifo->p_end_addr)
    {
        p_fifo->p_write_addr = p_fifo->p_start_addr;
    }

    len = (len < p_fifo->free_num) ? len : p_fifo->free_num;
    len_to_end = p_fifo->p_end_addr - p_fifo->p_write_addr + 1;

    if (len_to_end >= len) //no rollback
    {
        len_to_end = len;
        memcpy(p_fifo->p_write_addr, p_source, len_to_end);
        p_fifo->p_write_addr += len_to_end;
    }
    else //rollback
    {
        len_from_start = len - len_to_end;
        memcpy(p_fifo->p_write_addr, p_source, len_to_end);
        memcpy(p_fifo->p_start_addr, p_source + len_to_end, len_from_start);
        p_fifo->p_write_addr = p_fifo->p_start_addr + len_from_start;
    }

    p_fifo->free_num -= len;
    p_fifo->used_num += len;
    retval = len;
    INT_ENABLE();

    return retval;
}

/**
  * @brief  将多个元素放入FIFO，忽略中断保护
  * @param  [in]  p_fifo 有效FIFO的指针。
  * @param  [in]  p_source 要放入的数据元素
  * @param  [in]  len 元素数量
  * @retval 实际写入的数据数量，否则返回-1。
  */
int DATA_SFIFOPutsNoProtect(STR_SFIFO *p_fifo, char *p_source, int len)
{
    int retval;
    int len_to_end;
    int len_from_start;

    assert_param(p_fifo);

    if (NULL == p_source)
    {
        return -1;
    }

    if (0 == p_fifo->free_num)
    {
        return 0;
    }

    if (p_fifo->p_write_addr > p_fifo->p_end_addr)
    {
        p_fifo->p_write_addr = p_fifo->p_start_addr;
    }

    len = (len < p_fifo->free_num) ? len : p_fifo->free_num;
    len_to_end = p_fifo->p_end_addr - p_fifo->p_write_addr + 1;

    if (len_to_end >= len) //no rollback
    {
        len_to_end = len;
        memcpy(p_fifo->p_write_addr, p_source, len_to_end);
        p_fifo->p_write_addr += len_to_end;
    }
    else //rollback
    {
        len_from_start = len - len_to_end;
        memcpy(p_fifo->p_write_addr, p_source, len_to_end);
        memcpy(p_fifo->p_start_addr, p_source + len_to_end, len_from_start);
        p_fifo->p_write_addr = p_fifo->p_start_addr + len_from_start;
    }

    p_fifo->free_num -= len;
    p_fifo->used_num += len;
    retval = len;

    return retval;
}

/**
  * @brief  从FIFO中获取一个元素（单字节模式）。
  * @param  [in]  p_fifo 有效FIFO的指针。
  * @retval FIFO的数据元素。
  */
char DATA_SFIFOGet(STR_SFIFO *p_fifo)
{
    char retval = 0;

    //! Check input parameters.
    assert_param(p_fifo);

    //TODO:
    if (0 == p_fifo->used_num)
    {
        return 0;
    }

    INT_DISABLE();

    if (0 == p_fifo->used_num)
    {
        INT_ENABLE();
        return 0;
    }

    if (p_fifo->p_read_addr > p_fifo->p_end_addr)
    {
        p_fifo->p_read_addr = p_fifo->p_start_addr;
    }

    retval = *p_fifo->p_read_addr;
    // Update information
    p_fifo->p_read_addr++;
    p_fifo->free_num++;
    p_fifo->used_num--;
    INT_ENABLE();

    return (retval);
}

/**
  * @brief  从FIFO中获取多个元素（单字节模式）。
  * @param  [in]  p_fifo 有效FIFO的指针。
  * @retval 实际读取的数据数量。
  */
int DATA_SFIFOGets(STR_SFIFO *p_fifo, char *p_dest, int len)
{
    int retval;
    int len_to_end;
    int len_from_start;

    assert_param(p_fifo);

    if (NULL == p_dest)
    {
        return -1;
    }

    if (0 == p_fifo->used_num)
    {
        return 0;
    }

    INT_DISABLE();

    if (0 == p_fifo->used_num)
    {
        INT_ENABLE();
        return 0;
    }

    if (p_fifo->p_read_addr > p_fifo->p_end_addr)
    {
        p_fifo->p_read_addr = p_fifo->p_start_addr;
    }

    len = (len < p_fifo->used_num) ? len : p_fifo->used_num;
    len_to_end = p_fifo->p_end_addr - p_fifo->p_read_addr + 1;

    if (len_to_end >= len) //no rollback
    {
        len_to_end = len;
        memcpy(p_dest, p_fifo->p_read_addr, len_to_end);
        p_fifo->p_read_addr += len_to_end;
    }
    else //rollback
    {
        len_from_start = len - len_to_end;
        memcpy(p_dest, p_fifo->p_read_addr, len_to_end);
        memcpy(p_dest + len_to_end, p_fifo->p_start_addr, len_from_start);
        p_fifo->p_read_addr = p_fifo->p_start_addr + len_from_start;
    }

    p_fifo->free_num += len;
    p_fifo->used_num -= len;
    retval = len;
    INT_ENABLE();

    return retval;
}

/**
  * @brief  从FIFO中获取多个元素（单字节模式）。
  * @param  [in]  p_fifo 有效FIFO的指针。
  * @retval 实际读取的数据数量。
  */
int DATA_SFIFOGetsNoProtect(STR_SFIFO *p_fifo, char *p_dest, int len)
{
    int retval;
    int len_to_end;
    int len_from_start;

    assert_param(p_fifo);

    if (NULL == p_dest)
    {
        return -1;
    }

    if (0 == p_fifo->used_num)
    {
        return 0;
    }

    if (p_fifo->p_read_addr > p_fifo->p_end_addr)
    {
        p_fifo->p_read_addr = p_fifo->p_start_addr;
    }

    len = (len < p_fifo->used_num) ? len : p_fifo->used_num;
    len_to_end = p_fifo->p_end_addr - p_fifo->p_read_addr + 1;

    if (len_to_end >= len) //no rollback
    {
        len_to_end = len;
        memcpy(p_dest, p_fifo->p_read_addr, len_to_end);
        p_fifo->p_read_addr += len_to_end;
    }
    else //rollback
    {
        len_from_start = len - len_to_end;
        memcpy(p_dest, p_fifo->p_read_addr, len_to_end);
        memcpy(p_dest + len_to_end, p_fifo->p_start_addr, len_from_start);
        p_fifo->p_read_addr = p_fifo->p_start_addr + len_from_start;
    }

    p_fifo->free_num += len;
    p_fifo->used_num -= len;
    retval = len;

    return retval;
}

/**
  * @brief  预读FIFO中的一个元素（单字节模式）。
  * @param  [in]  p_fifo 有效FIFO的指针。
  * @param  [in]  offset 距离当前指针的偏移量。
  * @retval FIFO的数据元素。
  */
char DATA_SFIFOPreRead(STR_SFIFO *p_fifo, int offset)
{
    char *tmp_read_addr;

    //! Check input parameters.
    assert_param(p_fifo);

    if (offset > p_fifo->used_num)
    {
        return 0;
    }
    else
    {
        // Move Read Pointer to right position
        tmp_read_addr = p_fifo->p_read_addr + offset;
        if (tmp_read_addr > p_fifo->p_end_addr)
        {
            tmp_read_addr = tmp_read_addr - p_fifo->p_end_addr + p_fifo->p_start_addr - 1;
        }

        return *tmp_read_addr;
    }
}

/**
  * @brief  预读多个元素从FIFO中（单字节模式）。
  * 此函数从指定偏移位置开始预读多个元素，不会移动实际的读指针位置。
  *
  * @param  [in]  p_fifo 有效FIFO的指针。
  * @param  [out] p_dest 目标缓冲区地址，用于存储读取的数据。
  * @param  [in]  offset 距离当前读指针的偏移量。
  * @param  [in]  len 请求读取的元素数量。
  * @retval 实际成功读取的数据数量，如果出错返回-1。
  *
  * @note   -# 此操作不会改变FIFO的读指针位置和已用/空闲计数。
  * @note   -# 如果请求的长度超过可用数据量，则只读取可用数据。
  * @note   -# 此函数带有互斥锁保护，确保多任务环境下的数据安全。
  */
int DATA_SFIFOPreReads(STR_SFIFO *p_fifo, char *p_dest, int offset, int len)
{
    int retval;
    char *tmp_read_addr;
    int len_to_end;
    int len_from_start;

    assert_param(p_fifo);

    if (NULL == p_dest)
    {
        return -1;
    }

    if (0 == p_fifo->used_num)
    {
        return -1;
    }

    if (offset >= p_fifo->used_num)
    {
        return -1;
    }

    INT_DISABLE();

    if (0 == p_fifo->used_num)
    {
        INT_ENABLE();
        return -1;
    }

    if (offset >= p_fifo->used_num)
    {
        INT_ENABLE();
        return -1;
    }

    tmp_read_addr = p_fifo->p_read_addr + offset;
    if (tmp_read_addr > p_fifo->p_end_addr)
    {
        tmp_read_addr = tmp_read_addr - p_fifo->p_end_addr + p_fifo->p_start_addr - 1;
    }

    len = (len < (p_fifo->used_num - offset)) ? len : (p_fifo->used_num - offset);
    len_to_end = p_fifo->p_end_addr - tmp_read_addr + 1;

    if (len_to_end >= len) //no rollback
    {
        len_to_end = len;
        memcpy(p_dest, tmp_read_addr, len_to_end);
    }
    else //rollback
    {
        len_from_start = len - len_to_end;
        memcpy(p_dest, tmp_read_addr, len_to_end);
        memcpy(p_dest + len_to_end, p_fifo->p_start_addr, len_from_start);
    }

    retval = len;
    INT_ENABLE();

    return retval;
}

/**
  * @brief  FIFO是否为空（单字节模式）？
  * @param  [in] p_fifo 有效FIFO的指针。
  * @retval - 非零值(true) 如果为空
  *         - 零(false) 如果不为空
  */
char DATA_SFIFOIsEmpty(STR_SFIFO *p_fifo)
{
    //! Check input parameter.
    assert_param(p_fifo);
    return (p_fifo->used_num ? 0 : 1);
}

/**
  * @brief  FIFO是否为满（单字节模式）？
  * @param  [in] p_fifo 有效FIFO的指针。
  * @retval - 非零值(true) 如果为满
  *         - 零(false) 如果不为满
  */
char DATA_SFIFOIsFull(STR_SFIFO *p_fifo)
{
    //! Check input parameter.
    assert_param(p_fifo);
    return (p_fifo->free_num ? 0 : 1);
}

/**
  * @brief  获取FIFO中已使用的元素数量（单字节模式）
  * @param  [in] p_fifo 有效FIFO的指针。
  * @retval FIFO中的元素数量。
  */
int DATA_SFIFOUsed(STR_SFIFO *p_fifo)
{
    //! Check input parameter.
    assert_param(p_fifo);
    return p_fifo->used_num;
}

/**
  * @brief  获取FIFO中空闲的元素数量（单字节模式）
  * @param  [in] p_fifo 有效FIFO的指针。
  * @retval FIFO中空闲的元素数量。
  */
int DATA_SFIFOFree(STR_SFIFO *p_fifo)
{
    //! Check input parameter.
    assert_param(p_fifo);
    return p_fifo->free_num;
}

/**
  * @brief  清空FIFO的内容。
  * 此函数将FIFO重置为空状态，重置读写指针和计数。
  *
  * @param  [in] p_fifo 有效FIFO的指针。
  * @retval 无。
  *
  * @note   -# 此操作会丢失FIFO中的所有数据。
  * @note   -# 函数带有互斥锁保护，确保操作原子性。
  */
void DATA_SFIFOFlush(STR_SFIFO *p_fifo)
{
    //! Check input parameters.
    assert_param(p_fifo);
    //! Initialize FIFO Control Block.
    INT_DISABLE();
    p_fifo->free_num = p_fifo->p_end_addr - p_fifo->p_start_addr + 1;
    p_fifo->used_num = 0;
    p_fifo->p_read_addr = p_fifo->p_start_addr;
    p_fifo->p_write_addr = p_fifo->p_start_addr;
    INT_ENABLE();
}

/**
  * @brief  从FIFO中丢弃指定数量的元素。
  * 此函数移动读指针来跳过指定数量的元素，相当于快速删除数据。
  *
  * @param  [in] p_fifo 有效FIFO的指针。
  * @param  [in] len 要丢弃的元素数量。
  * @retval 实际丢弃的元素数量。
  *
  * @note   -# 如果请求丢弃的数量超过已用数量，则只丢弃所有可用数据。
  * @note   -# 此操作不可逆，丢弃的数据无法恢复。
  * @note   -# 函数带有互斥锁保护，确保操作原子性。
  */
int DATA_SFIFODiscard(STR_SFIFO *p_fifo, int len)
{
    //! Check input parameters.
    char *tmp_index;
    assert_param(p_fifo);

    INT_DISABLE();
    if (len > p_fifo->used_num)
    {
        len = p_fifo->used_num;
    }

    tmp_index = len + p_fifo->p_read_addr;
    if (tmp_index > p_fifo->p_end_addr)
    {
        tmp_index = tmp_index - p_fifo->p_end_addr + p_fifo->p_start_addr - 1;
    }
    p_fifo->p_read_addr = tmp_index;
    p_fifo->free_num += len;
    p_fifo->used_num -= len;
    INT_ENABLE();
    return len;
}

#ifdef USE_DYNAMIC_MEMORY
/**
  * @brief  创建新的FIFO实例。
  * 此函数为N个块的FIFO元素分配足够空间，然后返回指针
  *
  * @note   -# 在使用此函数前，必须启用USE_MEMORY_ALLOC宏并确保系统包含<stdlib.h>
  *            头文件。
  * @note   -# FIFO_Create和FIFO_Destory函数必须成对使用。
  *
  * @param  [in] unit_size FIFO元素大小。
  * @param	[in] unit_cnt FIFO元素数量。
  * @retval FIFO实例的指针，如果分配内存失败则返回NULL。
  */
fifo_t *fifo_create(char unit_size, int unit_cnt)
{
    fifo_t *p_fifo = NULL;    //!< FIFO Pointer
    char *p_base_addr = NULL; //!< Memory Base Address

    //! Check input parameters.
    assert_param(unit_size);
    assert_param(unit_cnt);

    //! Allocate Memory for pointer of new FIFO Control Block.
    p_fifo = (fifo_t *)malloc(sizeof(fifo_t));
    if (NULL == p_fifo)
    {
        //! Allocate Failure, exit now.
        return (NULL);
    }

    //! Allocate memory for FIFO.
    p_base_addr = malloc(unit_size * unit_cnt);
    if (NULL == p_base_addr)
    {
        //! Allocate Failure, exit now.
        free(p_fifo);
        return (NULL);
    }

    //! Initialize General FIFO Module.
    fifo_init(p_fifo, p_base_addr, unit_size, unit_cnt);

  return (p_fifo);
}

/**
  * @brief  销毁FIFO实例。
  * 此函数释放内存，然后重新初始化FIFO结构体。
  *
  * @note   -# 在使用此函数前，必须启用USE_MEMORY_ALLOC宏并确保系统包含<stdlib.h>
  *            头文件。
  *
  * @param  [in] p_fifo FIFO实例的指针
  * @retval 无。
  */
void fifo_destory(fifo_t *p_fifo)
{
    //! Check input parameters.
    assert_param(p_fifo);
    assert_param(p_fifo->p_start_addr);

    //! Free FIFO memory
    free(p_fifo->p_start_addr);
    //! Free FIFO Control Block memory.
    free(p_fifo);

    return; //!< Success
}

#endif // USE_DYNAMIC_MEMORY

/**
  * @brief  初始化静态FIFO结构体。
  * @param  [in] p_fifo 有效FIFO实例的指针。
  * @param  [in] p_base_addr 预分配内存的基地址，例如数组。
  * @param  [in] unit_size FIFO元素大小。
  * @param  [in] unit_cnt FIFO元素数量。
  * @retval 如果初始化成功返回0，否则返回-1。
  */
int DATA_FIFOInit(STR_FIFO *p_fifo, void *p_base_addr, int unit_size, int unit_cnt)
{
    //! Check input parameters.
    assert_param(p_fifo);
    assert_param(p_base_addr);
    assert_param(unit_size);
    assert_param(unit_cnt);

    //! Initialize FIFO Control Block.
    p_fifo->p_start_addr = (char *)p_base_addr;
    p_fifo->p_end_addr = (char *)p_base_addr + unit_size * unit_cnt - 1;
    p_fifo->free_num = unit_cnt;
    p_fifo->used_num = 0;
    p_fifo->unit_size = unit_size;
    p_fifo->p_read_addr = (char *)p_base_addr;
    p_fifo->p_write_addr = (char *)p_base_addr;

    return (0);
}

/**
  * @brief  将一个元素放入FIFO。
  * @param  [in] p_fifo 有效FIFO的指针。
  * @param  [in] p_element 要放入的元素地址
  * @retval 如果操作成功返回0，否则返回-1。
  */
int DATA_FIFOPut(STR_FIFO *p_fifo, void *p_element)
{
    //! Check input parameters.
    assert_param(p_fifo);
    assert_param(p_element);

    // Full ?
    if (0 == p_fifo->free_num)
    {
        //! Error, FIFO is full!
        return (-1);
    }

    //! Copy Data
    INT_DISABLE();

    if (0 == p_fifo->free_num)
    {
        //! Error, FIFO is full!
        INT_ENABLE();
        return (-1);
    }

    if (p_fifo->p_write_addr > p_fifo->p_end_addr)
    {
        p_fifo->p_write_addr = p_fifo->p_start_addr;
    }

    memcpy(p_fifo->p_write_addr, p_element, p_fifo->unit_size);
    p_fifo->p_write_addr += p_fifo->unit_size;
    p_fifo->free_num--;
    p_fifo->used_num++;
    INT_ENABLE();

    return (0);
}

/**
  * @brief  将一个元素放入FIFO，忽略中断保护。
  * @param  [in] p_fifo 有效FIFO的指针。
  * @param  [in] p_element 要放入的元素地址
  * @retval 如果操作成功返回0，否则返回-1。
  */
int DATA_FIFOPutNoProtect(STR_FIFO *p_fifo, void *p_element)
{
    //! Check input parameters.
    assert_param(p_fifo);
    assert_param(p_element);

    // Full ?
    if (0 == p_fifo->free_num)
    {
        //! Error, FIFO is full!
        return (-1);
    }

    //! Copy Data
    if (p_fifo->p_write_addr > p_fifo->p_end_addr)
        p_fifo->p_write_addr = p_fifo->p_start_addr;

    memcpy(p_fifo->p_write_addr, p_element, p_fifo->unit_size);
    p_fifo->p_write_addr += p_fifo->unit_size;
    p_fifo->free_num--;
    p_fifo->used_num++;

    return (0);
}

/**
  * @brief  从FIFO中获取一个元素。
  * @param  [in]  p_fifo 有效FIFO的指针。
  * @param	[out] p_element 要获取的元素地址。
  * @retval 如果操作成功返回0，否则返回-1。
  */
int DATA_FIFOGet(STR_FIFO *p_fifo, void *p_element)
{
    //! Check input parameters.
    assert_param(p_fifo);
    assert_param(p_element);

    // Empty ?
    if (0 == p_fifo->used_num)
    {
        //! Error, FIFO is Empty!
        return (-1);
    }

    //! Copy Data
    INT_DISABLE();

    if (0 == p_fifo->used_num)
    {
        //! Error, FIFO is Empty!
        INT_ENABLE();
        return (-1);
    }

    if (p_fifo->p_read_addr > p_fifo->p_end_addr)
    {
        p_fifo->p_read_addr = p_fifo->p_start_addr;
    }
    memcpy(p_element, p_fifo->p_read_addr, p_fifo->unit_size);
    p_fifo->p_read_addr += p_fifo->unit_size;
    p_fifo->free_num++;
    p_fifo->used_num--;
    INT_ENABLE();

    return (0);
}

/**
  * @brief  从FIFO中获取一个元素。
  * @param  [in]  p_fifo 有效FIFO的指针。
  * @param	[out] p_element 要获取的元素地址。
  * @retval 如果操作成功返回0，否则返回-1。
  */
int DATA_FIFOGetNoProtect(STR_FIFO *p_fifo, void *p_element)
{
    //! Check input parameters.
    assert_param(p_fifo);
    assert_param(p_element);

    // Empty ?
    if (0 == p_fifo->used_num)
    {
        //! Error, FIFO is Empty!
        return (-1);
    }

    //! Copy Data
    if (p_fifo->p_read_addr > p_fifo->p_end_addr)
    {
        p_fifo->p_read_addr = p_fifo->p_start_addr;
    }
    memcpy(p_element, p_fifo->p_read_addr, p_fifo->unit_size);
    p_fifo->p_read_addr += p_fifo->unit_size;
    p_fifo->free_num++;
    p_fifo->used_num--;

    return (0);
}

/**
  * @brief  预读FIFO中的一个元素。
  * @param  [in]  p_fifo 有效FIFO的指针。
  * @param	[in]  offset 距离当前指针的偏移量。
  * @param	[out] p_element 要获取的元素地址
  * @retval 如果操作成功返回0，否则返回-1。
  */
int DATA_FIFOPreRead(STR_FIFO *p_fifo, char offset, void *p_element)
{
    char *_pre_red_index = (void *)0;

    //! Check input parameters.
    assert_param(p_fifo);
    assert_param(p_element);

    // OverFlow ?
    if (offset >= p_fifo->used_num)
    {
        return (-1);
    }

    // Move Read Pointer to right position
    _pre_red_index = p_fifo->p_read_addr + p_fifo->unit_size * offset;
    while (_pre_red_index > p_fifo->p_end_addr)
    {
        _pre_red_index = _pre_red_index - p_fifo->p_end_addr + p_fifo->p_start_addr - 1;
    }
    //! Copy Data
    memcpy(p_element, _pre_red_index, p_fifo->unit_size);

    return (0);
}

/**
  * @brief  FIFO是否为空
  * @param  [in] p_fifo 有效FIFO的指针。
  * @retval - 非零值(true) 如果为空
  *         - 零(false) 如果不为空
  */
int DATA_FIFOIsEmpty(STR_FIFO *p_fifo)
{
    //! Check input parameter.
    assert_param(p_fifo);

    return (0 == p_fifo->used_num);
}

/**
  * @brief  FIFO是否为满
  * @param  [in] p_fifo 有效FIFO的指针。
  * @retval - 非零值(true) 如果为满
  *         - 零(false) 如果不为满
  */
int DATA_FIFOIsFull(STR_FIFO *p_fifo)
{
    //! Check input parameter.
    assert_param(p_fifo);

    return (0 == p_fifo->free_num);
}

/**
  * @brief  获取FIFO中已使用的元素数量
  * @param  [in] p_fifo 有效FIFO的指针。
  * @retval FIFO中的元素数量。
  */
int DATA_FIFOUsed(STR_FIFO *p_fifo)
{
    //! Check input parameter.
    assert_param(p_fifo);

    return (p_fifo->used_num);
}

/**
  * @brief  获取FIFO中空闲的元素数量
  * @param  [in] p_fifo 有效FIFO的指针。
  * @retval FIFO中空闲的元素数量。
  */
int DATA_FIFOFree(STR_FIFO *p_fifo)
{
    //! Check input parameter.
    assert_param(p_fifo);

    return (p_fifo->free_num);
}

/**
  * @brief  清空FIFO的内容。
  * @param  [in] p_fifo 有效FIFO的指针。
  * @retval 如果成功返回0，如果失败返回-1。
  */
int DATA_FIFOFlush(STR_FIFO *p_fifo)
{
    //! Check input parameters.
    assert_param(p_fifo);

    //! Initialize FIFO Control Block.
    INT_DISABLE();
    p_fifo->free_num = (p_fifo->p_end_addr - p_fifo->p_start_addr) / (p_fifo->unit_size);
    p_fifo->used_num = 0;
    p_fifo->p_read_addr = p_fifo->p_start_addr;
    p_fifo->p_write_addr = p_fifo->p_start_addr;
    INT_ENABLE();

    return (0);
}
