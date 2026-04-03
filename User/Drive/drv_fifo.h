/**
******************************************************************************
  * @file           : drv_fifo.h
  * @author         : WHY
  * @date           : 2025-10-11
  * @brief          : drv_fifo.c 的头文件
  *                   包含drv_fifo的宏定义
  *                   drv_fifo的变量类型声明
  *                   drv_fifo的函数声明
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DRV_FIFO_H
#define DRV_FIFO_H
#ifdef __cplusplus
extern "C" {
#endif
/* Exported types ------------------------------------------------------------*/
/* FIFO Memory Model (Single Byte Mode) */
typedef struct
{
    char *p_start_addr; //!< FIFO Memory Pool Start Address
    char *p_end_addr;   //!< FIFO Memory Pool End Address
    int free_num;       //!< The remain capacity of FIFO
    int used_num;       //!< The number of elements in FIFO
    char *p_read_addr;  //!< FIFO Data Read Index Pointer
    char *p_write_addr; //!< FIFO Data Write Index Pointer
} STR_SFIFO;

/* FIFO Memory Model */
typedef struct
{
    char *p_start_addr; //!< FIFO Memory Pool Start Address
    char *p_end_addr;   //!< FIFO Memory Pool End Address
    int free_num;       //!< The remain capacity of FIFO
    int used_num;       //!< The number of elements in FIFO
    int unit_size;      //!< FIFO Element Size(Unit: Byte)
    char *p_read_addr;  //!< FIFO Data Read Index Pointer
    char *p_write_addr; //!< FIFO Data Write Index Pointer
} STR_FIFO;
/* Exported functions prototypes ---------------------------------------------*/
#ifdef USE_DYNAMIC_MEMORY
fifo_s_t *fifo_s_create(int uint_cnt);
void fifo_s_destroy(fifo_s_t *p_fifo);
#endif // USE_DYNAMIC_MEMORY
int DATA_SFIFOInit(STR_SFIFO *p_fifo, void *p_base_addr, int uint_cnt);
int DATA_SFIFOPut(STR_SFIFO *p_fifo, char element);
int DATA_SFIFOPuts(STR_SFIFO *p_fifo, char *p_source, int len);
int DATA_SFIFOPutsNoProtect(STR_SFIFO *p_fifo, char *p_source, int len);
char DATA_SFIFOGet(STR_SFIFO *p_fifo);
int DATA_SFIFOGets(STR_SFIFO *p_fifo, char *p_dest, int len);
int DATA_SFIFOGetsNoProtect(STR_SFIFO *p_fifo, char *p_dest, int len);
char DATA_SFIFOPreRead(STR_SFIFO *p_fifo, int offset);
int DATA_SFIFOPreReads(STR_SFIFO *p_fifo, char *p_dest, int offset, int len);
char DATA_SFIFOIsEmpty(STR_SFIFO *p_fifo);
char DATA_SFIFOIsFull(STR_SFIFO *p_fifo);
int DATA_SFIFOUsed(STR_SFIFO *p_fifo);
int DATA_SFIFOFree(STR_SFIFO *p_fifo);
void DATA_SFIFOFlush(STR_SFIFO *p_fifo);
int DATA_SFIFODiscard(STR_SFIFO *p_fifo, int len);

#ifdef USE_DYNAMIC_MEMORY
fifo_t *fifo_create(char unit_size, int unit_cnt);
void fifo_destory(fifo_t * p_fifo);
#endif // USE_DYNAMIC_MEMORY
int DATA_FIFOInit(STR_FIFO * p_fifo, void *p_base_addr, int unit_size, int unit_cnt);
int DATA_FIFOPut(STR_FIFO * p_fifo, void *p_element);
int DATA_FIFOPutNoProtect(STR_FIFO * p_fifo, void *p_element);
int DATA_FIFOGet(STR_FIFO * p_fifo, void *p_element);
int DATA_FIFOGetNoProtect(STR_FIFO * p_fifo, void *p_element);
int DATA_FIFOPreRead(STR_FIFO * p_fifo, char offset, void *p_element);
int DATA_FIFOIsEmpty(STR_FIFO * p_fifo);
int DATA_FIFOIsFull(STR_FIFO * p_fifo);
int DATA_FIFOUsed(STR_FIFO * p_fifo);
int DATA_FIFOFree(STR_FIFO * p_fifo);
int DATA_FIFOFlush(STR_FIFO * p_fifo);
#ifdef __cplusplus
}
#endif
#endif  //DRV_FIFO_H
