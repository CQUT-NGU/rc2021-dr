/**
 * *****************************************************************************
 * @file         ctrl_pc.c/h
 * @brief        control by usart
 * @author       ngu
 * @date         20210427
 * @version      1
 * @copyright    Copyright (c) 2021
 * @code         utf-8                                                  @endcode
 * *****************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CTRL_PC_H__
#define __CTRL_PC_H__

/* Includes ------------------------------------------------------------------*/
/* Private includes ----------------------------------------------------------*/
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/
#define PC_RX_BUFSIZ 64U

/* Exported macro ------------------------------------------------------------*/
#undef __BEGIN_DECLS
#undef __END_DECLS

#if defined(__cplusplus)
#define __BEGIN_DECLS \
    extern "C"        \
    {
#define __END_DECLS }
#else
#define __BEGIN_DECLS
#define __END_DECLS
#endif /* __cplusplus */

#undef __packed
#define __packed __attribute__((__packed__))

/* Exported types ------------------------------------------------------------*/

typedef struct
{
    float x;
    float y;
    float z;

    uint8_t c;
} ctrl_pc_t;

/* Exported functions prototypes ---------------------------------------------*/

__BEGIN_DECLS

/**
 * @brief          Initializes the PC control function
*/
extern void ctrl_pc_init(void);

/**
 * @brief        Get pc control data point
 * @return       ctrl_pc_t pc control data point
*/
extern ctrl_pc_t *ctrl_pc_point(void);

__BEGIN_DECLS

/* Private defines -----------------------------------------------------------*/

/* __CTRL_PC_H__ -------------------------------------------------------------*/
#endif /* __CTRL_PC_H__ */

/************************ (C) COPYRIGHT ngu ********************END OF FILE****/
