/**
 * *****************************************************************************
 * @file         bsp_uart.c/h
 * @brief        uart of boards
 * @author       ngu
 * @date         20210417
 * @version      1
 * @copyright    Copyright (c) 2021
 * @code         utf-8                                                  @endcode
 * @details
 *               UART7_TX ------> PE8
 *               UART7_RX ------> PE7
 *               UART8_TX ------> PE1
 *               UART8_RX ------> PE0
 * *****************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_UART_H__
#define __BSP_UART_H__

/* Includes ------------------------------------------------------------------*/
#include "bsp_dma.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/
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

/* Exported types ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/

__BEGIN_DECLS

__END_DECLS

/* Private defines -----------------------------------------------------------*/

/* __BSP_UART_H__ ------------------------------------------------------------*/
#endif /* __BSP_UART_H__ */

/************************ (C) COPYRIGHT ngu ********************END OF FILE****/
