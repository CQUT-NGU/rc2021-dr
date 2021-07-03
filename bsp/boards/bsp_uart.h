/**
 * *****************************************************************************
 * @file         bsp_uart.h
 * @brief        uart of boards
 * @author       NGU
 * @date         20210417
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
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

#include "bsp_dma.h"
#include "main.h"

#include <stdint.h>

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

__BEGIN_DECLS

__END_DECLS

/* Enddef to prevent recursive inclusion ------------------------------------ */
#endif /* __BSP_UART_H__ */

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
