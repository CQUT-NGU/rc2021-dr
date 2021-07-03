/**
 * *****************************************************************************
 * @file         bsp_usart.h
 * @brief        usart of boards
 * @author       NGU
 * @date         20210101
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
 * @details
 *               USART1_RX ------> PB7
 *               USART1_TX ------> PA9
 *               USART3_RX ------> PC11 ------> DBUS
 *               USART6_TX ------> PG14
 *               USART6_RX ------> PG9
 * *****************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#include "bsp_dma.h"
#include "main.h"

#include <stdint.h>

#define huart_rc huart3 /* dbus, connect to the remote control */
#undef RC_IRQHandler
#define RC_IRQHandler USART3_IRQHandler

#define huart_os huart1 /* usart, connect to the upper computer */
#undef OS_IRQHandler
#define OS_IRQHandler USART1_IRQHandler

extern UART_HandleTypeDef huart_rc;
extern UART_HandleTypeDef huart_os;

#define BUFSIZ_OS 256U
extern uint8_t *os_buf;

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

/**
 * @brief        Disable USART
 * @param[in]    huart: UART handle Structure pointer
*/
extern void usart_disable(UART_HandleTypeDef *huart);

/**
 * @brief        Enable USART
 * @param[in]    huart: UART handle Structure pointer
*/
extern void usart_enable(UART_HandleTypeDef *huart);

/**
 * @brief        Restart USART DMA receiver
 * @param[in]    huart: UART handle Structure pointer
 * @param[in]    len:   the length of data
*/
extern void usart_dma_rerx(UART_HandleTypeDef *huart,
                           uint16_t len);

/**
 * @brief        Restart USART DMA transmit
 * @param[in]    huart: UART handle Structure pointer
 * @param[in]    len:   the length of data
*/
extern void usart_dma_retx(UART_HandleTypeDef *huart,
                           uint16_t len);

/**
 * @brief        Initializes USART DMA receiver
 * @param[in]    huart: UART handle Structure pointer
*/
extern void usart_dma_rx_init(UART_HandleTypeDef *huart);

/**
 * @brief        Initializes USART DMA transmit
 * @param[in]    huart: UART handle Structure pointer
*/
extern void usart_dma_tx_init(UART_HandleTypeDef *huart);

/**
 * @brief        Initializes USART DMA receiver and transmit
 * @param[in]    huart: UART handle Structure pointer
*/
extern void usart_dma_init(UART_HandleTypeDef *huart);

/**
 * @brief        Enable double memory buffer
 * @param[in]    huart: UART handle Structure pointer
 * @param[in]    buf1:  DMA stream x memory 0 address register
 * @param[in]    buf2:  DMA stream x memory 1 address register
 * @param[in]    len:   DMA stream x number of data register
*/
extern void usart_dma_rx(UART_HandleTypeDef *huart,
                         void *buf1,
                         void *buf2,
                         uint16_t len);

/**
 * @brief        USART transmit by DMA Stream
 * @param[in]    huart: UART handle Structure pointer
 * @param[in]    data:  the address of data
 * @param[in]    len:   the length of data
*/
extern void usart_dma_tx(UART_HandleTypeDef *huart,
                         void *data,
                         uint16_t len);

/**
 * @brief        print string to USART
 * @param[in]    format: printf format
*/
extern void dma_printf(const char *fmt, ...);

/**
 * @brief        print string to USART
 * @param[in]    format: printf format
*/
extern void os_printf(const char *format, ...);

/**
 * @brief        print float data to USART by little-endian mode
 * @param[in]    n: number of float data
 * @param[in]    ...: float data
*/
extern void os_justfloat(uint8_t n, ...);

extern void os_dma_init(void);

__END_DECLS

static inline void usart_init(void)
{
    /* Enable the usart that connect to the upper computer */
    usart_dma_init(&huart_os);
    /* Enable the usart that connect to the remote control */
    usart_dma_rx_init(&huart_rc);
}

/* Enddef to prevent recursive inclusion ------------------------------------ */
#endif /* __BSP_USART_H__ */

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
