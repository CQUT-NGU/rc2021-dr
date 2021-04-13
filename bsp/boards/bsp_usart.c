/**
 * *****************************************************************************
 * @file         bsp_usart.c/h
 * @brief        usart of boards
 * @author       ngu
 * @date         20210101
 * @version      1
 * @copyright    Copyright (c) 2021
 * @code         utf-8                                                  @endcode
 * @details
 *               USART6_TX ------> PG14
 *               USART6_RX ------> PG9
 *               USART1_RX ------> PB7
 *               USART1_TX ------> PA9
 *               USART3_RX ------> PC11
 * *****************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "bsp_usart.h"

/* Private includes ----------------------------------------------------------*/
#include "main.h"

#include <stdarg.h>
#include <stdio.h>

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

#define BSP_DMA_CLEAR_FLAG(_)                                     \
                                                                  \
    do                                                            \
    {                                                             \
        /*!< Clear Complete Transmit flag */                      \
        __HAL_DMA_CLEAR_FLAG(_, __HAL_DMA_GET_TC_FLAG_INDEX(_));  \
        /*!< Clear Half Complete Transmit flag */                 \
        __HAL_DMA_CLEAR_FLAG(_, __HAL_DMA_GET_HT_FLAG_INDEX(_));  \
        /*!< Transfer error interrupt mask */                     \
        __HAL_DMA_CLEAR_FLAG(_, __HAL_DMA_GET_TE_FLAG_INDEX(_));  \
        /*!< Direct mode error interrupt */                       \
        __HAL_DMA_CLEAR_FLAG(_, __HAL_DMA_GET_DME_FLAG_INDEX(_)); \
        /*!< FIFO error interrupt mask */                         \
        __HAL_DMA_CLEAR_FLAG(_, __HAL_DMA_GET_FE_FLAG_INDEX(_));  \
    } while (0)

#define BSP_DMA_WAIT_TC(_) \
                           \
    while (!__HAL_DMA_GET_FLAG(_, __HAL_DMA_GET_TC_FLAG_INDEX(_)))

/* Private typedef -----------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/

void usart_disable(UART_HandleTypeDef *huart)
{
    __HAL_UART_DISABLE(huart);
}

void usart_enable(UART_HandleTypeDef *huart)
{
    __HAL_UART_ENABLE(huart);
}

void usart_dma_rerx(UART_HandleTypeDef *huart,
                    uint16_t            len)
{
    __HAL_UART_DISABLE(huart);

    do
    {
        /* Disable the specified DMA Stream */
        __HAL_DMA_DISABLE(huart->hdmarx);
    } while (huart->hdmarx->Instance->CR & DMA_SxCR_EN);

    BSP_DMA_CLEAR_FLAG(huart->hdmarx);

    /*!< DMA stream x number of data register     */
    huart->hdmarx->Instance->NDTR = len;

    __HAL_DMA_ENABLE(huart->hdmarx);
    __HAL_UART_ENABLE(huart);
}

void usart_dma_retx(UART_HandleTypeDef *huart,
                    uint16_t            len)
{
    __HAL_UART_DISABLE(huart);

    do
    {
        /* Disable the specified DMA Stream */
        __HAL_DMA_DISABLE(huart->hdmatx);
    } while (huart->hdmatx->Instance->CR & DMA_SxCR_EN);

    BSP_DMA_CLEAR_FLAG(huart->hdmatx);

    /*!< DMA stream x number of data register     */
    huart->hdmatx->Instance->NDTR = len;

    __HAL_DMA_ENABLE(huart->hdmatx);
    __HAL_UART_ENABLE(huart);
}

void usart_dma_rx_init(UART_HandleTypeDef *huart)
{
    /*!<DMA Enable Receiver         */
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    /* Enable UART */
    __HAL_UART_ENABLE(huart);
    /* Enalbe idle interrupt */
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    do
    {
        /* Disable the specified DMA Stream */
        __HAL_DMA_DISABLE(huart->hdmarx);
    } while (huart->hdmarx->Instance->CR & DMA_SxCR_EN);

    BSP_DMA_CLEAR_FLAG(huart->hdmarx);

    /*!< DMA stream x peripheral address register */
    huart->hdmarx->Instance->PAR = (uint32_t) & (huart->Instance->DR);
}

void usart_dma_tx_init(UART_HandleTypeDef *huart)
{
    /*!<DMA Enable Transmitter      */
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAT);

    /* Enable UART */
    __HAL_UART_ENABLE(huart);

    do
    {
        /* Disable the specified DMA Stream */
        __HAL_DMA_DISABLE(huart->hdmatx);
    } while (huart->hdmatx->Instance->CR & DMA_SxCR_EN);

    BSP_DMA_CLEAR_FLAG(huart->hdmatx);

    /*!< DMA stream x peripheral address register */
    huart->hdmatx->Instance->PAR = (uint32_t) & (huart->Instance->DR);
}

void usart_dma_init(UART_HandleTypeDef *huart)
{
    usart_dma_rx_init(huart);
    usart_dma_tx_init(huart);
}

void usart_dma_rx(UART_HandleTypeDef *huart,
                  uint8_t *           buf1,
                  uint8_t *           buf2,
                  uint16_t            len)
{
    do
    {
        /* Disable the specified DMA Stream */
        __HAL_DMA_DISABLE(huart->hdmarx);
    } while (huart->hdmarx->Instance->CR & DMA_SxCR_EN);

    BSP_DMA_CLEAR_FLAG(huart->hdmarx);

    /*!< DMA stream x memory 0 address register   */
    huart->hdmarx->Instance->M0AR = (uint32_t)(buf1);
    /*!< DMA stream x memory 1 address register   */
    huart->hdmarx->Instance->M1AR = (uint32_t)(buf2);
    /*!< DMA stream x number of data register     */
    huart->hdmarx->Instance->NDTR = len;
    /*!< DMA stream x configuration register      */
    SET_BIT(huart->hdmarx->Instance->CR, DMA_SxCR_DBM);

    /*!< Enable the specified DMA Stream */
    __HAL_DMA_ENABLE(huart->hdmarx);
}

void usart_dma_tx(UART_HandleTypeDef *huart,
                  uint8_t *           data,
                  uint16_t            len)
{
    do
    {
        /* Disable the specified DMA Stream */
        __HAL_DMA_DISABLE(huart->hdmatx);
    } while (huart->hdmatx->Instance->CR & DMA_SxCR_EN);

    BSP_DMA_CLEAR_FLAG(huart->hdmatx);

    /*!< DMA stream x memory 0 address register   */
    huart->hdmatx->Instance->M0AR = (uint32_t)(data);
    /*!< DMA stream x number of data register     */
    huart->hdmatx->Instance->NDTR = len;

    /*!< Enable the specified DMA Stream */
    __HAL_DMA_ENABLE(huart->hdmatx);
}

void os_printf(const char *format, ...)
{
    static uint8_t buff[256U] = {0U};
    static va_list ap;
    static uint8_t n = 0U;
    va_start(ap, format);
    n = vsprintf((char *)buff, format, ap);
    va_end(ap);

    usart_dma_tx(&huart_os, buff, n);

    BSP_DMA_WAIT_TC(huart_os.hdmatx);
}

void os_putf(float x, uint8_t l)
{
    static char buff[32U] = {0U};

    uint8_t n = sprintf(buff, "%li", (long)x);

    if (l)
    {
        buff[n++] = '.';
    }
    for (uint8_t i = 0U; i < l && i < 32U; i++)
    {
        x *= 10;
        if (x < 0)
        {
            buff[n++] = '0' - (long)x % 10;
        }
        else
        {
            buff[n++] = '0' + (long)x % 10;
        }
    }

    usart_dma_tx(&huart_os, (uint8_t *)buff, n);

    BSP_DMA_WAIT_TC(huart_os.hdmatx);
}

/************************ (C) COPYRIGHT ngu ********************END OF FILE****/
