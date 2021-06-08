/**
 * *****************************************************************************
 * @file         bsp_usart.c/h
 * @brief        usart of boards
 * @author       tqfx
 * @date         20210426
 * @version      1
 * @copyright    Copyright (c) 2021
 * @code         utf-8                                                  @endcode
 * *****************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "bsp_usart.h"

/* Private includes ----------------------------------------------------------*/
#include "main.h"

#include <stdarg.h>
#include <stdio.h>

/* Private define ------------------------------------------------------------*/

#define BUFSIZ8  256
#define BUFSIZ32 16

/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static va_list ap;
static uint8_t cnt_8 = 0U;
static uint8_t buf_8[BUFSIZ8];
static uint8_t cnt_32 = 0U;
static float   buf_32[BUFSIZ32];
static uint8_t tail[sizeof(float)] = {0x00U, 0x00U, 0x80U, 0x7FU};

/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/

void usart_disable(UART_HandleTypeDef *huart)
{
    /* Disable UART */
    __HAL_UART_DISABLE(huart);
}

void usart_enable(UART_HandleTypeDef *huart)
{
    /* Enable UART */
    __HAL_UART_ENABLE(huart);
}

void usart_dma_rerx(UART_HandleTypeDef *huart,
                    uint16_t            len)
{
    /* Disable UART */
    __HAL_UART_DISABLE(huart);

    do
    {
        /* Disable the specified DMA Stream */
        __HAL_DMA_DISABLE(huart->hdmarx);
    } while (huart->hdmarx->Instance->CR & DMA_SxCR_EN);

    /* Clear all flag */
    BSP_DMA_CLEAR_FLAG(huart->hdmarx);

    /* DMA stream x number of data register */
    huart->hdmarx->Instance->NDTR = len;

    /* Enable the specified DMA Stream */
    __HAL_DMA_ENABLE(huart->hdmarx);
    /* Enable UART */
    __HAL_UART_ENABLE(huart);
}

void usart_dma_retx(UART_HandleTypeDef *huart,
                    uint16_t            len)
{
    /* Disable UART */
    __HAL_UART_DISABLE(huart);

    do
    {
        /* Disable the specified DMA Stream */
        __HAL_DMA_DISABLE(huart->hdmatx);
    } while (huart->hdmatx->Instance->CR & DMA_SxCR_EN);

    /* Clear all flag */
    BSP_DMA_CLEAR_FLAG(huart->hdmatx);

    /* DMA stream x number of data register */
    huart->hdmatx->Instance->NDTR = len;

    /* Enable the specified DMA Stream */
    __HAL_DMA_ENABLE(huart->hdmatx);
    /* Enable UART */
    __HAL_UART_ENABLE(huart);
}

void usart_dma_rx_init(UART_HandleTypeDef *huart)
{
    /* DMA Enable Receiver */
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

    /* Clear all flag */
    BSP_DMA_CLEAR_FLAG(huart->hdmarx);

    /* DMA stream x peripheral address register */
    huart->hdmarx->Instance->PAR = (uint32_t) & (huart->Instance->DR);
}

void usart_dma_tx_init(UART_HandleTypeDef *huart)
{
    /* DMA Enable Transmitter */
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAT);

    /* Enable UART */
    __HAL_UART_ENABLE(huart);
    /* Enable transmission complete interrupt */
    __HAL_UART_ENABLE_IT(huart, UART_IT_TC);

    do
    {
        /* Disable the specified DMA Stream */
        __HAL_DMA_DISABLE(huart->hdmatx);
    } while (huart->hdmatx->Instance->CR & DMA_SxCR_EN);

    /* Clear all flag */
    BSP_DMA_CLEAR_FLAG(huart->hdmatx);

    /* DMA stream x peripheral address register */
    huart->hdmatx->Instance->PAR = (uint32_t) & (huart->Instance->DR);
}

void usart_dma_init(UART_HandleTypeDef *huart)
{
    /* DMA Enable Receiver and Transmitter */
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

    /* Clear all flag */
    BSP_DMA_CLEAR_FLAG(huart->hdmarx);

    /* DMA stream x memory 0 address register   */
    huart->hdmarx->Instance->M0AR = (uint32_t)(buf1);
    /* DMA stream x memory 1 address register   */
    huart->hdmarx->Instance->M1AR = (uint32_t)(buf2);
    /* DMA stream x number of data register */
    huart->hdmarx->Instance->NDTR = len;
    /* DMA stream x configuration register  */
    SET_BIT(huart->hdmarx->Instance->CR, DMA_SxCR_DBM);

    /* Enable the specified DMA Stream */
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

    /* Clear all flag */
    BSP_DMA_CLEAR_FLAG(huart->hdmatx);

    /* DMA stream x memory 0 address register   */
    huart->hdmatx->Instance->M0AR = (uint32_t)(data);
    /* DMA stream x number of data register */
    huart->hdmatx->Instance->NDTR = len;

    /* Enable the specified DMA Stream */
    __HAL_DMA_ENABLE(huart->hdmatx);
}

unsigned char dma_print_buf[256] = {0};

unsigned char *dma_print_pn = dma_print_buf;

void dma_printf(const char *fmt, ...)
{
    va_start(ap, fmt);
    *dma_print_pn = vsprintf((char *)dma_print_pn + 1, fmt, ap);
    va_end(ap);

    if (*dma_print_pn)
    {
        unsigned char *p = dma_print_pn + 1;

        if (dma_print_pn == dma_print_buf)
        {
            usart_dma_tx(&huart_os, p, *dma_print_pn);
        }

        dma_print_pn = p + *dma_print_pn;
        if (dma_print_pn - dma_print_buf > 255)
        {
            dma_print_pn = dma_print_buf;
        }
    }
}

void os_printf(const char *format, ...)
{
    va_start(ap, format);
    cnt_8 = vsprintf((char *)buf_8, format, ap);
    va_end(ap);

    /* USART transmit by DMA Stream */
    usart_dma_tx(&huart_os, buf_8, cnt_8);

    /* Wait Complete Transmit flag to be set */
    BSP_DMA_WAIT_TC(huart_os.hdmatx);
}

void os_pushf(float f)
{
    buf_32[cnt_32++] = f;
}

void os_tail(void)
{
    buf_32[cnt_32++] = (*(float *)tail);

    /* USART transmit by DMA Stream */
    usart_dma_tx(&huart_os, (uint8_t *)buf_32, sizeof(float) * cnt_32);
    cnt_32 = 0U;

    /* Wait Complete Transmit flag to be set */
    BSP_DMA_WAIT_TC(huart_os.hdmatx);
}

void os_justfloat(uint8_t n, ...)
{
    va_list ap;
    cnt_32 = 0U;

    va_start(ap, n);
    while (cnt_32 != n)
    {
        buf_32[cnt_32++] = (float)va_arg(ap, double);
    }
    buf_32[cnt_32++] = (*(float *)tail);
    va_end(ap);

    /* USART transmit by DMA Stream */
    usart_dma_tx(&huart_os, (uint8_t *)buf_32, sizeof(float) * cnt_32);
    cnt_32 = 0U;

    /* Wait Complete Transmit flag to be set */
    BSP_DMA_WAIT_TC(huart_os.hdmatx);
}

void os_putf(float x, uint8_t l)
{
    cnt_8 = sprintf((char *)buf_8, "%li", (long)x);

    if (l)
    {
        buf_8[cnt_8++] = '.';
    }

    for (uint8_t i = 0U; i < l; ++i)
    {
        x *= 10;
        if (x < 0)
        {
            buf_8[cnt_8++] = '0' - (long)x % 10;
        }
        else
        {
            buf_8[cnt_8++] = '0' + (long)x % 10;
        }
    }

    /* USART transmit by DMA Stream */
    usart_dma_tx(&huart_os, buf_8, cnt_8);

    /* Wait Complete Transmit flag to be set */
    BSP_DMA_WAIT_TC(huart_os.hdmatx);
}

/************************ (C) COPYRIGHT tqfx *******************END OF FILE****/
