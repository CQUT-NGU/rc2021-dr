/**
 * *****************************************************************************
 * @file         bsp_usart.c
 * @brief        usart of boards
 * @author       NGU
 * @date         20210426
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
 * *****************************************************************************
*/

#include "bsp_usart.h"

#include <stdarg.h>
#include <stdio.h>

extern void os_rx_irq(volatile void *, uint16_t);

uint8_t os_buf8[2][BUFSIZ_OS];
uint8_t *os_buf = *os_buf8;

#define BUFSIZ32 128
#define BUFSIZ16 (BUFSIZ32 << 1)
#define BUFSIZ8  (BUFSIZ32 << 2)

static va_list ap;
static float buf32[BUFSIZ32];
static uint8_t *pn = (uint8_t *)buf32;

#define FLAG_TX_DMA (1 << 0)
#define FLAG_RX_BUF (1 << 1)
#define FLAG_TX     (1 << 2)
static int8_t flag = 0;

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
                    uint16_t len)
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
                    uint16_t len)
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
                  void *buf1,
                  void *buf2,
                  uint16_t len)
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
                  const void *data,
                  uint16_t len)
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

void dma_printf(const char *fmt, ...)
{
    va_start(ap, fmt);
    uint16_t n = (uint16_t)vsprintf((char *)pn + 2, fmt, ap);
    va_end(ap);

    if (n)
    {
        uint8_t *p = pn + 2;
        uint8_t *buf = (uint8_t *)buf32;

        pn[0] = (uint8_t)n;
        pn[1] = (uint8_t)(n >> 8);

        if (pn == buf)
        {
            SET_BIT(flag, FLAG_TX_DMA);
            usart_dma_tx(&huart_os, p, n);
        }

        pn = p + n;
        if (pn - buf > BUFSIZ8 - 2)
        {
            pn = buf;
        }
    }
}

static void dma_printf_irq(void)
{
    static uint8_t *pi = (uint8_t *)buf32;

    uint16_t n = (uint16_t)(pi[0] + ((uint16_t)pi[1] << 8));
    uint8_t *p = pi + 2 + n;

    pi[0] = pi[1] = 0;
    if (p < pn)
    {
        pi = p;
        SET_BIT(flag, FLAG_TX_DMA);
        usart_dma_tx(&huart_os, p + 2, n);
    }
    else
    {
        pi = pn = (uint8_t *)buf32;
        CLEAR_BIT(flag, FLAG_TX_DMA);
    }
}

/**
 * @brief This function handles USART global interrupt.
*/
void OS_IRQHandler(void)
{
    if (huart_os.Instance->SR & UART_FLAG_RXNE) /* USART Status register*/
    {
        __HAL_UART_CLEAR_PEFLAG(&huart_os); /* Clears the UART PE pending flag */
    }
    else if (huart_os.Instance->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart_os); /* Clears the UART PE pending flag */

        /* Disable the specified DMA Stream */
        __HAL_DMA_DISABLE(huart_os.hdmarx);

        /* DMA stream x configuration register */
        if ((huart_os.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            /* Receive data length, length = set_data_length - remain_length */
            os_buf = os_buf8[0];
            os_buf[0] = (uint8_t)(BUFSIZ_OS - 1 - huart_os.hdmarx->Instance->NDTR);

            /* Reset set_data_lenght */
            huart_os.hdmarx->Instance->NDTR = BUFSIZ_OS - 1;

            /* Set memory buffer 1 */
            huart_os.hdmarx->Instance->CR |= DMA_SxCR_CT;

            /* Enable the specified DMA Stream */
            __HAL_DMA_ENABLE(huart_os.hdmarx);
        }
        else
        {
            /* Current memory buffer used is Memory 1 */

            /* Receive data length, length = set_data_length - remain_length */
            os_buf = os_buf8[1];
            os_buf[0] = (uint8_t)(BUFSIZ_OS - 1 - huart_os.hdmarx->Instance->NDTR);

            /* Reset set_data_lenght */
            huart_os.hdmarx->Instance->NDTR = BUFSIZ_OS - 1;

            /* Set memory buffer 0 */
            huart_os.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
        }

        /* Enable the specified DMA Stream */
        __HAL_DMA_ENABLE(huart_os.hdmarx);

        if (*os_buf > 1 && os_buf[2] == ':')
        {
            os_rx_irq(os_buf + 1, *os_buf);
        }
    }
    else if (huart_os.Instance->SR & UART_FLAG_TC)
    {
        __HAL_UART_CLEAR_FLAG(&huart_os, UART_FLAG_TC);

        if (READ_BIT(flag, FLAG_TX_DMA))
        {
            dma_printf_irq();
        }
    }
}

void os_dma_init(void)
{
    usart_dma_rx_init(&huart_os);
    usart_dma_rx(&huart_os, os_buf8[0] + 1, os_buf8[1] + 1, BUFSIZ_OS - 1);
}

void os_printf(const char *format, ...)
{
    va_start(ap, format);
    uint16_t n = (uint16_t)vsprintf((char *)buf32, format, ap);
    va_end(ap);

    /* USART transmit by DMA Stream */
    usart_dma_tx(&huart_os, (uint8_t *)buf32, n);

    /* Wait Complete Transmit flag to be set */
    BSP_DMA_WAIT_TC(huart_os.hdmatx);
}

void os_justfloat(uint8_t n, ...)
{
    float *p = buf32;
    float *d = buf32 + n;

    va_start(ap, n);
    while (p != d)
    {
        *p++ = (float)va_arg(ap, double);
    }
    *p = 1.0F / 0.0F;
    va_end(ap);

    /* USART transmit by DMA Stream */
    usart_dma_tx(&huart_os, (void *)buf32, (uint16_t)(sizeof(float) * (n + 1U)));

    /* Wait Complete Transmit flag to be set */
    BSP_DMA_WAIT_TC(huart_os.hdmatx);
}

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
