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

/* Includes ------------------------------------------------------------------*/
#include "ctrl_pc.h"

/* Private includes ----------------------------------------------------------*/
#include "bsp_usart.h"
#include "main.h"

#include <stdlib.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#undef hpc
#define hpc huart_os

/* Private includes ----------------------------------------------------------*/
extern UART_HandleTypeDef hpc;

/* Private macro -------------------------------------------------------------*/

#undef PC_IS_FLOAT
#define PC_IS_FLOAT(x) \
    ((x >= '0' &&      \
      x <= '9') ||     \
     x == '+' ||       \
     x == '-' ||       \
     x == '.')

#undef PC_DEAL_BUF
#define PC_DEAL_BUF(x)                       \
    while (i != len && !PC_IS_FLOAT(buf[i])) \
    {                                        \
        i++;                                 \
    }                                        \
    if (i == len)                            \
    {                                        \
        break;                               \
    }                                        \
    j = 0U;                                  \
    while (i != len && PC_IS_FLOAT(buf[i]))  \
    {                                        \
        buff[j++] = buf[i++];                \
    }                                        \
    buff[j] = 0;                             \
    x       = (float)atof((char *)buff);

/* Private variables ---------------------------------------------------------*/

static ctrl_pc_t pc; /* pc control data */

static uint8_t pc_rx_buf[2][PC_RX_BUFSIZ];

/* Private function prototypes -----------------------------------------------*/

static void ctrl_pc(volatile const uint8_t *buf,
                    uint16_t                len);

/* Private user code ---------------------------------------------------------*/

void ctrl_pc_init(void)
{
    usart_dma_rx_init(&hpc);
    usart_dma_rx(&hpc, pc_rx_buf[0], pc_rx_buf[1], PC_RX_BUFSIZ);
}

ctrl_pc_t *ctrl_pc_point(void)
{
    return &pc;
}

extern unsigned char  dma_print_buf[256];
extern unsigned char *dma_print_pn;

/**
 * @brief This function handles USART global interrupt.
*/
void PC_IRQHandler(void)
{
    if (hpc.Instance->SR & UART_FLAG_RXNE) /*!< USART Status register*/
    {
        __HAL_UART_CLEAR_PEFLAG(&hpc); /* Clears the UART PE pending flag */
    }
    else if (hpc.Instance->SR & UART_FLAG_TC)
    {
        __HAL_UART_CLEAR_FLAG(&hpc, UART_FLAG_TC);

        static unsigned char *dma_print_pin = dma_print_buf;

        unsigned char *p = dma_print_pin + 1 + *dma_print_pin;

        *dma_print_pin = 0;
        if (p < dma_print_pn)
        {
            dma_print_pin = p;
            usart_dma_tx(&huart_os, p + 1, *p);
        }
        else
        {
            dma_print_pin = dma_print_pn = dma_print_buf;

            *dma_print_buf = 0;
        }
    }
    else if (hpc.Instance->SR & UART_FLAG_IDLE)
    {
        static uint16_t len_rx = 0;
        static uint8_t *buf_rx = 0;

        __HAL_UART_CLEAR_PEFLAG(&hpc); /* Clears the UART PE pending flag */

        /*!< DMA stream x configuration register */
        if ((hpc.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            /* Disable the specified DMA Stream */
            __HAL_DMA_DISABLE(hpc.hdmarx);

            /* Receive data length, length = set_data_length - remain_length */
            len_rx = (uint8_t)(PC_RX_BUFSIZ - hpc.hdmarx->Instance->NDTR);
            buf_rx = pc_rx_buf[0];

            /* Reset set_data_lenght */
            hpc.hdmarx->Instance->NDTR = PC_RX_BUFSIZ;

            /* Set memory buffer 1 */
            hpc.hdmarx->Instance->CR |= DMA_SxCR_CT;

            /* Enable the specified DMA Stream */
            __HAL_DMA_ENABLE(hpc.hdmarx);
        }
        else
        {
            /* Current memory buffer used is Memory 1 */

            /* Disable the specified DMA Stream */
            __HAL_DMA_DISABLE(hpc.hdmarx);

            /* Receive data length, length = set_data_length - remain_length */
            len_rx = (uint8_t)(PC_RX_BUFSIZ - hpc.hdmarx->Instance->NDTR);
            buf_rx = pc_rx_buf[1];

            /* Reset set_data_lenght */
            hpc.hdmarx->Instance->NDTR = PC_RX_BUFSIZ;

            /* Set memory buffer 0 */
            hpc.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);

            /* Enable the specified DMA Stream */
            __HAL_DMA_ENABLE(hpc.hdmarx);
        }

        if (len_rx > 1 && buf_rx[1] == ':')
        {
            ctrl_pc(buf_rx, len_rx);
        }
    }
}

static void ctrl_pc(volatile const uint8_t *buf,
                    uint16_t                len)
{
    if ('A' <= *buf && *buf <= 'Z')
    {
        pc.c = *buf;
        buf += 2;

        float buf32[3];

        uint8_t *pi = (uint8_t *)buf32;
        uint8_t *pd = (uint8_t *)buf32 + 12;
        while (pi != pd)
        {
            *pi++ = *buf++;
        }

        pc.x = buf32[0];
        pc.y = buf32[1];
        pc.z = buf32[2];
        return;
    }
    else if ('a' <= *buf && *buf <= 'z')
    {
        do
        {
            uint8_t  buff[32] = {0};
            uint16_t i        = 2U;
            uint8_t  j        = 0U;

            PC_DEAL_BUF(pc.x);
            PC_DEAL_BUF(pc.y);
            PC_DEAL_BUF(pc.z);

            pc.c = buf[0];
            return;
        } while (0);
    }

    pc.c = 0;
}

/************************ (C) COPYRIGHT ngu ********************END OF FILE****/
