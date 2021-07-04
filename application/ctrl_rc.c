/**
 * *****************************************************************************
 * @file         ctrl_rc.c
 * @brief        control by remote
 * @author       NGU
 * @date         20210427
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
 * *****************************************************************************
*/

#include "ctrl_rc.h"

#undef hrc
#define hrc huart_rc

extern UART_HandleTypeDef hrc;

static ctrl_rc_t rc; /* remote control data */

/* receive data, 18 bytes one frame, but set 36 bytes */
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/**
 * @brief        Remote control protocol resolution
 * @param[in]    buf: raw data point
*/
static void sbus_to_rc(const void *buf);

void ctrl_rc_init(void)
{
    usart_dma_rx_init(&hrc);
    usart_dma_rx(&hrc, sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

ctrl_rc_t *ctrl_rc_point(void)
{
    return &rc;
}

/**
 * @brief This function handles USART global interrupt.
*/
void RC_IRQHandler(void)
{
    if (hrc.Instance->SR & UART_FLAG_RXNE) /* USART Status register*/
    {
        __HAL_UART_CLEAR_PEFLAG(&hrc); /* Clears the UART PE pending flag */
    }
    else if (hrc.Instance->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&hrc); /* Clears the UART PE pending flag */

        void *rx_p = 0;

        /* Disable the specified DMA Stream */
        __HAL_DMA_DISABLE(hrc.hdmarx);

        /* Receive data length, length = set_data_length - remain_length */
        uint16_t rx_l = (uint16_t)(SBUS_RX_BUF_NUM - hrc.hdmarx->Instance->NDTR);

        /* DMA stream x configuration register */
        if ((hrc.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */
            rx_p = (void *)hrc.hdmarx->Instance->M0AR;
            /* Set memory buffer 1 */
            hrc.hdmarx->Instance->CR |= DMA_SxCR_CT;
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            rx_p = sbus_rx_buf[1];

            /* Set memory buffer 0 */
            hrc.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
        }

        /* Reset set_data_lenght */
        hrc.hdmarx->Instance->NDTR = SBUS_RX_BUF_NUM;

        /* Enable the specified DMA Stream */
        __HAL_DMA_ENABLE(hrc.hdmarx);

        if (rx_l == RC_FRAME_LENGTH)
        {
            sbus_to_rc(rx_p);
        }
    }
}

static void sbus_to_rc(const void *buf)
{
    const uint8_t *p = (const uint8_t *)buf;

    /* Channel 0 */
    rc.rc.ch[0] = 0x7FF & (int16_t)((p[1] << 8) | p[0]);
    /* Channel 1 */
    rc.rc.ch[1] = 0x7FF & (int16_t)((p[1] >> 3) | (p[2] << 5));
    /* Channel 2 */
    rc.rc.ch[2] = 0x7FF & (int16_t)((p[2] >> 6) | (p[3] << 2) | (p[4] << 10));
    /* Channel 3 */
    rc.rc.ch[3] = 0x7FF & (int16_t)((p[4] >> 1) | (p[5] << 7));
    /* Switch left */
    rc.rc.s[0] = ((p[5] >> 4) & 0x3);
    /* Switch right */
    rc.rc.s[1] = ((p[5] >> 4) & 0xC) >> 2;
    /* Mouse X axis */
    rc.mouse.x = (int16_t)(p[6] | (p[7] << 8));
    /* Mouse Y axis */
    rc.mouse.y = (int16_t)(p[8] | (p[9] << 8));
    /* Mouse Z axis */
    rc.mouse.z = (int16_t)(p[10] | (p[11] << 8));
    /* Mouse Left Is Press ? */
    rc.mouse.press_l = p[12];
    /* Mouse Right Is Press ? */
    rc.mouse.press_r = p[13];
    /* KeyBoard value */
    rc.key.v = (uint16_t)(p[14] | (p[15] << 8));
    /* NULL */
    rc.rc.ch[4] = (int16_t)(p[16] | (p[17] << 8));

    rc.rc.ch[0] = (int16_t)(rc.rc.ch[0] - RC_CH_VALUE_OFFSET);
    rc.rc.ch[1] = (int16_t)(rc.rc.ch[1] - RC_CH_VALUE_OFFSET);
    rc.rc.ch[2] = (int16_t)(rc.rc.ch[2] - RC_CH_VALUE_OFFSET);
    rc.rc.ch[3] = (int16_t)(rc.rc.ch[3] - RC_CH_VALUE_OFFSET);
    rc.rc.ch[4] = (int16_t)(rc.rc.ch[4] - RC_CH_VALUE_OFFSET);
}

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
