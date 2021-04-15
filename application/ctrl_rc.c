/**
 * *****************************************************************************
 * @file         ctrl_rc.c/h
 * @brief        control by remote
 * @author       ngu
 * @date         20210427
 * @version      1
 * @copyright    Copyright (c) 2021
 * @code         utf-8                                                  @endcode
 * *****************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "ctrl_rc.h"

/* Private includes ----------------------------------------------------------*/
#include "bsp_usart.h"
#include "main.h"

/* Private define ------------------------------------------------------------*/
#undef hrc
#define hrc huart_rc

/* Private includes ----------------------------------------------------------*/
extern UART_HandleTypeDef hrc;

/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static ctrl_rc_t rc; /* remote control data */

/* receive data, 18 bytes one frame, but set 36 bytes */
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief        Remote control protocol resolution
 * @param[in]    buf: raw data point
 * @param[out]   rc:  remote control data struct point
*/
static void sbus_to_rc(volatile const uint8_t *buf,
                       ctrl_rc_t *             rc);

/* Private user code ---------------------------------------------------------*/

void ctrl_rc_init(void)
{
    usart_dma_rx_init(&hrc);
    usart_dma_rx(&hrc, sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

const ctrl_rc_t *ctrl_rc_point(void)
{
    return &rc;
}

/**
 * @brief This function handles USART global interrupt.
*/
void RC_IRQHandler(void)
{
    if (hrc.Instance->SR & UART_FLAG_RXNE) /*!< USART Status register*/
    {
        __HAL_UART_CLEAR_PEFLAG(&hrc); /* Clears the UART PE pending flag */
    }
    else if (hrc.Instance->SR & UART_FLAG_IDLE)
    {
        static uint16_t len_rx = 0;

        __HAL_UART_CLEAR_PEFLAG(&hrc); /* Clears the UART PE pending flag */

        /*!< DMA stream x configuration register */
        if ((hrc.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            /* Disable the specified DMA Stream */
            __HAL_DMA_DISABLE(hrc.hdmarx);

            /* Receive data length, length = set_data_length - remain_length */
            len_rx = SBUS_RX_BUF_NUM - hrc.hdmarx->Instance->NDTR;

            /* Reset set_data_lenght */
            hrc.hdmarx->Instance->NDTR = SBUS_RX_BUF_NUM;

            /* Set memory buffer 1 */
            hrc.hdmarx->Instance->CR |= DMA_SxCR_CT;

            /* Enable the specified DMA Stream */
            __HAL_DMA_ENABLE(hrc.hdmarx);

            if (len_rx == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */

            /* Disable the specified DMA Stream */
            __HAL_DMA_DISABLE(hrc.hdmarx);

            /* Receive data length, length = set_data_length - remain_length */
            len_rx = SBUS_RX_BUF_NUM - hrc.hdmarx->Instance->NDTR;

            /* Reset set_data_lenght */
            hrc.hdmarx->Instance->NDTR = SBUS_RX_BUF_NUM;

            /* Set memory buffer 0 */
            hrc.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);

            /* Enable the specified DMA Stream */
            __HAL_DMA_ENABLE(hrc.hdmarx);

            if (len_rx == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[1], &rc);
            }
        }
    }
}

static void sbus_to_rc(volatile const uint8_t *buf, ctrl_rc_t *rc)
{
    /*!< Channel 0 */
    rc->rc.ch[0] = 0x7ff & ((buf[1] << 8) | buf[0]);
    /*!< Channel 1 */
    rc->rc.ch[1] = 0x7ff & ((buf[1] >> 3) | (buf[2] << 5));
    /*!< Channel 2 */
    rc->rc.ch[2] = 0x7ff & ((buf[2] >> 6) | (buf[3] << 2) | (buf[4] << 10));
    /*!< Channel 3 */
    rc->rc.ch[3] = 0x7ff & ((buf[4] >> 1) | (buf[5] << 7));
    /*!< Switch left */
    rc->rc.s[0] = ((buf[5] >> 4) & 0x3);
    /*!< Switch right */
    rc->rc.s[1] = ((buf[5] >> 4) & 0xC) >> 2;
    /*!< Mouse X axis */
    rc->mouse.x = buf[6] | (buf[7] << 8);
    /*!< Mouse Y axis */
    rc->mouse.y = buf[8] | (buf[9] << 8);
    /*!< Mouse Z axis */
    rc->mouse.z = buf[10] | (buf[11] << 8);
    /*!< Mouse Left Is Press ? */
    rc->mouse.press_l = buf[12];
    /*!< Mouse Right Is Press ? */
    rc->mouse.press_r = buf[13];
    /*!< KeyBoard value */
    rc->key.v = buf[14] | (buf[15] << 8);
    /*!< NULL */
    rc->rc.ch[4] = buf[16] | (buf[17] << 8);

    rc->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

/************************ (C) COPYRIGHT ngu ********************END OF FILE****/
