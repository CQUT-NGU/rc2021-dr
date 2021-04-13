/**
 * *****************************************************************************
 * @file         bsp_spi.c/h
 * @brief        spi of boards
 * @author       ngu
 * @date         20210101
 * @version      1
 * @copyright    Copyright (c) 2021
 * @code         utf-8                                                  @endcode
 * @details      
 *               SPI1_SCK  ------> PB3
 *               SPI1_MOSI ------> PA7
 *               SPI1_MISO ------> PB4
 * *****************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "bsp_spi.h"

/* Private includes ----------------------------------------------------------*/
#include "main.h"

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/

void spi_dma_init(SPI_HandleTypeDef *hspi)
{
    SET_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN); /*!< Tx Buffer DMA Enable */
    SET_BIT(hspi->Instance->CR2, SPI_CR2_RXDMAEN); /*!< Rx Buffer DMA Enable */

    /*!< Enable the SPI peripheral */
    __HAL_SPI_ENABLE(hspi);
    /*!< Enable Transfer complete interrupt */
    __HAL_DMA_ENABLE_IT(hspi->hdmarx, DMA_IT_TC);

    do
    {
        /*!< Disable the specified DMA Stream */
        __HAL_DMA_DISABLE(hspi->hdmarx);
    } while (hspi->hdmarx->Instance->CR & DMA_SxCR_EN);

    /*!< Clear the DMA Stream pending flags Transfer complete flag */
    __HAL_DMA_CLEAR_FLAG(hspi->hdmarx,
                         __HAL_DMA_GET_TC_FLAG_INDEX(hspi->hdmarx));

    /*!< DMA stream x peripheral address register */
    hspi->hdmarx->Instance->PAR = (uint32_t) & (hspi->Instance->DR);

    do
    {
        /*!< Disable the specified DMA Stream */
        __HAL_DMA_DISABLE(hspi->hdmatx);
    } while (hspi->hdmatx->Instance->CR & DMA_SxCR_EN);

    /*!< Clear the DMA Stream pending flags Transfer complete flag */
    __HAL_DMA_CLEAR_FLAG(hspi->hdmatx,
                         __HAL_DMA_GET_TC_FLAG_INDEX(hspi->hdmatx));

    /*!< DMA stream x peripheral address register */
    hspi->hdmarx->Instance->PAR = (uint32_t) & (hspi->Instance->DR);
}

void spi_dma_start(SPI_HandleTypeDef *hspi,
                   uint32_t           buf_tx,
                   uint32_t           buf_rx,
                   uint16_t           len)
{
    do
    {
        /*!< Disable the specified DMA Stream */
        __HAL_DMA_DISABLE(hspi->hdmarx);
    } while (hspi->hdmarx->Instance->CR & DMA_SxCR_EN);

    do
    {
        /*!< Disable the specified DMA Stream */
        __HAL_DMA_DISABLE(hspi->hdmatx);
    } while (hspi->hdmatx->Instance->CR & DMA_SxCR_EN);

    /*!< Transfer complete interrupt mask */
    __HAL_DMA_CLEAR_FLAG(hspi->hdmarx,
                         __HAL_DMA_GET_TC_FLAG_INDEX(hspi->hdmarx));
    /*!< Half transfer complete interrupt mask */
    __HAL_DMA_CLEAR_FLAG(hspi->hdmarx,
                         __HAL_DMA_GET_HT_FLAG_INDEX(hspi->hdmarx));
    /*!< Transfer error interrupt mask */
    __HAL_DMA_CLEAR_FLAG(hspi->hdmarx,
                         __HAL_DMA_GET_TE_FLAG_INDEX(hspi->hdmarx));
    /*!< Direct mode error interrupt */
    __HAL_DMA_CLEAR_FLAG(hspi->hdmarx,
                         __HAL_DMA_GET_DME_FLAG_INDEX(hspi->hdmarx));
    /*!< FIFO error interrupt mask */
    __HAL_DMA_CLEAR_FLAG(hspi->hdmarx,
                         __HAL_DMA_GET_FE_FLAG_INDEX(hspi->hdmarx));

    /*!< Transfer complete interrupt mask */
    __HAL_DMA_CLEAR_FLAG(hspi->hdmatx,
                         __HAL_DMA_GET_TC_FLAG_INDEX(hspi->hdmatx));
    /*!< Half transfer complete interrupt mask */
    __HAL_DMA_CLEAR_FLAG(hspi->hdmatx,
                         __HAL_DMA_GET_HT_FLAG_INDEX(hspi->hdmatx));
    /*!< Transfer error interrupt mask */
    __HAL_DMA_CLEAR_FLAG(hspi->hdmatx,
                         __HAL_DMA_GET_TE_FLAG_INDEX(hspi->hdmatx));
    /*!< Direct mode error interrupt */
    __HAL_DMA_CLEAR_FLAG(hspi->hdmatx,
                         __HAL_DMA_GET_DME_FLAG_INDEX(hspi->hdmatx));
    /*!< FIFO error interrupt mask */
    __HAL_DMA_CLEAR_FLAG(hspi->hdmatx,
                         __HAL_DMA_GET_FE_FLAG_INDEX(hspi->hdmatx));

    /*!< DMA stream x memory 0 address register   */
    hspi->hdmarx->Instance->M0AR = buf_rx;
    /*!< DMA stream x number of data register     */
    hspi->hdmarx->Instance->NDTR = len;

    /*!< DMA stream x memory 0 address register   */
    hspi->hdmatx->Instance->M0AR = buf_tx;
    /*!< DMA stream x number of data register     */
    hspi->hdmatx->Instance->NDTR = len;

    /*!< Enable the specified DMA Stream */
    __HAL_DMA_ENABLE(hspi->hdmarx);
    /*!< Enable the specified DMA Stream */
    __HAL_DMA_ENABLE(hspi->hdmatx);
}

/************************ (C) COPYRIGHT ngu ********************END OF FILE****/
