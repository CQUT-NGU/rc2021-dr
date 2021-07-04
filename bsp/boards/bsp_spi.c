/**
 * *****************************************************************************
 * @file         bsp_spi.c
 * @brief        spi of boards
 * @author       NGU
 * @date         20210427
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
 * *****************************************************************************
*/

#include "bsp_spi.h"

void spi_dma_init(SPI_HandleTypeDef *hspi)
{
    SET_BIT(hspi->Instance->CR2, SPI_CR2_RXDMAEN); /* Rx Buffer DMA Enable */
    SET_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN); /* Tx Buffer DMA Enable */

    /* Enable the SPI peripheral */
    __HAL_SPI_ENABLE(hspi);

    do
    {
        /* Disable the specified DMA Stream */
        __HAL_DMA_DISABLE(hspi->hdmarx);
    } while (hspi->hdmarx->Instance->CR & DMA_SxCR_EN);

    /* Clear the DMA Stream pending flags Transfer complete flag */
    BSP_DMA_CLEAR_TC(hspi->hdmarx);

    /* DMA stream x peripheral address register */
    hspi->hdmarx->Instance->PAR = (uint32_t) & (hspi->Instance->DR);

    /* Enable Transfer complete interrupt */
    __HAL_DMA_ENABLE_IT(hspi->hdmarx, DMA_IT_TC);

    do
    {
        /* Disable the specified DMA Stream */
        __HAL_DMA_DISABLE(hspi->hdmatx);
    } while (hspi->hdmatx->Instance->CR & DMA_SxCR_EN);

    /* Clear the DMA Stream pending flags Transfer complete flag */
    BSP_DMA_CLEAR_TC(hspi->hdmatx);

    /* DMA stream x peripheral address register */
    hspi->hdmatx->Instance->PAR = (uint32_t) & (hspi->Instance->DR);
}

void spi_dma_start(SPI_HandleTypeDef *hspi,
                   uint32_t buf_tx,
                   uint32_t buf_rx,
                   uint16_t len)
{
    do
    {
        /* Disable the specified DMA Stream */
        __HAL_DMA_DISABLE(hspi->hdmarx);
    } while (hspi->hdmarx->Instance->CR & DMA_SxCR_EN);

    do
    {
        /* Disable the specified DMA Stream */
        __HAL_DMA_DISABLE(hspi->hdmatx);
    } while (hspi->hdmatx->Instance->CR & DMA_SxCR_EN);

    /* Clear all DMA flag */
    BSP_DMA_CLEAR_FLAG(hspi->hdmarx);
    BSP_DMA_CLEAR_FLAG(hspi->hdmatx);

    /* DMA stream x memory 0 address register   */
    hspi->hdmarx->Instance->M0AR = buf_rx;
    /* DMA stream x number of data register     */
    hspi->hdmarx->Instance->NDTR = len;

    /* DMA stream x memory 0 address register   */
    hspi->hdmatx->Instance->M0AR = buf_tx;
    /* DMA stream x number of data register     */
    hspi->hdmatx->Instance->NDTR = len;

    /* Enable the specified DMA Stream */
    __HAL_DMA_ENABLE(hspi->hdmarx);
    /* Enable the specified DMA Stream */
    __HAL_DMA_ENABLE(hspi->hdmatx);
}

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
