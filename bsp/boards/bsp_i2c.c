/**
 * *****************************************************************************
 * @file         bsp_i2c.c
 * @brief        i2c of boards
 * @author       NGU
 * @date         20210427
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
 * @details
 * *****************************************************************************
*/

#include "bsp_i2c.h"

void i2c_reset(I2C_HandleTypeDef *hi2c)
{
    /*!< Software Reset */
    SET_BIT(hi2c->Instance->CR1, I2C_CR1_SWRST);
    CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_SWRST);

    if (HAL_I2C_Init(hi2c) != HAL_OK)
    {
        Error_Handler();
    }
}

void i2c_master_transmit(I2C_HandleTypeDef *hi2c,
                         uint16_t addr,
                         uint8_t *data,
                         uint16_t len)
{
    HAL_I2C_Master_Transmit(hi2c, addr, data, len, 100U);
}

uint8_t i2c_check_ack(I2C_HandleTypeDef *hi2c,
                      uint16_t addr)
{
    if ((hi2c->Instance->CR2 & I2C_CR2_DMAEN) &&
        ((hi2c->hdmatx != NULL && hi2c->hdmatx->Instance->NDTR != 0) ||
         (hi2c->hdmarx != NULL && hi2c->hdmarx->Instance->NDTR != 0)))
    {
        return I2C_ACK;
    }
    else
    {
        uint16_t timeout = 0;

        timeout = 0;
        while (hi2c->Instance->SR2 & 0x02)
        {
            timeout++;
            if (timeout > 100)
            {
                SET_BIT(hi2c->Instance->CR1, I2C_CR1_STOP);
                return I2C_NO_ACK;
            }
        }

        CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_POS);

        SET_BIT(hi2c->Instance->CR1, I2C_CR1_START);

        timeout = 0;
        while (!(hi2c->Instance->SR1 & 0x01))
        {
            timeout++;
            if (timeout > 100)
            {
                SET_BIT(hi2c->Instance->CR1, I2C_CR1_STOP);
                return I2C_NO_ACK;
            }
        }

        hi2c->Instance->DR = I2C_7BIT_ADD_WRITE(addr);

        timeout = 0;
        while (!(hi2c->Instance->SR1 & 0x02))
        {
            timeout++;
            if (timeout > 500)
            {
                SET_BIT(hi2c->Instance->CR1, I2C_CR1_STOP);
                return I2C_NO_ACK;
            }
        }

        do
        {
            __IO uint32_t tmpreg = 0x00U;

            tmpreg = hi2c->Instance->SR1;
            tmpreg = hi2c->Instance->SR2;

            UNUSED(tmpreg);
        } while (0);

        timeout = 0;
        while (!(hi2c->Instance->SR1 & 0x80))
        {
            timeout++;
            if (timeout > 500)
            {
                SET_BIT(hi2c->Instance->CR1, I2C_CR1_STOP);
                return I2C_NO_ACK;
            }
        }

        SET_BIT(hi2c->Instance->CR1, I2C_CR1_STOP);
        return I2C_ACK;
    }
}

void i2c_dma_tx_init(I2C_HandleTypeDef *hi2c)
{
    /*!<DMA Requests Enable     */
    SET_BIT(hi2c->Instance->CR2, I2C_CR2_DMAEN);

    do
    {
        /* Disable the specified DMA Stream */
        __HAL_DMA_DISABLE(hi2c->hdmatx);
    } while (hi2c->hdmatx->Instance->CR & DMA_SxCR_EN);

    /*!< DMA stream x peripheral address register */
    hi2c->hdmatx->Instance->PAR = (uint32_t)(&hi2c->Instance->DR);

    /* Clear Complete Transmit flag */
    BSP_DMA_CLEAR_TC(hi2c->hdmarx);

    /* Enable the specified DMA Stream interrupts */
    __HAL_DMA_ENABLE_IT(hi2c->hdmatx, DMA_IT_TC);
}

void i2c_dma_tx(I2C_HandleTypeDef *hi2c,
                uint32_t addr,
                uint16_t ndtr)
{
    do
    {
        /* Disable the specified DMA Stream */
        __HAL_DMA_DISABLE(hi2c->hdmatx);
    } while (hi2c->hdmatx->Instance->CR & DMA_SxCR_EN);

    /* Clear Complete Transmit flag */
    BSP_DMA_CLEAR_TC(hi2c->hdmatx);

    /*!< DMA stream x memory 0 address register   */
    hi2c->hdmatx->Instance->M0AR = addr;
    /*!< DMA stream x number of data register     */
    hi2c->hdmatx->Instance->NDTR = ndtr;

    /* Enable the specified DMA Stream */
    __HAL_DMA_ENABLE(hi2c->hdmatx);
}

HAL_StatusTypeDef i2c_dma_tx_start(I2C_HandleTypeDef *hi2c,
                                   uint16_t DevAddress)
{
    uint16_t timeout = 0;

    if ((hi2c->Instance->CR1 & I2C_CR1_PE) != I2C_CR1_PE)
    {
        hi2c->Instance->CR1 |= I2C_CR1_PE;
    }

    timeout = 0;
    while (hi2c->Instance->SR2 & 0x02)
    {
        timeout++;
        if (timeout > 100)
        {
            SET_BIT(hi2c->Instance->CR1, I2C_CR1_STOP);
            return HAL_BUSY;
        }
    }

    CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_POS);

    SET_BIT(hi2c->Instance->CR1, I2C_CR1_START);

    timeout = 0;
    while (!(hi2c->Instance->SR1 & 0x01))
    {
        timeout++;
        if (timeout > 100)
        {
            SET_BIT(hi2c->Instance->CR1, I2C_CR1_STOP);
            return HAL_ERROR;
        }
    }

    hi2c->Instance->DR = I2C_7BIT_ADD_WRITE(DevAddress);

    timeout = 0;
    while (!(hi2c->Instance->SR1 & 0x02))
    {
        timeout++;
        if (timeout > 500)
        {
            SET_BIT(hi2c->Instance->CR1, I2C_CR1_STOP);
            return HAL_ERROR;
        }
    }

    do
    {
        __IO uint32_t tmpreg = 0x00U;

        tmpreg = hi2c->Instance->SR1;
        tmpreg = hi2c->Instance->SR2;

        UNUSED(tmpreg);
    } while (0);

    timeout = 0;
    while (!(hi2c->Instance->SR1 & 0x80))
    {
        timeout++;
        if (timeout > 500)
        {
            SET_BIT(hi2c->Instance->CR1, I2C_CR1_STOP);
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

void i2c_dma_transmit(I2C_HandleTypeDef *hi2c,
                      uint16_t DevAddress,
                      uint8_t *pData,
                      uint16_t Size)
{
    if (i2c_dma_tx_start(hi2c, DevAddress) == HAL_OK)
    {
        i2c_dma_tx(hi2c, (uint32_t)pData, Size);
    }
}

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
