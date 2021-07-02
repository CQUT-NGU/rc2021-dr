/**
 * *****************************************************************************
 * @file         bsp_i2c.c/h
 * @brief        i2c of boards
 * @author       ngu
 * @date         20210427
 * @version      1
 * @copyright    Copyright (c) 2021
 * @code         utf-8                                                  @endcode
 * @details
 * *****************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_I2C_H__
#define __BSP_I2C_H__

/* Includes ------------------------------------------------------------------*/
#include "bsp_dma.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
#include <stdint.h>

extern I2C_HandleTypeDef hi2c3;

/* Exported constants --------------------------------------------------------*/

#define I2C_ACK    1
#define I2C_NO_ACK 0

/* Exported macro ------------------------------------------------------------*/
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

/* Exported types ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/

__BEGIN_DECLS

extern void i2c_reset(I2C_HandleTypeDef *hi2c);

extern void i2c_master_transmit(I2C_HandleTypeDef *hi2c,
                                uint16_t           addr,
                                uint8_t *          data,
                                uint16_t           len);

extern uint8_t i2c_check_ack(I2C_HandleTypeDef *hi2c,
                             uint16_t           addr);

extern void i2c_dma_tx_init(I2C_HandleTypeDef *hi2c);

extern HAL_StatusTypeDef i2c_dma_tx_start(I2C_HandleTypeDef *hi2c,
                                          uint16_t           DevAddress);

extern void i2c_dma_tx(I2C_HandleTypeDef *hi2c,
                       uint32_t           addr,
                       uint16_t           ndtr);

extern void i2c_dma_transmit(I2C_HandleTypeDef *hi2c,
                             uint16_t           DevAddress,
                             uint8_t *          pData,
                             uint16_t           Size);

__END_DECLS

/* Private defines -----------------------------------------------------------*/

/* __BSP_I2C_H__ -------------------------------------------------------------*/
#endif /* __BSP_I2C_H__ */

/************************ (C) COPYRIGHT ngu ********************END OF FILE****/
