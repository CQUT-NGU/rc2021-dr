/**
 * *****************************************************************************
 * @file         bsp_delay.c/h
 * @brief        delay of boards
 * @author       ngu
 * @date         20210101
 * @version      1
 * @copyright    Copyright (c) 2021
 * @code         utf-8                                                  @endcode
 * *****************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_DELAY_H__
#define __BSP_DELAY_H__

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Private includes ----------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
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

/**
 * @brief        Initializes the delay function
*/
extern void delay_init(void);

/**
 * @brief        Delay microseconds
 * @param[in]    nus: 0x0000 ~ 0xFFFF
*/
extern void delay_us(uint16_t nus);

/**
 * @brief        Delay millisecond
 * @param[in]    nms: 0x0000 ~ 0xFFFF
*/
extern void delay_ms(uint16_t nms);

__END_DECLS

/* Private defines -----------------------------------------------------------*/

/* __BSP_DELAY_H__ -----------------------------------------------------------*/
#endif /* __BSP_DELAY_H__ */

/************************ (C) COPYRIGHT ngu ********************END OF FILE****/
