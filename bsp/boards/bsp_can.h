/**
 * *****************************************************************************
 * @file         bsp_can.c/h
 * @brief        can of boards
 * @author       ngu
 * @date         20210101
 * @version      1
 * @copyright    Copyright (c) 2021
 * @code         utf-8                                                  @endcode
 * @details
 *               CAN1_RX ------> PD0
 *               CAN1_TX ------> PD1
 *               CAN2_RX ------> PB5
 *               CAN2_TX ------> PB6
 * *****************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

/* Includes ------------------------------------------------------------------*/
/* Private includes ----------------------------------------------------------*/
#include <stdint.h>

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

#ifndef CAN2_IS_ENABLE
#define CAN2_IS_ENABLE 0
#endif /* CAN2_IS_ENABLE */

/* Exported types ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/

__BEGIN_DECLS

/**
 * @brief        Intialize the CAN filter
*/
void can_filter_init(void);

__END_DECLS

/* Private defines -----------------------------------------------------------*/

/* __BSP_LED_H__ -------------------------------------------------------------*/
#endif /* __BSP_CAN_H__ */

/************************ (C) COPYRIGHT ngu ********************END OF FILE****/
