/**
 * *****************************************************************************
 * @file         task_led.c/h
 * @brief        led task
 * @author       ngu
 * @date         20210427
 * @version      1
 * @copyright    Copyright (c) 2021
 * @code         utf-8                                                  @endcode
 * *****************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TASK_LED_H__
#define __TASK_LED_H__

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

/* Exported types ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/

__BEGIN_DECLS

extern void task_led(void *pvParameters);

__END_DECLS

/* Private defines -----------------------------------------------------------*/

/* __TASK_LED_H__ ------------------------------------------------------------*/
#endif /* __TASK_LED_H__ */

/************************ (C) COPYRIGHT ngu ********************END OF FILE****/
