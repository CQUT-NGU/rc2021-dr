/**
 * *****************************************************************************
 * @file         bsp.h
 * @brief        Header file of bsp
 * @author       ngu
 * @date         20210101
 * @version      1
 * @copyright    Copyright (c) 2021
 * @code         utf-8                                                  @endcode
 * *****************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_H__
#define __BSP_H__

/* Includes ------------------------------------------------------------------*/
#include "bsp_buzzer.h"
#include "bsp_can.h"
#include "bsp_delay.h"
#include "bsp_flash.h"
#include "bsp_led.h"
#include "bsp_spi.h"
#include "bsp_usart.h"

/* Using the operating system */
#define USED_OS 1

#if USED_OS
#include "cmsis_os.h"
#endif /* USED_OS */

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/
/* Private defines -----------------------------------------------------------*/

/* __BSP_H__ -----------------------------------------------------------------*/
#endif /* __BSP_H__ */

/************************ (C) COPYRIGHT ngu ********************END OF FILE****/
