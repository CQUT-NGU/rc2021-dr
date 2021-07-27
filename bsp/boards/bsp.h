/**
 * *****************************************************************************
 * @file         bsp.h
 * @brief        Header file of bsp
 * @author       NGU
 * @date         20210427
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
 * *****************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_H__
#define __BSP_H__

#undef __BEGIN_DECLS
#undef __END_DECLS

#if defined(__cplusplus)
#define __BEGIN_DECLS \
    extern "C"        \
    {
#define __END_DECLS \
    }
#else
#define __BEGIN_DECLS
#define __END_DECLS
#endif /* __cplusplus */

#undef __packed
#define __packed __attribute__((__packed__))

#include "main.h"

/* Using the operating system */
#define USED_OS 1

#if USED_OS
#include "cmsis_os.h"
#endif /* USED_OS */

#include "bsp_nvic.h"
#include "bsp_gpio.h"
#include "bsp_led.h"
#include "bsp_delay.h"
#include "bsp_flash.h"
#include "bsp_dma.h"
#include "bsp_usart.h"
#include "bsp_can.h"
#include "bsp_i2c.h"
#include "bsp_spi.h"
#include "bsp_buzzer.h"
#include "bsp_imu_pwm.h"

/* Enddef to prevent recursive inclusion ------------------------------------ */
#endif /* __BSP_H__ */

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
