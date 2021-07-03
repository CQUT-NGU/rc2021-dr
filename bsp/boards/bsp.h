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

#include "bsp_buzzer.h"
#include "bsp_can.h"
#include "bsp_delay.h"
#include "bsp_flash.h"
#include "bsp_gpio.h"
#include "bsp_i2c.h"
#include "bsp_imu_pwm.h"
#include "bsp_led.h"
#include "bsp_spi.h"
#include "bsp_uart.h"
#include "bsp_usart.h"

/* Using the operating system */
#define USED_OS 1

#if USED_OS
#include "cmsis_os.h"
#endif /* USED_OS */

/* Enddef to prevent recursive inclusion ------------------------------------ */
#endif /* __BSP_H__ */

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
