/**
 * *****************************************************************************
 * @file         bsp_led.c/h
 * @brief        led of boards
 * @author       ngu
 * @date         20210101
 * @version      1
 * @copyright    Copyright (c) 2021
 * @code         utf-8                                                  @endcode
 * @details      GPIO PIN
 *               LED_R ------> PH12
 *               LED_G ------> PH11
 *               LED_B ------> PH10
 * *****************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_LED_H__
#define __BSP_LED_H__

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/
#define LED_PORT GPIOH

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

typedef enum
{
    LED_ON  = 0U,      /*!< turn on */
    LED_OFF = !LED_ON, /*!< turn off */
} led_state_e;

typedef enum
{
    LED_R = GPIO_PIN_12, /*!< red */
    LED_G = GPIO_PIN_11, /*!< green */
    LED_B = GPIO_PIN_10, /*!< blue */
} led_e;

/* Exported functions prototypes ---------------------------------------------*/

__BEGIN_DECLS

/**
 * @brief        set the led on or off
 * @param[in]    pin:   led_e { LED_R LED_G LED_B }
 * @param[in]    state: led_state_e { LED_ON LED_OFF }
 */
extern void led_write(led_e       pin,
                      led_state_e state);

/**
 * @brief        read the led state
 * @param[in]    pin: led_e { LED_R LED_G LED_B }
 * @return       led_state_e { LED_ON LED_OFF }
 */
extern led_state_e led_read(led_e pin);

/**
 * @brief        toggle the led
 * @param[in]    pin: led_e { LED_R LED_G LED_B }
 */
extern void led_toggle(led_e pin);

/**
 * @brief        start pwm of the red led
 */
extern void led_pwm_start(void);

/**
 * @brief        stop pwm of the red led
 */
extern void led_pwm_stop(void);

/**
 * @brief        set pwm of the red led
 * @param[in]    pin:   led_e { LED_R LED_G LED_B }
 * @param[in]    value: 0 ~ 1000U
 */
extern void led_pwm_set(led_e    pin,
                        uint16_t value);

__END_DECLS

/* Private defines -----------------------------------------------------------*/

/* __BSP_LED_H__ -------------------------------------------------------------*/
#endif /* __BSP_LED_H__ */

/************************ (C) COPYRIGHT ngu ********************END OF FILE****/
