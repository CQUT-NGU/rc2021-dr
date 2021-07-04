/**
 * *****************************************************************************
 * @file         bsp_led.h
 * @brief        led of boards
 * @author       NGU
 * @date         20210501
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
 * @details      GPIO PIN
 *               LED_R ------> PH12
 *               LED_G ------> PH11
 *               LED_B ------> PH10
 * *****************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_LED_H__
#define __BSP_LED_H__

#include "bsp.h"

#define LED_PORT      GPIOH
#define LED_PRESCALER 84

#define LED_PWM_MAX  1000
#define LED_PWM_DIV1 (LED_PWM_MAX >> 0)
#define LED_PWM_DIV2 (LED_PWM_MAX >> 1)
#define LED_PWM_DIV4 (LED_PWM_MAX >> 2)
#define LED_PWM_DIV8 (LED_PWM_MAX >> 3)

typedef enum
{
    LED_ON = 0,        /* turn on */
    LED_OFF = !LED_ON, /* turn off */
} led_state_e;

typedef enum
{
    LED_R = GPIO_PIN_12, /* red */
    LED_G = GPIO_PIN_11, /* green */
    LED_B = GPIO_PIN_10, /* blue */
} led_e;

__BEGIN_DECLS

/**
 * @brief        set the led on or off
 * @param[in]    pin:   led_e { LED_R LED_G LED_B }
 * @param[in]    state: led_state_e { LED_ON LED_OFF }
 */
extern void led_write(led_e pin,
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
 * @param[in]    value: 0 ~ LED_PWM_MAX
 */
extern void led_pwm_set(led_e pin,
                        uint16_t value);

__END_DECLS

/* Enddef to prevent recursive inclusion ------------------------------------ */
#endif /* __BSP_LED_H__ */

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
