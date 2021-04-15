/**
 * *****************************************************************************
 * @file         bsp_led.c/h
 * @brief        led of boards
 * @author       ngu
 * @date         20210427
 * @version      1
 * @copyright    Copyright (c) 2021
 * @code         utf-8                                                  @endcode
 * @details      GPIO PIN
 *               LED_R ------> PH12
 *               LED_G ------> PH11
 *               LED_B ------> PH10
 * *****************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "bsp_led.h"

/* Private includes ----------------------------------------------------------*/
#include "main.h"

/* Private define ------------------------------------------------------------*/

#undef htim
#define htim htim5

#undef LED_CHANNEL_R
#define LED_CHANNEL_R TIM_CHANNEL_3
#undef LED_CHANNEL_G
#define LED_CHANNEL_G TIM_CHANNEL_2
#undef LED_CHANNEL_B
#define LED_CHANNEL_B TIM_CHANNEL_1

/* Private includes ----------------------------------------------------------*/
extern TIM_HandleTypeDef htim;

/* Private macro -------------------------------------------------------------*/

#undef LED_W
#define LED_W(GPIOx, PIN, STATE)                \
    do                                          \
    {                                           \
        if (STATE == LED_OFF)                   \
        {                                       \
            GPIOx->BSRR = (uint32_t)PIN;        \
        }                                       \
        else                                    \
        {                                       \
            GPIOx->BSRR = (uint32_t)PIN << 16U; \
        }                                       \
    } while (0)

#undef LED_R
#define LED_R(GPIOx, PIN, STATE) \
    do                           \
    {                            \
        if (GPIOx->IDR & PIN)    \
        {                        \
            STATE = LED_OFF;     \
        }                        \
        else                     \
        {                        \
            STATE = LED_ON;      \
        }                        \
    } while (0)

#undef LED_T
#define LED_T(GPIOx, PIN)                       \
    do                                          \
    {                                           \
        if ((GPIOx->ODR & PIN) == PIN)          \
        {                                       \
            GPIOx->BSRR = (uint32_t)PIN << 16U; \
        }                                       \
        else                                    \
        {                                       \
            GPIOx->BSRR = (uint32_t)PIN;        \
        }                                       \
    } while (0)

/* Private typedef -----------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/

void led_write(led_e       pin,
               led_state_e state)
{
    switch (pin)
    {
    case LED_R:
    case LED_G:
    case LED_B:
    {
        LED_W(LED_PORT, pin, state);
        break;
    }
    default:
        break;
    }
}

led_state_e led_read(led_e pin)
{
    led_state_e state = LED_OFF;

    switch (pin)
    {
    case LED_R:
    case LED_G:
    case LED_B:
    {
        LED_R(LED_PORT, pin, state);
        break;
    }
    default:
        break;
    }

    return state;
}

void led_toggle(led_e pin)
{
    switch (pin)
    {
    case LED_R:
    case LED_G:
    case LED_B:
    {
        LED_T(LED_PORT, pin);
        break;
    }
    default:
        break;
    }
}

void led_pwm_start(void)
{
    /*!< Set the TIM Prescaler on runtime */
    __HAL_TIM_SET_PRESCALER(&htim, 83U);
    /*!< Set the TIM Autoreload Register value on runtime */
    __HAL_TIM_SetAutoreload(&htim, 999U);
    /*!< Set the TIM Clock Division value on runtime */
    __HAL_TIM_SetClockDivision(&htim, TIM_CLOCKDIVISION_DIV1);
    /*!< Starts the PWM signal generation */
    HAL_TIM_PWM_Start(&htim, LED_CHANNEL_R);
    HAL_TIM_PWM_Start(&htim, LED_CHANNEL_G);
    HAL_TIM_PWM_Start(&htim, LED_CHANNEL_B);
}

void led_pwm_stop(void)
{
    /*!< Stops the PWM signal generation */
    HAL_TIM_PWM_Stop(&htim, LED_CHANNEL_R);
    HAL_TIM_PWM_Stop(&htim, LED_CHANNEL_G);
    HAL_TIM_PWM_Stop(&htim, LED_CHANNEL_B);
}

void led_pwm_set(led_e    pin,
                 uint16_t value)
{
    /*!< Set the TIM Capture Compare Register value on runtime */
    switch (pin)
    {
    case LED_R:
    {
        __HAL_TIM_SetCompare(&htim, LED_CHANNEL_R, value);
        break;
    }
    case LED_G:
    {
        __HAL_TIM_SetCompare(&htim, LED_CHANNEL_G, value);
        break;
    }
    case LED_B:
    {
        __HAL_TIM_SetCompare(&htim, LED_CHANNEL_B, value);
        break;
    }
    default:
        break;
    }
}

/************************ (C) COPYRIGHT ngu ********************END OF FILE****/
