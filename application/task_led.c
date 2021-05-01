/**
 * *****************************************************************************
 * @file         task_led.c/h
 * @brief        led task
 * @author       ngu
 * @date         20210407
 * @version      1
 * @copyright    Copyright (c) 2021
 * @code         utf-8                                                  @endcode
 * *****************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "task_led.h"

/* Private includes ----------------------------------------------------------*/
#include "bsp.h"
#include "bsp_delay.h"
#include "bsp_led.h"

#if USED_OS
#include "cmsis_os.h"
#endif /* USED_OS */

#include <stdint.h>

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/

void task_led(void *pvParameters)
{
    buzzer_start();
    buzzer_set(0, BUZZER_PWM_DIV2);
    osDelay(1000);
    buzzer_set(0, 0);
    buzzer_stop();

    led_pwm_start();

    uint16_t count;

    for (;;)
    {
        for (count = 0U; count != LED_PWM_MAX; ++count)
        {
            led_pwm_set(LED_R, count);
            led_pwm_set(LED_B, LED_PWM_MAX - count);
            osDelay(1U);
        }
        for (count = 0U; count != LED_PWM_MAX; ++count)
        {
            led_pwm_set(LED_G, count);
            led_pwm_set(LED_R, LED_PWM_MAX - count);
            osDelay(1U);
        }
        for (count = 0U; count != LED_PWM_MAX; ++count)
        {
            led_pwm_set(LED_B, count);
            led_pwm_set(LED_G, LED_PWM_MAX - count);
            osDelay(1U);
        }
    }

    osThreadExit();
}

/************************ (C) COPYRIGHT ngu ********************END OF FILE****/
