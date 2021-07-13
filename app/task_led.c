/**
 * *****************************************************************************
 * @file         task_led.c
 * @brief        led task
 * @author       NGU
 * @date         20210407
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
 * *****************************************************************************
*/

#include "task_led.h"

#include "bsp.h"

#if USED_OS
#include "cmsis_os.h"
#endif /* USED_OS */

void task_led(void *pvParameters)
{
    (void)pvParameters;

    buzzer_start();
    {
        uint16_t i = 0;
        while (i != 4)
        {
            buzzer_set(i++, BUZZER_PWM_DIV2);
            osDelay(0x80);
        }
        while (i)
        {
            buzzer_set(--i, BUZZER_PWM_DIV2);
            osDelay(0x80);
        }
    }
    buzzer_stop();

    led_pwm_start();

    for (;;)
    {
        for (uint16_t count = 0; count != LED_PWM_MAX; ++count)
        {
            led_pwm_set(LED_R, count);
            led_pwm_set(LED_B, (uint16_t)(LED_PWM_MAX - count));
            osDelay(1);
        }
        for (uint16_t count = 0; count != LED_PWM_MAX; ++count)
        {
            led_pwm_set(LED_G, count);
            led_pwm_set(LED_R, (uint16_t)(LED_PWM_MAX - count));
            osDelay(1);
        }
        for (uint16_t count = 0; count != LED_PWM_MAX; ++count)
        {
            led_pwm_set(LED_B, count);
            led_pwm_set(LED_G, (uint16_t)(LED_PWM_MAX - count));
            osDelay(1);
        }
    }
}

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
