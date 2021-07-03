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
#include "bsp_delay.h"
#include "bsp_led.h"

#if USED_OS
#include "cmsis_os.h"
#endif /* USED_OS */

#include <stdint.h>

#include "task_chassis.h"
extern chassis_move_t move;

void task_led(void *pvParameters)
{
    (void)pvParameters;

    buzzer_start();
    buzzer_set(0, BUZZER_PWM_DIV64);
    osDelay(1000);
    buzzer_set(0, 0);
    buzzer_stop();

    led_pwm_start();

    uint8_t t = 0;
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

#if 0
        if (t == 1)
        {
            move.data_pc->c = 'p';
            move.data_pc->x = 0;
            move.data_pc->y = 1;
        }
        else if (t == 2)
        {
            move.data_pc->c = 'p';
            move.data_pc->x = 1;
            move.data_pc->y = 1;
        }
        else if (t == 3)
        {
            move.data_pc->c = 'p';
            move.data_pc->x = 1;
            move.data_pc->y = 0;
        }
        else if (t == 4)
        {
            move.data_pc->c = 'p';
            move.data_pc->x = 0;
            move.data_pc->y = 0;
            t               = 0;
        }
#endif
        ++t;
    }
}

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
