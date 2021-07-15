/**
 * *****************************************************************************
 * @file         app_led.c
 * @brief        led application
 * @author       NGU
 * @date         20210427
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
 * *****************************************************************************
*/

#define NOTEFREQS_PROPORT ((168000000 >> 1) / BUZZER_PWM_MAX)
#define NOTEFREQS_TYPE    unsigned int

#include "notefreqs.h"

#include "app_led.h"

static unsigned int song[][2] = {
    {NOTEFREQS_G4 - 1, 50},
    {NOTEFREQS_A4 - 1, 50},
    {NOTEFREQS_C5 - 1, 50},
    {0, 10},
    {NOTEFREQS_D5 - 1, 50},
    {0, 10},
    {NOTEFREQS_D5 - 1, 50},
    {0, 1},
    {NOTEFREQS_D5 - 1, 50},
    {0, 1},
    {NOTEFREQS_C5 - 1, 50},
    {0, 1},
    {NOTEFREQS_D5 - 1, 50},
    {0, 15},
    {NOTEFREQS_C5 - 1, 50},
    {0, 15},
    {NOTEFREQS_E5 - 1, 50},
    {0, 15},
    {0, 50},
    {NOTEFREQS_B4 - 1, 40},
    {0, 10},
    {NOTEFREQS_B4 - 1, 40},
    {0, 10},
    {NOTEFREQS_B4 - 1, 40},
    {0, 10},
    {NOTEFREQS_B4 - 1, 75},
    {0, 15},
    {NOTEFREQS_B4 - 1, 50},
    {0, 25},
    {NOTEFREQS_A4 - 1, 25},
    {0, 25},
    {NOTEFREQS_C5 - 1, 25},
    {0, 25},
};

void task_led(void *pvParameters)
{
    (void)pvParameters;

    buzzer_start();
    {
        unsigned int n = sizeof(song) / sizeof(*song);
        unsigned int i = 0;
        while (i != n)
        {
            buzzer_set(song[i][0], BUZZER_PWM_DIV2);
            vTaskDelay((song[i][1] << 2) + song[i][1]);
            ++i;
        }
    }
    buzzer_stop();

    led_pwm_start();

    for (;;)
    {
        for (uint32_t count = 0; count != LED_PWM_MAX; ++count)
        {
            led_pwm_set(LED_R, count);
            led_pwm_set(LED_B, LED_PWM_MAX - count);
            osDelay(1);
        }
        for (uint32_t count = 0; count != LED_PWM_MAX; ++count)
        {
            led_pwm_set(LED_G, count);
            led_pwm_set(LED_R, LED_PWM_MAX - count);
            osDelay(1);
        }
        for (uint32_t count = 0; count != LED_PWM_MAX; ++count)
        {
            led_pwm_set(LED_B, count);
            led_pwm_set(LED_G, LED_PWM_MAX - count);
            osDelay(1);
        }
    }
}

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
