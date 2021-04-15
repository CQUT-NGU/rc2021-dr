/*!< @encoding utf-8 */
/**
 * *****************************************************************************
 * @file         task_led.c/h
 * @brief        led task
 * @author       tqfx
 * @date         20210101
 * @version      0.01
 * @copyright    Copyright (c) 2020-2021
 * *****************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "task_test.h"

/* Private includes ----------------------------------------------------------*/

#include "bsp.h"
#include "bsp_delay.h"
#include "cmsis_os.h"
#include "ctrl.h"
#include "main.h"

#include <stdint.h>

extern TIM_HandleTypeDef htim8;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/

void servo_start(void)
{
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
}

void servo_stop(void)
{
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
}

void task_test(void *pvParameters)
{
    const ctrl_rc_t *rc = ctrl_rc_point();
    ctrl_pc_t *      pc = ctrl_pc_point();

    //servo_start();

    __HAL_TIM_SET_PRESCALER(&htim8, 168);
    __HAL_TIM_SetAutoreload(&htim8, 999);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 500);

    uint8_t flag_stop = 1;

    for (;;)
    {
        switch (pc->c)
        {
        case 't':
        {
            int32_t tmp = (int32_t)pc->x;
            __HAL_TIM_SET_PRESCALER(&htim8, tmp);
            tmp = (int32_t)pc->y;
            __HAL_TIM_SetAutoreload(&htim8, tmp);
            __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, (tmp / 2));
            if (flag_stop)
            {
                flag_stop = 0;
                HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
            }
            break;
        }

        default:
        {
            if (rc->rc.s[1] == RC_SW_DOWN || rc->rc.s[0] == RC_SW_DOWN)
            {
                if (rc->rc.ch[3] > 10)
                {
                    GPIOC->BSRR = GPIO_PIN_6;
                    __HAL_TIM_PRESCALER(&htim8, 660 - rc->rc.ch[3]);
                    if (flag_stop)
                    {
                        flag_stop = 0;
                        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
                    }
                }
                else if (rc->rc.ch[3] < -10)
                {
                    GPIOC->BSRR = (uint32_t)GPIO_PIN_6 << 16;
                    __HAL_TIM_PRESCALER(&htim8, 660 + rc->rc.ch[3]);
                    if (flag_stop)
                    {
                        flag_stop = 0;
                        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
                    }
                }
                else
                {
                    if (!flag_stop)
                    {
                        flag_stop = 1;
                        HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
                    }
                }
            }
            break;
        }
        }

        osDelay(2U);
    }

    osThreadExit();
}

/************************ (C) COPYRIGHT tqfx *******************END OF FILE****/
