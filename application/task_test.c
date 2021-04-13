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
    HAL_TIM_Base_Start(&htim8);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
}

void servo_stop(void)
{
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
    HAL_TIM_Base_Stop(&htim8);
}

void task_test(void *pvParameters)
{
    ctrl_rc_t *rc = ctrl_rc_point();
    ctrl_pc_t *pc = ctrl_pc_point();

    servo_start();

    for (;;)
    {
        if (rc->rc.s[1U] == RC_SW_UP)
        {
            /* 1000 ~ 2000 */
            uint16_t servo_up   = 1000 + 100 * rc->rc.ch[3] / 66;
            uint16_t servo_down = 1400 - 100 * rc->rc.ch[1] / 66;
            /* 1400 ~ 400 */

            if (rc->rc.ch[1U] < -10 || rc->rc.ch[3U] < -10)
            {
                /* Clip ball steering gear reset */
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 1000U);
                /* Pitch steering gear reset */
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 1400U);
            }
            else
            {
                /* Clip ball steering gear */
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, servo_up > 1000U ? servo_up : 1000U);
                /* Pitch steering gear */
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, servo_down > 400U ? servo_down : 400U);
            }
        }

        if (rc->rc.ch[4U] < -10) /* clockwise rotation */
        {
            GPIOE->BSRR = (uint32_t)GPIO_PIN_9; /* kick the ball */
        }
        else if (rc->rc.ch[4U] > 10) /* contrarotate */
        {
            GPIOE->BSRR = (uint32_t)GPIO_PIN_11; /* throw the ball */
        }
        else /* relay reset */
        {
            GPIOE->BSRR = (uint32_t)GPIO_PIN_9 << 16;
            GPIOE->BSRR = (uint32_t)GPIO_PIN_11 << 16;
        }

        if (pc->c == 'a') /* ctrl by pc */
        {
            /* servo up */
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pc->x);
            /* servo down */
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, pc->y);
        }

        osDelay(2U);
    }

    osThreadExit();
}

/************************ (C) COPYRIGHT tqfx *******************END OF FILE****/
