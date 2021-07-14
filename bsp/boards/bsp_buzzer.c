/**
 * *****************************************************************************
 * @file         bsp_buzzer.c
 * @brief        buzzer of boards
 * @author       NGU
 * @date         20210501
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
 * *****************************************************************************
*/

#include "bsp_buzzer.h"

#undef htim
#define htim htim4
#undef BUZZER_CHANNEL
#define BUZZER_CHANNEL TIM_CHANNEL_3

extern TIM_HandleTypeDef htim;

void buzzer_start(void)
{
    /* Set the TIM Autoreload Register value on runtime */
    __HAL_TIM_SetAutoreload(&htim, BUZZER_PWM_MAX - 1U);
    /* Set the TIM Clock Division value on runtime */
    __HAL_TIM_SetClockDivision(&htim, TIM_CLOCKDIVISION_DIV1);
    /* Starts the PWM signal generation */
    HAL_TIM_PWM_Start(&htim, BUZZER_CHANNEL);
}

void buzzer_stop(void)
{
    /* Stops the PWM signal generation */
    HAL_TIM_PWM_Stop(&htim, BUZZER_CHANNEL);
}

void buzzer_set(uint32_t psc,
                uint32_t pwm)
{
    /* Set the TIM Prescaler on runtime */
    __HAL_TIM_SET_PRESCALER(&htim, psc);
    /* Set the TIM Capture Compare Register value on runtime */
    __HAL_TIM_SetCompare(&htim, BUZZER_CHANNEL, pwm);
}

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
