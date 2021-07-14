/**
 * *****************************************************************************
 * @file         ctrl_step.c
 * @brief        stepping motor control
 * @author       NGU
 * @date         20210502
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
 * *****************************************************************************
*/

#include "ctrl_step.h"

#include "ca.h"

#define PICK_TIM htim8

extern TIM_HandleTypeDef PICK_TIM;

#define PICK_CHANNEL TIM_CHANNEL_2

#define PICK_PWM_DIVIDE 6400
#define PICK_PWM_DELTA  500

ctrl_step_t step;

void pick_set(int32_t hz)
{
    uint32_t set = 1;

    if (hz > 0)
    {
        gpio_pin_set(PICK_DIR_GPIO_Port, PICK_DIR_Pin);
        set = (uint32_t)hz;
    }
    else if (hz < 0)
    {
        gpio_pin_reset(PICK_DIR_GPIO_Port, PICK_DIR_Pin);
        set = (uint32_t)-hz;
    }
    else
    {
        if (READ_BIT(step.flag, PICK_FLAG_RUN))
        {
            CLEAR_BIT(step.flag, PICK_FLAG_RUN);
            HAL_TIM_PWM_Stop(&PICK_TIM, PICK_CHANNEL);
        }
        return;
    }

    if (!READ_BIT(step.flag, PICK_FLAG_RUN))
    {
        SET_BIT(step.flag, PICK_FLAG_RUN);
        HAL_TIM_PWM_Start(&PICK_TIM, PICK_CHANNEL);
    }

    uint32_t x = (uint32_t)ca_sqrt_u32(SystemCoreClock / set);
    __HAL_TIM_SET_PRESCALER(&PICK_TIM, x - 1);
    __HAL_TIM_SET_AUTORELOAD(&PICK_TIM, x - 1);
    __HAL_TIM_SetCompare(&PICK_TIM, PICK_CHANNEL, (x >> 1));
}

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
