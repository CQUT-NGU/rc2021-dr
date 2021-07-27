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

#define PICK_TIM         htim8
#define COUNT_TIM        htim2
#define COUNT_IRQHandler TIM2_IRQHandler

extern TIM_HandleTypeDef PICK_TIM;
extern TIM_HandleTypeDef COUNT_TIM;

#define PICK_IT_CC   TIM_IT_CC2
#define PICK_CHANNEL TIM_CHANNEL_2

ctrl_step_t step;

void pick_init(void)
{
    pick_zero_cli(PICK_INDEX_CLI);
}

uint32_t pick_set_dir(int32_t offset)
{
    if (offset < 0)
    {
        SET_BIT(step.flag, PICK_FLAG_REVERSE);
        step.cnt = (uint32_t)-offset;
        gpio_pin_set(PICK_DIR_GPIO_Port, PICK_DIR_Pin);
    }
    else
    {
        CLEAR_BIT(step.flag, PICK_FLAG_REVERSE);
        step.cnt = (uint32_t)offset;
        gpio_pin_reset(PICK_DIR_GPIO_Port, PICK_DIR_Pin);
    }

    return step.cnt;
}

void pick_set_freq(uint32_t hz)
{
    step.fr = hz;

    uint32_t x = (uint32_t)ca_sqrt_u32(SystemCoreClock / hz);
    uint32_t xh = x >> 1;

    __HAL_TIM_SET_PRESCALER(&PICK_TIM, x - 1);
    __HAL_TIM_SET_AUTORELOAD(&PICK_TIM, x - 1);
    __HAL_TIM_SET_COMPARE(&PICK_TIM, PICK_CHANNEL, xh);
}

void pick_set(int32_t hz)
{
    if (hz == 0)
    {
        if (READ_BIT(step.flag, PICK_FLAG_RUN))
        {
            CLEAR_BIT(step.flag, PICK_FLAG_RUN);

            pick_stop();
        }
        return;
    }
    else if (READ_BIT(step.flag, PICK_FLAG_AUTO | PICK_FLAG_ZERO))
    {
        CLEAR_BIT(step.flag, PICK_FLAG_AUTO | PICK_FLAG_ZERO);

        pick_stop();
    }

    uint32_t set = pick_set_dir(hz);
    if (step.fr != set)
    {
        pick_set_freq(set);
    }

    if (!READ_BIT(step.flag, PICK_FLAG_RUN))
    {
        SET_BIT(step.flag, PICK_FLAG_RUN);

        __HAL_TIM_CLEAR_FLAG(&COUNT_TIM, PICK_IT_CC);
        __HAL_TIM_DISABLE_IT(&COUNT_TIM, PICK_IT_CC);
        __HAL_TIM_ENABLE(&COUNT_TIM);

        HAL_TIM_PWM_Start(&PICK_TIM, PICK_CHANNEL);
    }
}

void pick_zero_cli(int32_t idx)
{
    if (READ_BIT(step.flag, PICK_FLAG_RUN))
    {
        CLEAR_BIT(step.flag, PICK_FLAG_RUN);

        pick_stop();
    }

    if (!READ_BIT(step.flag, PICK_FLAG_ZERO))
    {
        SET_BIT(step.flag, PICK_FLAG_ZERO);

        pick_start(idx);
    }
}

void pick_update(uint32_t inc, uint32_t cnt)
{
    if (READ_BIT(step.flag, PICK_FLAG_AUTO))
    {
        step.cnt = __HAL_TIM_GET_COUNTER(&COUNT_TIM);
        if (step.cnt < cnt)
        {
            step.fr += inc;
            pick_set_freq(step.fr);
        }
    }
}

void pick_index(int32_t idx)
{
    if (READ_BIT(step.flag, PICK_FLAG_RUN | PICK_FLAG_AUTO))
    {
        CLEAR_BIT(step.flag, PICK_FLAG_RUN);

        pick_stop();
    }

    if (step.idx != idx)
    {
        step.set = idx;
        pick_start(step.set - step.idx);
    }
}

void pick_start(int32_t offset)
{
    step.cnt = pick_set_dir(offset);
    pick_set_freq(PICK_PWM_DELTA);

    SET_BIT(step.flag, PICK_FLAG_AUTO);

    __HAL_TIM_SET_COUNTER(&COUNT_TIM, 0);
    __HAL_TIM_SET_COMPARE(&COUNT_TIM, PICK_CHANNEL, step.cnt);
    __HAL_TIM_CLEAR_FLAG(&COUNT_TIM, PICK_IT_CC);
    __HAL_TIM_ENABLE_IT(&COUNT_TIM, PICK_IT_CC);
    __HAL_TIM_ENABLE(&COUNT_TIM);

    HAL_TIM_PWM_Start(&PICK_TIM, PICK_CHANNEL);
}

void pick_stop(void)
{
    HAL_TIM_PWM_Stop(&PICK_TIM, PICK_CHANNEL);

    CLEAR_BIT(step.flag, PICK_FLAG_AUTO);

    step.cnt = __HAL_TIM_GET_COUNTER(&COUNT_TIM);
    __HAL_TIM_SET_COUNTER(&COUNT_TIM, 0);
    if (READ_BIT(step.flag, PICK_FLAG_REVERSE))
    {
        step.idx -= (int32_t)step.cnt;
    }
    else
    {
        step.idx += (int32_t)step.cnt;
    }
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void COUNT_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&COUNT_TIM, PICK_IT_CC))
    {
        __HAL_TIM_CLEAR_FLAG(&COUNT_TIM, PICK_IT_CC);
        __HAL_TIM_DISABLE_IT(&COUNT_TIM, PICK_IT_CC);
    }

    if (READ_BIT(step.flag, PICK_FLAG_AUTO))
    {
        pick_stop();
    }

    if (READ_BIT(step.flag, PICK_FLAG_ZERO))
    {
        CLEAR_BIT(step.flag, PICK_FLAG_ZERO);

        step.idx = 0;
        step.set = 0;
    }
}

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
