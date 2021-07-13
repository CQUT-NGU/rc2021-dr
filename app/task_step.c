/**
 * *****************************************************************************
 * @file         task_step.c
 * @brief        stepping motor control task
 * @author       NGU
 * @date         20210101
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
 * *****************************************************************************
*/

#include "task_step.h"

#include "ca.h"
#include "bsp.h"
#include "ctrl.h"

#if USED_OS
#include "cmsis_os.h"
#endif /* USED_OS */

#define PICK_TIM htim1   // 168M
#define HOLD_TIM htim8   // 168M
#define POT_TIM  htim12  // 84M

#define PICK_CHANNEL TIM_CHANNEL_1
#define HOLD_CHANNEL TIM_CHANNEL_1
#define POT_CHANNEL  TIM_CHANNEL_1

#define FLAG_RUN_PICK (1 << 0)
#define FLAG_RUN_HOLD (1 << 1)
#define FLAG_RUN_POT  (1 << 2)

extern TIM_HandleTypeDef PICK_TIM;
extern TIM_HandleTypeDef HOLD_TIM;
extern TIM_HandleTypeDef POT_TIM;

static int8_t step_flag_run = 0;

static void pick_set(uint32_t hz)
{
    uint32_t x = (uint32_t)ca_sqrt_u32(SystemCoreClock / hz);

    __HAL_TIM_SET_PRESCALER(&PICK_TIM, x - 1);
    __HAL_TIM_SET_AUTORELOAD(&PICK_TIM, x - 1);
    __HAL_TIM_SET_COMPARE(&PICK_TIM, PICK_CHANNEL, x >> 1);
}

static void hold_set(uint32_t hz)
{
    uint32_t x = (uint32_t)ca_sqrt_u32(SystemCoreClock / hz);

    __HAL_TIM_SET_PRESCALER(&HOLD_TIM, x - 1);
    __HAL_TIM_SetAutoreload(&HOLD_TIM, x - 1);
    __HAL_TIM_SetCompare(&HOLD_TIM, HOLD_CHANNEL, (x >> 1));
}

static void pot_set(uint32_t hz)
{
    uint32_t x = (uint32_t)ca_sqrt_u32(SystemCoreClock / hz);

    __HAL_TIM_SET_PRESCALER(&POT_TIM, x - 1);
    __HAL_TIM_SetAutoreload(&POT_TIM, x - 1);
    __HAL_TIM_SetCompare(&POT_TIM, HOLD_CHANNEL, (x >> 1));
}

void task_step(void *pvParameters)
{
    (void)pvParameters;

    const ctrl_rc_t *rc = ctrl_rc_point();

    osDelay(1000);

    {
        HAL_TIM_Base_Start(&PICK_TIM);
        HAL_TIM_Base_Start(&HOLD_TIM);
        HAL_TIM_Base_Start(&POT_TIM);
    }

    while (1)
    {
        int16_t fr = 0;
        if (switch_is_down(rc->rc.s[RC_SW_L]) &&
            switch_is_down(rc->rc.s[RC_SW_R]))
        {
            /* pick */
            fr = rc->rc.ch[RC_CH_LH] << 1;
            if (fr > 10)
            {
                gpio_pin_set(PICK_DIR_GPIO_Port, PICK_DIR_Pin);
            }
            else if (fr < -10)
            {
                gpio_pin_reset(PICK_DIR_GPIO_Port, PICK_DIR_Pin);
                fr = (int16_t)(-fr);
            }
            if (fr > 10)
            {
                pick_set((uint32_t)fr);
                if (!(step_flag_run & FLAG_RUN_PICK))
                {
                    SET_BIT(step_flag_run, FLAG_RUN_PICK);
                    HAL_TIM_PWM_Start(&PICK_TIM, PICK_CHANNEL);
                }
            }
            else
            {
                if (step_flag_run & FLAG_RUN_PICK)
                {
                    CLEAR_BIT(step_flag_run, FLAG_RUN_PICK);
                    HAL_TIM_PWM_Stop(&PICK_TIM, PICK_CHANNEL);
                }
            }

            /* hold */
            fr = rc->rc.ch[RC_CH_LV] << 1;
            if (fr > 10)
            {
                gpio_pin_set(HOLD_DIR_GPIO_Port, HOLD_DIR_Pin);
            }
            else if (fr < -10)
            {
                gpio_pin_reset(HOLD_DIR_GPIO_Port, HOLD_DIR_Pin);
                fr = (int16_t)(-fr);
            }
            if (fr > 10)
            {
                gpio_pin_set(HOLD_DIR_GPIO_Port, HOLD_DIR_Pin);
            }
            else if (fr < -10)
            {
                gpio_pin_reset(HOLD_DIR_GPIO_Port, HOLD_DIR_Pin);
                fr = (int16_t)(-fr);
            }
            if (fr > 10)
            {
                hold_set((uint32_t)fr);

                if (!(step_flag_run & FLAG_RUN_HOLD))
                {
                    SET_BIT(step_flag_run, FLAG_RUN_HOLD);
                    HAL_TIM_PWM_Start(&HOLD_TIM, HOLD_CHANNEL);
                }
            }
            else
            {
                if (step_flag_run & FLAG_RUN_HOLD)
                {
                    CLEAR_BIT(step_flag_run, FLAG_RUN_HOLD);
                    HAL_TIM_PWM_Stop(&HOLD_TIM, HOLD_CHANNEL);
                }
            }

            /* pot */
            fr = rc->rc.ch[RC_CH_RH] << 1;
            if (fr > 10)
            {
                gpio_pin_set(POT_DIR_GPIO_Port, POT_DIR_Pin);
            }
            else if (fr < -10)
            {
                gpio_pin_reset(POT_DIR_GPIO_Port, POT_DIR_Pin);
                fr = (int16_t)(-fr);
            }
            if (fr > 10)
            {
                pot_set((uint32_t)fr);
                if (!(step_flag_run & FLAG_RUN_POT))
                {
                    SET_BIT(step_flag_run, FLAG_RUN_POT);
                    HAL_TIM_PWM_Start(&POT_TIM, POT_CHANNEL);
                }
            }
            else
            {
                if (step_flag_run & FLAG_RUN_POT)
                {
                    CLEAR_BIT(step_flag_run, FLAG_RUN_POT);
                    HAL_TIM_PWM_Stop(&POT_TIM, POT_CHANNEL);
                }
            }

            /* relay control */
            if (rc->rc.ch[RC_CH_S] > 1000)
            {
                gpio_pin_set(RELAY_GPIO_Port, RELAY_Pin);
            }
            else
            {
                gpio_pin_reset(RELAY_GPIO_Port, RELAY_Pin);
            }
        }

        osDelay(10);
    }
}

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
