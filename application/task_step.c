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

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

#define PICK_TIM htim1
#define NIP_TIM  htim8

#define PICK_L_CHANNEL TIM_CHANNEL_1
#define PICK_R_CHANNEL TIM_CHANNEL_3
#define NIP_CHANNEL    TIM_CHANNEL_1

#define LIMIT_LOW_PICK 220
#define LIMIT_LOW_NIP  110

#define FLAG_RUN_PICK_L (1 << 0)
#define FLAG_RUN_PICK_R (1 << 1)
#define FLAG_RUN_NIP    (1 << 2)

static int8_t step_flag_run = 0;

static void pick_set(uint32_t ch, uint32_t hz)
{
    uint32_t x = (uint32_t)ca_sqrt_u32(SystemCoreClock / hz);

    __HAL_TIM_SET_PRESCALER(&PICK_TIM, x - 1);
    __HAL_TIM_SetAutoreload(&PICK_TIM, x - 1);

    switch (ch)
    {
    case PICK_L_CHANNEL:
    {
        __HAL_TIM_SetCompare(&PICK_TIM, PICK_L_CHANNEL, (x >> 1));
        break;
    }

    case PICK_R_CHANNEL:
    {
        __HAL_TIM_SetCompare(&PICK_TIM, PICK_R_CHANNEL, (x >> 1));
        break;
    }

    default:
        break;
    }
}

static void nip_set(uint32_t hz)
{
    uint32_t x = (uint32_t)ca_sqrt_u32(SystemCoreClock / hz);

    __HAL_TIM_SET_PRESCALER(&NIP_TIM, x - 1);
    __HAL_TIM_SetAutoreload(&NIP_TIM, x - 1);
    __HAL_TIM_SetCompare(&NIP_TIM, NIP_CHANNEL, (x >> 1));
}

static void step_init(void)
{
    HAL_TIM_Base_Start(&PICK_TIM);
    HAL_TIM_Base_Start(&NIP_TIM);
}

static void pick_update(const ctrl_rc_t *rc)
{
    if (switch_is_mid(rc->rc.s[RC_SW_L]) &&
        switch_is_down(rc->rc.s[RC_SW_R]))
    {
        int16_t fr = rc->rc.ch[RC_CH_LV];

        if (fr > LIMIT_LOW_PICK)
        {
            gpio_pin_set(PICK_R_DIR_GPIO_Port, PICK_R_DIR_Pin);
        }
        else if (fr < -LIMIT_LOW_PICK)
        {
            gpio_pin_reset(PICK_R_DIR_GPIO_Port, PICK_R_DIR_Pin);
            fr = (int16_t)(-fr);
        }

        if (fr > LIMIT_LOW_PICK)
        {
            pick_set(PICK_R_CHANNEL, (uint32_t)fr);

            if (!(step_flag_run & FLAG_RUN_PICK_R))
            {
                SET_BIT(step_flag_run, FLAG_RUN_PICK_R);
                HAL_TIM_PWM_Start(&PICK_TIM, PICK_R_CHANNEL);
            }
        }
        else
        {
            if (step_flag_run & FLAG_RUN_PICK_R)
            {
                CLEAR_BIT(step_flag_run, FLAG_RUN_PICK_R);
                HAL_TIM_PWM_Stop(&PICK_TIM, PICK_R_CHANNEL);
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
}

static void nip_update(const ctrl_rc_t *rc)
{
    if (switch_is_down(rc->rc.s[RC_SW_L]) &&
        switch_is_down(rc->rc.s[RC_SW_R]))
    {
        int16_t fr = rc->rc.ch[RC_CH_LV];

        if (fr > LIMIT_LOW_NIP)
        {
            gpio_pin_set(NIP_DIR_GPIO_Port, NIP_DIR_Pin);
        }
        else if (fr < -LIMIT_LOW_NIP)
        {
            gpio_pin_reset(NIP_DIR_GPIO_Port, NIP_DIR_Pin);
            fr = (int16_t)(-fr);
        }

        if (fr > LIMIT_LOW_NIP)
        {
            nip_set((uint32_t)fr);

            if (!(step_flag_run & FLAG_RUN_NIP))
            {
                SET_BIT(step_flag_run, FLAG_RUN_NIP);
                HAL_TIM_PWM_Start(&NIP_TIM, NIP_CHANNEL);
            }
        }
        else
        {
            if (step_flag_run & FLAG_RUN_NIP)
            {
                CLEAR_BIT(step_flag_run, FLAG_RUN_NIP);
                HAL_TIM_PWM_Stop(&NIP_TIM, NIP_CHANNEL);
            }
        }
    }
}

void task_step(void *pvParameters)
{
    (void)pvParameters;

    const ctrl_rc_t *rc = ctrl_rc_point();

    step_init();

    osDelay(1000);

    while (1)
    {
        pick_update(rc);
        nip_update(rc);

        osDelay(2);
    }
}

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
