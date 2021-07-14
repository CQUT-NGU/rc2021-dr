/**
 * *****************************************************************************
 * @file         ctrl_servo.h
 * @brief        steering gear control
 * @author       NGU
 * @date         20210502
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
 * *****************************************************************************
*/

#include "ctrl_servo.h"

#define SERVO_TIM htim1

extern TIM_HandleTypeDef SERVO_TIM;

#define SERVO_PSC     168
#define SERVO_PWM_MAX 20000

#define POT_CHANNEL  TIM_CHANNEL_3
#define CLIP_CHANNEL TIM_CHANNEL_4

ctrl_servo_t servo = {
    .match = SERVO_MATCH_POT | SERVO_MATCH_CLIP,
};

#define SERVO_UPDATE(htim, ch, flag, ref, set)       \
    do                                               \
    {                                                \
        int delta = (int)((set) - (ref));            \
        if (delta > 0)                               \
        {                                            \
            ++(ref);                                 \
        }                                            \
        else if (delta < 0)                          \
        {                                            \
            --(ref);                                 \
        }                                            \
        if (delta)                                   \
        {                                            \
            __HAL_TIM_SET_COMPARE(&htim, ch, (ref)); \
            CLEAR_BIT(servo.match, flag);            \
        }                                            \
        else                                         \
        {                                            \
            SET_BIT(servo.match, flag);              \
        }                                            \
    } while (0)

void servo_init(void)
{
    __HAL_TIM_SET_PRESCALER(&SERVO_TIM, SERVO_PSC - 1);
    __HAL_TIM_SetAutoreload(&SERVO_TIM, SERVO_PWM_MAX - 1);
}

void servo_start(uint32_t pwm[2])
{
    HAL_TIM_PWM_Start(&SERVO_TIM, POT_CHANNEL);
    HAL_TIM_PWM_Start(&SERVO_TIM, CLIP_CHANNEL);

    servo.pot_set = pwm[0];
    servo.pot = servo.pot_set - 1;

    servo.clip_set = pwm[1];
    servo.clip = servo.clip_set - 1;
}

void servo_update(void)
{
    SERVO_UPDATE(SERVO_TIM,
                 POT_CHANNEL,
                 SERVO_MATCH_POT,
                 servo.pot,
                 servo.pot_set);
#if SERVO_CONFIG_CLIP_FAST
    if (servo.clip != servo.clip_set)
    {
        __HAL_TIM_SET_COMPARE(&SERVO_TIM, CLIP_CHANNEL, servo.clip_set);
    }
#else
    SERVO_UPDATE(SERVO_TIM,
                 CLIP_CHANNEL,
                 SERVO_MATCH_CLIP,
                 servo.clip,
                 servo.clip_set);
#endif
}

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
