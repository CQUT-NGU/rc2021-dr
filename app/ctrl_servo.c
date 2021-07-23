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

#define SERVO_UPDATE(htim, ch, flag, ref, set, inc) \
    do                                              \
    {                                               \
        int delta = (int)((set) - (ref));           \
        if (delta >= (int)(inc))                    \
        {                                           \
            (ref) = (ref) + (inc);                  \
        }                                           \
        else if (delta <= -(int)(inc))              \
        {                                           \
            (ref) = (ref) - (inc);                  \
        }                                           \
        else                                        \
        {                                           \
            (ref) = (set);                          \
            delta = 0;                              \
        }                                           \
        if (delta)                                  \
        {                                           \
            CLEAR_BIT(servo.match, flag);           \
        }                                           \
        else                                        \
        {                                           \
            SET_BIT(servo.match, flag);             \
        }                                           \
        __HAL_TIM_SET_COMPARE(&htim, ch, (ref));    \
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

    pot_set_pwm(pwm[0]);
    clip_set_pwm(pwm[1]);
}

void pot_set_pwm(uint32_t pwm)
{
    pot_set(pwm);
    servo.pot = pwm;
    __HAL_TIM_SET_COMPARE(&SERVO_TIM, POT_CHANNEL, pwm);
}

void clip_set_pwm(uint32_t pwm)
{
    clip_set(pwm);
    servo.clip = pwm;
    __HAL_TIM_SET_COMPARE(&SERVO_TIM, CLIP_CHANNEL, pwm);
}

void servo_update(uint32_t inc)
{
    SERVO_UPDATE(SERVO_TIM,
                 POT_CHANNEL,
                 SERVO_MATCH_POT,
                 servo.pot,
                 servo.pot_set,
                 inc);
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
                 servo.clip_set,
                 inc);
#endif /* SERVO_CONFIG_CLIP_FAST */
}

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
