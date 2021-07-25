/**
 * *****************************************************************************
 * @file         app_defense.c
 * @brief        defense application
 * @author       NGU
 * @date         20210703
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
 * *****************************************************************************
*/

#include "app.h"

#define DEFENSE_RC_DEADLINE     10
#define DEFENSE_JET_TIME_MS     300
#define DEFENSE_CONTROL_TIME_MS 2
#define DEFENSE_CONTROL_TIME    0.002F

#define DEFENSE_AIM_NONE (0)       //!< none
#define DEFENSE_AIM_DO   (1 << 0)  //!< aim do
#define DEFENSE_AIM_DONE (1 << 1)  //!< aim done

#define DEFENSE_JET_NONE  (0)       //!< none
#define DEFENSE_JET_ON    (1 << 0)  //!< open the jet
#define DEFENSE_JET_OFF   (1 << 1)  //!< shot down the jet
#define DEFENSE_JET_CNT   (1 << 2)  //!< jet counting
#define DEFENSE_JET_LEFT  (1 << 4)  //!< jet on the left
#define DEFENSE_JET_RIGHT (1 << 5)  //!< jet on the right
#define DEFENSE_JET_COUNT (DEFENSE_JET_TIME_MS / DEFENSE_CONTROL_TIME_MS)

#define DEFENSE_ROTATE_NONE  (0)
#define DEFENSE_ROTATE_STATE (1 << 0)  //!< change state

#define M3505_MOTOR_SPEED_PID_KP       16000.0F
#define M3505_MOTOR_SPEED_PID_KI       10.0F
#define M3505_MOTOR_SPEED_PID_KD       0.0F
#define M3505_MOTOR_SPEED_PID_MAX_OUT  MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0F

#define LIMIT_RC(x, max) ((x) > -(max) && ((x) < (max)) ? 0 : x)

defense_t defense;

__STATIC_INLINE
void jet_left_on(void)
{
    if (!READ_BIT(defense.jet, DEFENSE_JET_ON))
    {
        gpio_pin_set(SHOT0_GPIO_Port, SHOT0_Pin);
        SET_BIT(defense.jet, DEFENSE_JET_ON | DEFENSE_JET_CNT | DEFENSE_JET_LEFT);
    }
}

__STATIC_INLINE
void jet_right_on(void)
{
    if (!READ_BIT(defense.jet, DEFENSE_JET_ON))
    {
        gpio_pin_set(SHOT1_GPIO_Port, SHOT1_Pin);
        SET_BIT(defense.jet, DEFENSE_JET_ON | DEFENSE_JET_CNT | DEFENSE_JET_RIGHT);
    }
}

__STATIC_INLINE
void jet_off(void)
{
    if (READ_BIT(defense.jet, DEFENSE_JET_OFF))
    {
        CLEAR_BIT(defense.jet, DEFENSE_JET_ON | DEFENSE_JET_OFF);
    }
}

__STATIC_INLINE
void aim_on(void)
{
    if (!READ_BIT(defense.aim, DEFENSE_AIM_DONE))
    {
        SET_BIT(defense.aim, DEFENSE_AIM_DO);
    }
}

__STATIC_INLINE
void aim_off(void)
{
    if (READ_BIT(defense.aim, DEFENSE_AIM_DONE))
    {
        CLEAR_BIT(defense.aim, DEFENSE_AIM_DONE);
    }
}

__STATIC_INLINE
void rotate_on(void)
{
    if (defense.rotate == DEFENSE_ROTATE_NONE)
    {
        defense.rotate = DEFENSE_ROTATE_STATE;
        gpio_pin_toggle(DEFENSE_GPIO_Port, DEFENSE_Pin);
    }
}

void rotate_off(void)
{
    if (defense.rotate != DEFENSE_ROTATE_NONE)
    {
        defense.rotate = DEFENSE_ROTATE_NONE;
    }
}

void task_defense(void *pvParameters)
{
    (void)pvParameters;

    osDelay(1);

    /* Initialization block */
    {
        static float kpid_v[3] = {
            M3505_MOTOR_SPEED_PID_KP,
            M3505_MOTOR_SPEED_PID_KI,
            M3505_MOTOR_SPEED_PID_KD,
        };

        servo_init();
        uint32_t pwm[2] = {
            SERVO_PWMMID + 222,
            900,
        };
        servo_start(pwm);

        pick_init();

        ca_pid_f32_position(defense.pid,
                            kpid_v,
                            -M3505_MOTOR_SPEED_PID_MAX_OUT,
                            M3505_MOTOR_SPEED_PID_MAX_OUT,
                            M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }

    /* The data address of the host computer */
    ctrl_serial_t *serial = ctrl_serial_point();
    const ctrl_rc_t *rc = ctrl_rc_point();
    defense.mo->fb = defense_point();

    for (;;)
    {
        int16_t value = rc->rc.ch[RC_CH_S];
        if (value > 650)
        {
            rotate_on();
        }
        else
        {
            rotate_off();
        }

        if (switch_is_mid(rc->rc.s[RC_SW_L]) &&
            (switch_is_mid(rc->rc.s[RC_SW_R]) ||
             switch_is_down(rc->rc.s[RC_SW_R])))
        {
            value = LIMIT_RC(rc->rc.ch[RC_CH_LV], DEFENSE_RC_DEADLINE);
            if (value > 330)
            {
                pot_set(SERVO_PWMMID - 111);
            }
            if (value < -330)
            {
                pot_set(SERVO_PWMMID + 222);
            }
        }
        else if (switch_is_mid(rc->rc.s[RC_SW_L]) &&
                 switch_is_down(rc->rc.s[RC_SW_R]))
        {
            value = rc->rc.ch[RC_CH_LV];
            /* restart control */
            if (value < -220)
            {
                serial->c = 0;
            }
            /* Start sending aiming signal */
            else if (value > 220)
            {
                aim_on();
            }
            /* End sending aiming signal */
            else if (value < 220)
            {
                aim_off();
            }
        }

        if (rc->rc.ch[RC_CH_LV] < RC_ROCKER_MIN + DEFENSE_RC_DEADLINE)
        {
            /* 2 rps */
            value = LIMIT_RC(rc->rc.ch[RC_CH_LH], DEFENSE_RC_DEADLINE);
            defense.mo->v_set = value * (5.0F / RC_ROCKER_MIN);
        }
        else
        {
            if (switch_is_down(rc->rc.s[RC_SW_L]) &&
                switch_is_down(rc->rc.s[RC_SW_R]))
            {
                value = -LIMIT_RC(rc->rc.ch[RC_CH_RH], DEFENSE_RC_DEADLINE);
                pick_set(value * (1 << 4));  // 3200 div

                value = LIMIT_RC(rc->rc.ch[RC_CH_LV], DEFENSE_RC_DEADLINE);
                if (value > 0)
                {
                    clip_set((uint32_t)(SERVO_PWMMID + (value * 800) / 660 - 500));
                }
                else
                {
                    clip_set_pwm(1000);
                }

                value = LIMIT_RC(rc->rc.ch[RC_CH_RV], DEFENSE_RC_DEADLINE);
                if (value > 330)
                {
                    const uint32_t idx = 17500;

                    clip_set_pwm(900);
                    pick_index(idx);
                    do
                    {
                        pick_update(PICK_PWM_DELTA, PICK_PWM_DIVIDE);
                        osDelay(DEFENSE_CONTROL_TIME_MS);
                    } while (step.idx != idx);
                    osDelay(500);

                    clip_set_pwm(1800);
                    osDelay(500);

                    pick_index(0);
                    do
                    {
                        pick_update(PICK_PWM_DELTA, PICK_PWM_DIVIDE);
                        osDelay(DEFENSE_CONTROL_TIME_MS);
                    } while (step.idx != 0);
                    osDelay(500);

                    clip_set_pwm(900);
                    osDelay(500);

                    pick_index(idx);
                    do
                    {
                        pick_update(PICK_PWM_DELTA, PICK_PWM_DIVIDE);
                        osDelay(DEFENSE_CONTROL_TIME_MS);
                    } while (step.idx != idx);
                }
                else if (value < -330)
                {
                    pick_zero_cli(PICK_INDEX_CLI);
                }
            }
            else if (switch_is_down(rc->rc.s[RC_SW_L]) &&
                     switch_is_mid(rc->rc.s[RC_SW_R]))
            {
                value = rc->rc.ch[RC_CH_RH];
                if (value > 440)
                {
                    jet_right_on();
                }
                else if (value < -440)
                {
                    jet_left_on();
                }
                else
                {
                    jet_off();
                }
            }
        }

        if (serial->c == 'h')
        {
            serial->c = 0;
            pick_index((uint32_t)serial->x);
        }

        {
            /* Send aiming signal */
            if (READ_BIT(defense.aim, DEFENSE_AIM_DO))
            {
                CLEAR_BIT(defense.aim, DEFENSE_AIM_DO);
                SET_BIT(defense.aim, DEFENSE_AIM_DONE);
                usart_dma_tx(&huart_os, (const void *)"a\n", 2);
            }

            if (READ_BIT(defense.jet, DEFENSE_JET_CNT) &&
                defense.jet_count++ == DEFENSE_JET_COUNT)
            {
                defense.jet_count = 0;
                SET_BIT(defense.jet, DEFENSE_JET_OFF);
                if (READ_BIT(defense.jet, DEFENSE_JET_LEFT))
                {
                    CLEAR_BIT(defense.jet, DEFENSE_JET_CNT | DEFENSE_JET_LEFT);
                    gpio_pin_reset(SHOT0_GPIO_Port, SHOT0_Pin);
                }
                if (READ_BIT(defense.jet, DEFENSE_JET_RIGHT))
                {
                    CLEAR_BIT(defense.jet, DEFENSE_JET_CNT | DEFENSE_JET_RIGHT);
                    gpio_pin_reset(SHOT1_GPIO_Port, SHOT1_Pin);
                }
                usart_dma_tx(&huart_os, (const void *)"b\n", 2);
            }

            pick_update(PICK_PWM_DELTA, PICK_PWM_DIVIDE);

            if (defense.tick % 2 == 0)
            {
                servo_update(1);
            }
        }

        ++defense.tick;

        osDelay(DEFENSE_CONTROL_TIME_MS);
    }
}

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
