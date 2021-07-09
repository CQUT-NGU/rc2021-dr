/**
 * *****************************************************************************
 * @file         task_shoot.c
 * @brief        task shoot
 * @author       NGU
 * @date         20210703
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
 * *****************************************************************************
*/

#include "ca.h"
#include "bsp.h"
#include "ctrl.h"

#if USED_OS
#include "cmsis_os.h"
#endif /* USED_OS */

/* chassis 3508 max motor control current */
#define MAX_MOTOR_CAN_CURRENT 0x4000
/* chassis 3508 max motor calibration control current */
#define MAX_MOTOR_CLI_CURRENT 2000

/* M3508 rmp change to speed */
/* x * 2 * PI / 60 / 19 */
#define M3508_MOTOR_RPM_TO_VECTOR 0.005511566058929462F

/**
 * @brief        aiming signal
*/
typedef enum
{
    SIGNAL_AIMING_NONE = (1 << 0),  //!< none
    SIGNAL_AIMING_DO = (1 << 1),    //!< aim do
    SIGNAL_AIMING_DONE = (1 << 2),  //!< aim done
} signal_aiming_t;

typedef enum
{
    SHOOT_STATS_RESET = (1 << 0),  //!< reset
    SHOOT_STATS_DO = (1 << 1),     //!< do
    SHOOT_STATS_CLI = (1 << 2),    //!< calibration
    SHOOT_STATS_BACK = (1 << 3),   //!< back
    SHOOT_STATS_STOP = (1 << 4),   //!< stop
} shoot_stats_t;

typedef struct
{
    shoot_stats_t stats;
    const motor_t *motor;
    ca_pid_f32_t pidv;
    float v;
    // float v_set;
    ca_pid_f32_t pida;
    float angle;
    float angle_min;
    float angle_max;
    int16_t out;
} shoot_t;

shoot_t shoot;

void shoot_angle_update(motor_t *mo)
{
    int delta = (int)(mo->ecd_last - mo->ecd);
    if (delta > ECD_RANGE_HALF)
    {
        delta = (int)(delta - ECD_RANGE);
    }
    else if (delta < -ECD_RANGE_HALF)
    {
        delta = (int)(delta + ECD_RANGE);
    }
    shoot.angle += MOTOR_ROLLER_RAD * (float)delta;
}

void task_shoot(void *pvParameters)
{
    (void)pvParameters;

    osDelay(1000);

    /* Initialization block */
    {
        shoot.stats = SHOOT_STATS_RESET;
        /* Get motor data address */
        shoot.motor = chassis_point(4);
        /* Speed pid */
        const float kpidv[3] = {
            1000,
            1,
            0,
        };
        /* Angle pid */
        const float kpida[3] = {
            1000,
            1,
            0,
        };
        /* Speed pid initialization */
        ca_pid_f32_position(&shoot.pidv,
                            kpidv,
                            -MAX_MOTOR_CAN_CURRENT,
                            MAX_MOTOR_CAN_CURRENT,
                            MAX_MOTOR_CAN_CURRENT);
        /* Angle pid initialization */
        ca_pid_f32_position(&shoot.pida,
                            kpida,
                            -MAX_MOTOR_CLI_CURRENT,
                            MAX_MOTOR_CLI_CURRENT,
                            MAX_MOTOR_CLI_CURRENT);
        /* Set the limit of the angle */
        shoot.angle_min = 0;
        shoot.angle_max = 1;
    }

    static float angle = 0;
    /* The data address of the host computer */
    ctrl_pc_t *pc = (ctrl_pc_t *)ctrl_pc_point();

    const ctrl_rc_t *rc = ctrl_rc_point();

    static int8_t signal_aiming = SIGNAL_AIMING_NONE;

    for (;;)
    {
        /* Update speed and acceleration data */
        {
            shoot.v = shoot.motor->v_rpm * M3508_MOTOR_RPM_TO_VECTOR;
        }

        /* Aiming signal control */
        {
            if (switch_is_mid(rc->rc.s[RC_SW_L]) &&
                switch_is_down(rc->rc.s[RC_SW_R]))
            {
                /* restart control */
                if (rc->rc.ch[RC_CH_LV] < -220)
                {
                    pc->c = 0;
                }

                /* Start sending aiming signal */
                if (signal_aiming == SIGNAL_AIMING_NONE &&
                    rc->rc.ch[RC_CH_LV] > 220)
                {
                    signal_aiming = SIGNAL_AIMING_DO;
                }

                /* End sending aiming signal */
                else if (signal_aiming != SIGNAL_AIMING_NONE &&
                         rc->rc.ch[RC_CH_LV] < 220)
                {
                    signal_aiming = SIGNAL_AIMING_NONE;
                }
            }

            /* Send aiming signal */
            if (signal_aiming == SIGNAL_AIMING_DO)
            {
                signal_aiming = SIGNAL_AIMING_DONE;

                usart_dma_tx(&huart_os, (const void *)"a\n", 2);
            }
        }

        /* Process the data of the host computer */
        if (pc->c == 'a')
        {
            pc->c = 0;
            float x = pc->x;
            if (x > 0)
            {
                if (shoot.stats != SHOOT_STATS_STOP)
                {
                    // shoot.v_set = pc->x;
                    shoot.stats = SHOOT_STATS_DO;
                }
                else
                {
                    shoot.stats = SHOOT_STATS_BACK;
                }
            }
            else if (x < 0)
            {
                shoot.stats = SHOOT_STATS_CLI;
            }
        }

        /* Output calculation block */
        if (shoot.stats == SHOOT_STATS_DO)
        {
            if (shoot.angle < shoot.angle_max)
            {
                shoot.out = -MAX_MOTOR_CAN_CURRENT;
            }
            else
            {
                shoot.stats = SHOOT_STATS_STOP;
            }
        }
        else if (shoot.stats == SHOOT_STATS_BACK)
        {
            angle = shoot.angle;
            uint8_t count = 10;
            do
            {
                shoot.out = (int16_t)ca_pid_f32(&shoot.pida,
                                                shoot.angle_min,
                                                shoot.angle);
                other_ctrl(shoot.out, 0, 0, 0);

                osDelay(10);

                if (*(int32_t *)&angle == *(int32_t *)&shoot.angle)
                {
                    --count;
                }
                angle = shoot.angle;
            } while (count);
            shoot.angle = 0;
            shoot.stats = SHOOT_STATS_RESET;
        }
        else if (shoot.stats == SHOOT_STATS_CLI)
        {
            angle = 0;
            uint8_t count = 10;
            uint8_t wait = 200;
            do
            {
                other_ctrl(MAX_MOTOR_CLI_CURRENT, 0, 0, 0);

                osDelay(10);

                if (*(int32_t *)&angle == *(int32_t *)&shoot.angle)
                {
                    --count;
                }
                angle = shoot.angle;
            } while (--wait && count);
            shoot.angle = 0;
            shoot.stats = SHOOT_STATS_RESET;
        }

        /* stats block */
        if (shoot.stats == SHOOT_STATS_RESET)
        {
            shoot.out = 0;
        }
        else if (shoot.stats == SHOOT_STATS_STOP)
        {
            shoot.out = (int16_t)ca_pid_f32(&shoot.pidv,
                                            shoot.v,
                                            0);
        }

        /* Control motor */
        other_ctrl(shoot.out, 0, 0, 0);

        osDelay(2);
    }
}

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
