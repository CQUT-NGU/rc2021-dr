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

#include "bsp.h"
#include "ca.h"
#include "ctrl.h"
#include "main.h"

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

typedef enum
{
    SHOOT_STATS_RESET,  //!< reset
    SHOOT_STATS_RUN,    //!< runnig
    SHOOT_STATS_STOP,   //!< stop
} shoot_stats_t;

typedef struct
{
    shoot_stats_t stats;
    const motor_t *motor;
    ca_pid_f32_t pidv;
    float v;
    float v_set;
    float acc;
    ca_pid_f32_t pida;
    float angle;
    float angle_set;
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

static void shoot_cli(shoot_t *pdata)
{
    static float angle = 0;

    uint16_t count = 200;

    do
    {
        other_ctrl(1000, 0, 0, 0);

        osDelay(50);

        if (*(int32_t *)&angle == *(int32_t *)&pdata->angle)
        {
            pdata->angle = 0;
            break;
        }
        angle = pdata->angle;
    } while (--count);

    pdata->out = 0;
    other_ctrl(0, 0, 0, 0);
}

void task_shoot(void *pvParameters)
{
    (void)pvParameters;

    osDelay(1000);

    /* Initialization block */
    {
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
            500,
            2,
            0,
        };
        /* Speed pid initialization */
        ca_pid_f32_position(&shoot.pidv,
                            kpidv,
                            -MAX_MOTOR_CAN_CURRENT,
                            MAX_MOTOR_CAN_CURRENT,
                            MAX_MOTOR_CLI_CURRENT);
        /* Angle pid initialization */
        ca_pid_f32_position(&shoot.pida,
                            kpida,
                            -MAX_MOTOR_CLI_CURRENT,
                            MAX_MOTOR_CLI_CURRENT,
                            MAX_MOTOR_CLI_CURRENT);
        /* Set the limit of the angle */
        shoot.angle_min = 0;
        shoot.angle_max = (float)M_PI;
        /* Start to calibrate the zero point */
        shoot_cli(&shoot);
    }

    /* The data address of the host computer */
    ctrl_pc_t *pc = (ctrl_pc_t *)ctrl_pc_point();

    while (1)
    {
        /* Update speed and acceleration data */
        {
            float v_tmp = shoot.motor->v_rpm * M3508_MOTOR_RPM_TO_VECTOR;
            shoot.acc = shoot.v - v_tmp;
            shoot.v = v_tmp;
        }

        /* Process the data of the host computer */
        {
            if (pc->c == 'a')
            {
                pc->c = 0;
                shoot.v_set = pc->x;
                shoot.stats = SHOOT_STATS_RUN;
            }
        }

        /* Output calculation block */
        {
            if (shoot.stats == SHOOT_STATS_RUN)
            {
                if (shoot.angle < 2)
                {
                    shoot.out = (int16_t)ca_pid_f32(&shoot.pidv,
                                                    shoot.v_set,
                                                    shoot.v);
                }
                else
                {
                    shoot.stats = SHOOT_STATS_STOP;
                }
            }

            if (shoot.stats == SHOOT_STATS_STOP)
            {
                if (-0.01F < shoot.v && shoot.v < 0.01F)
                {
                    shoot.stats = SHOOT_STATS_RESET;
                    ca_pid_f32_reset(&shoot.pidv);
                    float angle = shoot.angle;
                    do
                    {
                        shoot.out = (int16_t)ca_pid_f32(&shoot.pida,
                                                        shoot.angle_min,
                                                        shoot.angle);
                        other_ctrl(shoot.out, 0, 0, 0);

                        osDelay(10);

                        if (*(int32_t *)&angle == *(int32_t *)&shoot.angle)
                        {
                            shoot.out = 0;
                            shoot.angle = 0;
                            break;
                        }
                        angle = shoot.angle;
                    } while (1);
                }
                else
                {
                    if (shoot.v_set > 0.01F)
                    {
                        shoot.v_set -= 0.01F;
                    }
                    else
                    {
                        shoot.v_set = 0;
                    }

                    shoot.out = (int16_t)ca_pid_f32(&shoot.pidv,
                                                    shoot.v,
                                                    shoot.v_set);
                }
            }
        }

        /* Control motor */
        other_ctrl(shoot.out, 0, 0, 0);

        // os_printf("%i,%g,%g,%g\r\n",
        //           shoot.out,
        //           shoot.v,
        //           shoot.acc,
        //           shoot.angle);

        osDelay(2);
    }
}

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
