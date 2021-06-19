/**
 * *****************************************************************************
 * @file         task_shoot.c
 * @brief        task shoot
 * @author       ngu
 * @date         20210509
 * @version      1
 * @copyright    Copyright (c) 2021
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
#define MAX_MOTOR_CAN_CURRENT 16000.0F

typedef struct
{
    const motor_t *motor;
    ca_pid_f32_t   pid;
    int16_t        out;
    float          angle;
    float          angle_min;
    float          angle_max;
} shoot_t;

shoot_t shoot;

void shoot_angle_update(motor_t *mo)
{
    int16_t delta = (int16_t)(mo->ecd_last - mo->ecd);
    if (delta > ECD_RANGE_HALF)
    {
        delta -= ECD_RANGE;
    }
    else if (delta < -ECD_RANGE_HALF)
    {
        delta += ECD_RANGE;
    }
    shoot.angle += MOTOR_ROLLER_RAD * delta;
    if (shoot.angle > (float)M_PI * 2)
    {
        shoot.angle -= (float)M_PI * 2;
    }
}

static void shoot_cli(shoot_t *pdata)
{
    static float angle = 0;
    while (1)
    {
        other_ctrl(1000, 0, 0, 0);
        osDelay(100);
        if (*(int32_t *)&angle == *(int32_t *)&pdata->angle)
        {
            pdata->angle = 0;
            break;
        }
        angle = pdata->angle;
    }
}

void task_shoot(void *pvParameters)
{
    (void)pvParameters;

    osDelay(1000);

    const float kpid[3] = {
        3000,
        0,
        10000,
    };

    shoot.motor     = chassis_point(4);
    shoot.angle_min = 0;
    shoot.angle_max = (float)M_PI;
    ca_pid_f32_position(&shoot.pid,
                        kpid,
                        -MAX_MOTOR_CAN_CURRENT,
                        MAX_MOTOR_CAN_CURRENT,
                        2000);

    //shoot_cli(&shoot);

    while (1)
    {
        // shoot.out = (int16_t)ca_pid_f32(&shoot.pid, 2, shoot.angle);
        // other_ctrl(shoot.out, 0, 0, 0);

        // os_printf("%g %i\r\n", shoot.angle, shoot.out);

        osDelay(2);
    }
}