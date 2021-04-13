/**
 * *****************************************************************************
 * @file         cc_pid.c/h
 * @brief        Proportional Integral Derivative
 * @author       tqfx
 * @date         20210101
 * @version      1
 * @copyright    Copyright (c) 2021
 * @code         utf-8                                                  @endcode
 * *****************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "cc_pid.h"

/* Private includes ----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

#define MAX_LIMIT(input, max)  \
                               \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

/* Private typedef -----------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/

void cc_pid_init(cc_pid_t *  pid,
                 uint8_t     mode,
                 const float kpid[3],
                 float       max_out,
                 float       max_out_i)
{
    pid->mode      = mode;
    pid->kp        = kpid[0];
    pid->ki        = kpid[1];
    pid->kd        = kpid[2];
    pid->max_out   = max_out;
    pid->max_out_i = max_out_i;
    pid->buf_d[0] = pid->buf_d[1] = pid->buf_d[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->out = pid->out_p = pid->out_i = pid->out_d = 0.0f;
}

void cc_pid_position(cc_pid_t *  pid,
                     const float kpid[3],
                     float       max_out,
                     float       max_out_i)
{
    cc_pid_init(pid, CC_PID_POSITION, kpid, max_out, max_out_i);
}

void cc_pid_delta(cc_pid_t *pid, const float kpid[3], float max_out)
{
    cc_pid_init(pid, CC_PID_DELTA, kpid, max_out, 0.0f);
}

float cc_pid(cc_pid_t *pid,
             float     ref,
             float     set)
{
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];

    pid->set      = set;
    pid->ref      = ref;
    pid->error[0] = set - ref;

    pid->buf_d[2] = pid->buf_d[1];
    pid->buf_d[1] = pid->buf_d[0];

    switch (pid->mode)
    {
    case CC_PID_POSITION:
    {
        pid->buf_d[0] = (pid->error[0] - pid->error[1]);

        pid->out_p = pid->kp * pid->error[0]; /*!< Proportional */

        pid->out_i += pid->ki * pid->error[0]; /*!< Integral */

        pid->out_d = pid->kd * pid->buf_d[0]; /*!< Derivative */

        MAX_LIMIT(pid->out_i, pid->max_out_i);
        pid->out = pid->out_p + pid->out_i + pid->out_d;
        MAX_LIMIT(pid->out, pid->max_out);

        break;
    }

    case CC_PID_DELTA:
    {
        pid->buf_d[0] = (pid->error[0] + pid->error[2] - 2.0f * pid->error[1]);

        /*!< Proportional */
        pid->out_p = pid->kp * (pid->error[0] - pid->error[1]);

        pid->out_i = pid->ki * pid->error[0]; /*!< Integral */

        pid->out_d = pid->kd * pid->buf_d[0]; /*!< Derivative */

        pid->out += pid->out_p + pid->out_i + pid->out_d;
        MAX_LIMIT(pid->out, pid->max_out);

        break;
    }

    default:
        break;
    }

    return pid->out;
}

void cc_pid_clear(cc_pid_t *pid)
{
    pid->out = pid->set = pid->ref = 0.0f;
    pid->out_p = pid->out_i = pid->out_d = 0.0f;
    pid->buf_d[0] = pid->buf_d[1] = pid->buf_d[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
}

/************************ (C) COPYRIGHT tqfx *******************END OF FILE****/
