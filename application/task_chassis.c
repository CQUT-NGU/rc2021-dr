/**
 * *****************************************************************************
 * @file         task_chassis.c/h
 * @brief        task chassis
 * @author       ngu
 * @date         20210509
 * @version      1
 * @copyright    Copyright (c) 2021
 * @code         utf-8                                                  @endcode
 * *****************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "task_chassis.h"

/* Private includes ----------------------------------------------------------*/
#include "bsp.h"
#include "ca.h"
#include "ctrl.h"
#include "main.h"

#if USED_OS
#include "cmsis_os.h"
#endif /* USED_OS */

extern float ins_angle[3]; /* euler angle, unit rad */

/* Private define ------------------------------------------------------------*/

/* in the beginning of task ,wait a time */
#define CHASSIS_TASK_INIT_TIME 357

/* the channel num of controlling horizontal speed */
#define CHASSIS_X_CHANNEL 0
/* the channel num of controlling vertial speed */
#define CHASSIS_Y_CHANNEL 1

/* in some mode, can use remote control to control rotation speed */
#define CHASSIS_WZ_CHANNEL 2

/* rocker value (max 660) change to horizontal speed (m/s) */
#define CHASSIS_VX_RC_SEN 0.005F
/* rocker value (max 660) change to vertial speed (m/s) */
#define CHASSIS_VY_RC_SEN 0.005F
/* in following yaw angle mode, rocker value add to angle */
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002F
/* in not following yaw angle mode, rocker value change to rotation speed */
#define CHASSIS_WZ_RC_SEN 0.01F
/* vertial speed slowly (dm/s) */
#define CHASSIS_RC_SLOW_SEN 0.1F

#define CHASSIS_ACCEL_X_NUM 0.1F
#define CHASSIS_ACCEL_Y_NUM 0.1F
#define CHASSIS_ACCEL_Z_NUM 0.1F

/* rocker value deadline */
#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25F
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25F
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25F

/* a = 0.338 b = 0.550 */
#define MOTOR_DISTANCE_TO_CENTER 0.444F

/* chassis task control time 2ms */
#define CHASSIS_CONTROL_TIME_MS 2
/* chassis task control time 0.002s */
#define CHASSIS_CONTROL_TIME 0.002F
/* chassis control frequence, no use now */
#define CHASSIS_CONTROL_FREQUENCE 500.0F
/* chassis 3508 max motor control current */
#define MAX_MOTOR_CAN_CURRENT 16000.0F
/* press the key, chassis will swing */
#define SWING_KEY KEY_PRESSED_OFFSET_CTRL
/* chassi forward, back, left, right key */
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY  KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY  KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

/* M3508 rmp change to chassis speed */
#define M3508_MOTOR_RPM_TO_VECTOR       0.000415809748903494517209F
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

/* single chassis motor max speed */
#define MAX_WHEEL_SPEED 4.0F
/* chassis left or right max speed */
#define NORMAL_MAX_CHASSIS_SPEED_X 2.0F
/* chassis forward or back max speed */
#define NORMAL_MAX_CHASSIS_SPEED_Y 2.0F
/* chassis left or right max accelerated speed */
#define NORMAL_MAX_CHASSIS_ACC_X 0.5F
/* chassis forward or back max accelerated speed */
#define NORMAL_MAX_CHASSIS_ACC_Y 0.5F
/* chassis left or right max speed increment */
#define NORMAL_MAX_CHASSIS_SPEED_DELTA_X (NORMAL_MAX_CHASSIS_ACC_X * CHASSIS_CONTROL_TIME)
/* chassis forward or back max speed increment */
#define NORMAL_MAX_CHASSIS_SPEED_DELTA_Y (NORMAL_MAX_CHASSIS_ACC_Y * CHASSIS_CONTROL_TIME)

#define CHASSIS_WZ_SET_SCALE 0.1F

/* when chassis is not set to move, swing max angle */
#define SWING_NO_MOVE_ANGLE 0.7F
/* when chassis is set to move, swing max angle */
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795F

/* chassis motor speed PID */
#define M3505_MOTOR_SPEED_PID_KP       15000.0F
#define M3505_MOTOR_SPEED_PID_KI       10.0F
#define M3505_MOTOR_SPEED_PID_KD       0.0F
#define M3505_MOTOR_SPEED_PID_MAX_OUT  MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0F

/* chassis follow angle PID */
#define CHASSIS_FOLLOW_GIMBAL_PID_KP       10.0F
#define CHASSIS_FOLLOW_GIMBAL_PID_KI       0.0F
#define CHASSIS_FOLLOW_GIMBAL_PID_KD       0.0F
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT  (float)M_PI
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.1F

/* Private macro -------------------------------------------------------------*/

#undef LIMIT_RC
#define LIMIT_RC(x, max)           \
                                   \
    (/**/                          \
     (x) > -(max) && ((x) < (max)) \
         ? 0                       \
         : x /**/)

/* Private typedef -----------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

chassis_move_t move;

/* Private function prototypes -----------------------------------------------*/

static void chassis_mecanum(const float vx_set,
                            const float vy_set,
                            const float wz_set,
                            float       wheel_speed[4]);

static void chassis_init(chassis_move_t *move);
static void chassis_update(chassis_move_t *move);
static void chassis_rc(float *         vx_set,
                       float *         vy_set,
                       chassis_move_t *move);
static void chassis_mode_set(chassis_move_t *move);
static void chassis_mode_ctrl(float *         vx_set,
                              float *         vy_set,
                              float *         wz_set,
                              chassis_move_t *move);
static void chassis_loop_set(chassis_move_t *move);
static void chassis_loop(chassis_move_t *move);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief        "move" valiable initialization,
 *               include pid initialization,
 *               remote control data point initialization,
 *               3508 chassis motors data point initialization,
 *               gimbal motor data point initialization,
 *               and gyro sensor angle point initialization.
 * @param[out]   move: "move" valiable point
*/
static void chassis_init(chassis_move_t *move)
{
    /* chassis motor speed PID */
    const float kpid_v[3] = {
        M3505_MOTOR_SPEED_PID_KP,
        M3505_MOTOR_SPEED_PID_KI,
        M3505_MOTOR_SPEED_PID_KD,
    };

    const float kpid_p[3] = {
        5.0F,
        0.0F,
        0.0F,
    };

    /* chassis angle PID */
    const float kpid_yaw[3] = {
        CHASSIS_FOLLOW_GIMBAL_PID_KP,
        CHASSIS_FOLLOW_GIMBAL_PID_KI,
        CHASSIS_FOLLOW_GIMBAL_PID_KD,
    };

    const float lpf_x = {CHASSIS_ACCEL_X_NUM};
    const float lpf_y = {CHASSIS_ACCEL_Y_NUM};
    const float lpf_z = {CHASSIS_ACCEL_Z_NUM};

    /* in beginning， chassis mode is stop */
    move->mode = CHASSIS_VECTOR_STOP;

    /* get remote control point */
    move->data_rc = ctrl_rc_point();

    /* get pc control point */
    move->data_pc = ctrl_pc_point();

    /* get gyro sensor euler angle point */
    move->angle_ins = ins_angle;

    /* get gimbal motor data point */

    /* get chassis motor data point, initialize motor speed PID */
    for (uint8_t i = 0U; i != 4U; ++i)
    {
        move->motor[i].measure = chassis_point(i);
        ca_pid_f32_position(&move->pid_speed[i],
                            kpid_v,
                            -M3505_MOTOR_SPEED_PID_MAX_OUT,
                            M3505_MOTOR_SPEED_PID_MAX_OUT,
                            M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }

    ca_pid_f32_position(move->pid_offset,
                        kpid_p,
                        -NORMAL_MAX_CHASSIS_SPEED_X,
                        NORMAL_MAX_CHASSIS_SPEED_X,
                        0);
    ca_pid_f32_position(move->pid_offset + 1,
                        kpid_p,
                        -NORMAL_MAX_CHASSIS_SPEED_Y,
                        NORMAL_MAX_CHASSIS_SPEED_Y,
                        0);

    /* initialize angle PID */
    ca_pid_f32_position(&move->pid_angle,
                        kpid_yaw,
                        -CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT,
                        CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT,
                        CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);

    /* first order low-pass filter  replace ramp function */
    ca_lpf_f32_init(&move->vx_slow, lpf_x, CHASSIS_CONTROL_TIME);
    ca_lpf_f32_init(&move->vy_slow, lpf_y, CHASSIS_CONTROL_TIME);
    ca_lpf_f32_init(&move->wz_slow, lpf_z, CHASSIS_CONTROL_TIME);

    /* max and min speed */
    move->vx_max = NORMAL_MAX_CHASSIS_SPEED_X;
    move->vx_min = -NORMAL_MAX_CHASSIS_SPEED_X;

    move->vy_max = NORMAL_MAX_CHASSIS_SPEED_Y;
    move->vy_min = -NORMAL_MAX_CHASSIS_SPEED_Y;

    /* update data */
    chassis_update(move);
}

/**
 * @brief        chassis some measure data updata,
 *               such as motor speed, euler angle, robot speed
 * @param[out]   move: "move" valiable point
*/
static void chassis_update(chassis_move_t *move)
{
    for (uint8_t i = 0U; i != 4; ++i)
    {
        /* update motor speed, accel is differential of speed PID */
        move->motor[i].v = move->motor[i].measure->v_rpm *
                           CHASSIS_MOTOR_RPM_TO_VECTOR_SEN;

        move->motor[i].accel = (move->pid_speed[i].x[1] - move->pid_speed[i].x[0]) *
                               CHASSIS_CONTROL_FREQUENCE;
    }

    /**
     * calculate horizontal speed, vertical speed, rotation speed,
     * right hand rule
    */
    move->vx = (move->motor[0].v +
                move->motor[1].v -
                move->motor[2].v -
                move->motor[3].v) *
               MOTOR_SPEED_TO_CHASSIS_SPEED_VX;

    move->vy = (-move->motor[0].v +
                move->motor[1].v +
                move->motor[2].v -
                move->motor[3].v) *
               MOTOR_SPEED_TO_CHASSIS_SPEED_VY;

    move->wz = (-move->motor[0].v -
                move->motor[1].v -
                move->motor[2].v -
                move->motor[3].v) *
               MOTOR_SPEED_TO_CHASSIS_SPEED_WZ /
               MOTOR_DISTANCE_TO_CENTER;

    /**
     * calculate horizontal offset, vertical offset
    */
    move->x += move->vx * CHASSIS_CONTROL_TIME;
    move->y += move->vy * CHASSIS_CONTROL_TIME;

    /**
     * calculate chassis euler angle,
     * if chassis add a new gyro sensor,please change this code
    */
    move->roll = move->angle_ins[ZYX_ROLL];

    move->pitch = restrict_rad_f32(move->angle_ins[ZYX_PITCH]);

    move->yaw = restrict_rad_f32(move->angle_ins[ZYX_YAW]);
}

/**
 * @brief        accroding to the channel value of remote control,
 *               calculate chassis horizontal and vertical speed set-point
 * @param[out]   vx_set: horizontal speed set-point
 * @param[out]   vy_set: vertical speed set-point
 * @param[out]   move:   "move" valiable point
*/
static void chassis_rc(float *         vx_set,
                       float *         vy_set,
                       chassis_move_t *move)
{
    /**
     * deadline, because some remote control need be calibrated,
     * the value of rocker is not zero in middle place
    */
    int16_t vx_channel = LIMIT_RC(move->data_rc->rc.ch[CHASSIS_X_CHANNEL],
                                  CHASSIS_RC_DEADLINE);

    int16_t vy_channel = LIMIT_RC(move->data_rc->rc.ch[CHASSIS_Y_CHANNEL],
                                  CHASSIS_RC_DEADLINE);

    float vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    float vy_set_channel = vy_channel * CHASSIS_VY_RC_SEN;

    /* keyboard set speed set-point */
    if (move->data_rc->key.v & CHASSIS_RIGHT_KEY)
    {
        vx_set_channel = move->vx_max;
    }
    else if (move->data_rc->key.v & CHASSIS_LEFT_KEY)
    {
        vx_set_channel = move->vx_min;
    }

    if (move->data_rc->key.v & CHASSIS_FRONT_KEY)
    {
        vy_set_channel = move->vy_max;
    }
    else if (move->data_rc->key.v & CHASSIS_BACK_KEY)
    {
        vy_set_channel = move->vy_min;
    }

    if (move->mode == CHASSIS_VECTOR_SLOW)
    {
        /* in slow mode, the speed will decrease */
        *vx_set = vx_set_channel * CHASSIS_RC_SLOW_SEN;
        *vy_set = vy_set_channel * CHASSIS_RC_SLOW_SEN;
    }
    else
    {
        /**
         * first order low-pass replace ramp function,
         * calculate chassis speed set-point to improve control performance
        */
        *vx_set = ca_lpf_f32(&move->vx_slow, vx_set_channel);
        *vy_set = ca_lpf_f32(&move->vy_slow, vy_set_channel);
    }
}

/**
 * @brief        set chassis control mode,
 * @param[out]   move: "move" valiable point
*/
static void chassis_mode_set(chassis_move_t *move)
{
    if (!switch_is_mid(move->data_rc->rc.s[RC_SW_L]))
    {
        move->mode = CHASSIS_VECTOR_STOP;
        return;
    }

    /* middle, up */
    if (switch_is_up(move->data_rc->rc.s[RC_SW_R]))
    {
        move->mode = CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW;
    }
    /* middle, middle */
    else if (switch_is_mid(move->data_rc->rc.s[RC_SW_R]))
    {
        move->mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
    }
    /* middle, down */
    else if (switch_is_down(move->data_rc->rc.s[RC_SW_R]))
    {
        move->mode = CHASSIS_VECTOR_SLOW;
    }
}

/**
 * @brief        set control set-point. three movement param, according to
 *               difference control mode, will control corresponding movement.
 *               in the function, usually call different control function.
 * @param[out]   vx_set: usually controls vertical speed.
 * @param[out]   vy_set: usually controls horizotal speed.
 * @param[out]   wz_set: usually controls rotation speed.
 * @param[in]    move:   has all data of chassis
*/
static void chassis_mode_ctrl(float *         vx_set,
                              float *         vy_set,
                              float *         wz_set,
                              chassis_move_t *move)
{
    switch (move->mode)
    {
    case CHASSIS_VECTOR_STOP:
    {
        *vx_set = 0;
        *vy_set = 0;
        *wz_set = 0;

        break;
    }

    case CHASSIS_VECTOR_SLOW:
    case CHASSIS_VECTOR_NO_FOLLOW_YAW:
    {
        chassis_rc(vx_set, vy_set, move);

        int16_t wz_channel = LIMIT_RC(move->data_rc->rc.ch[CHASSIS_WZ_CHANNEL],
                                      CHASSIS_RC_DEADLINE);

        float wz_set_channel = -wz_channel * CHASSIS_WZ_RC_SEN;

        if (move->mode == CHASSIS_VECTOR_SLOW)
        {
            /* in slow mode, the speed will decrease */
            *wz_set = wz_set_channel * CHASSIS_RC_SLOW_SEN;
        }
        else
        {
            *wz_set = ca_lpf_f32(&move->wz_slow, wz_set_channel);
        }

        switch (move->data_pc->c)
        {
        case 'v':
        {
            *vx_set = move->data_pc->x;
            *vy_set = move->data_pc->y;
            *wz_set = move->data_pc->z;

            break;
        }

        case 'p':
        {
            float tmp = move->data_pc->x - move->x;
            if (ABS(tmp) > 0.001F)
            {
                *vx_set = ca_pid_f32(move->pid_offset,
                                     move->x,
                                     move->data_pc->x);
            }

            tmp = move->data_pc->y - move->y;
            if (ABS(tmp) > 0.001F)
            {
                *vy_set = ca_pid_f32(move->pid_offset + 1,
                                     move->y,
                                     move->data_pc->y);
            }

            break;
        }

        default:
            break;
        }

        break;
    }

    case CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW:
    {
        chassis_rc(vx_set, vy_set, move);

        *wz_set = restrict_rad_f32(move->yaw_set -
                                   move->data_rc->rc.ch[CHASSIS_WZ_CHANNEL] *
                                       CHASSIS_ANGLE_Z_RC_SEN);
        break;
    }

    default:
        break;
    }
}

/**
 * @brief        set chassis control set-point, three movement control value
 * @param[out]   move: "move" valiable point
*/
static void chassis_loop_set(chassis_move_t *move)
{
    float vx_set = 0;
    float vy_set = 0;

    float angle_set = 0;

    /* get three control set-point*/
    chassis_mode_ctrl(&vx_set, &vy_set, &angle_set, move);

    if (move->mode == CHASSIS_VECTOR_STOP)
    {
        move->vx_set = vx_set;
        move->vy_set = vy_set;
        move->wz_set = angle_set;

        ca_lpf_f32_reset(&move->vx_slow);
        ca_lpf_f32_reset(&move->vy_slow);
        ca_lpf_f32_reset(&move->wz_slow);
    }
    else if (move->mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        float delat_angle;

        /* set chassis yaw angle set-point */
        move->yaw_set = restrict_rad_f32(angle_set);
        delat_angle   = restrict_rad_f32(move->yaw_set - move->yaw);

        /* calculate rotation speed */
        move->wz_set = ca_pid_f32(&move->pid_angle, 0, delat_angle);

        /* speed limit */
        move->vx_set = LIMIT(vx_set, move->vx_min, move->vx_max);
        move->vy_set = LIMIT(vy_set, move->vy_min, move->vy_max);
    }
    else if (move->mode == CHASSIS_VECTOR_NO_FOLLOW_YAW ||
             move->mode == CHASSIS_VECTOR_SLOW)
    {
        /* "angle_set" is rotation speed set-point */
        move->wz_set = angle_set;

        /* left or right accelerated speed limit */
        float delta = vx_set - move->vx_set;
        if (delta > NORMAL_MAX_CHASSIS_SPEED_DELTA_X)
        {
            vx_set = move->vx_set + NORMAL_MAX_CHASSIS_SPEED_DELTA_X;
        }
        else if (delta < -NORMAL_MAX_CHASSIS_SPEED_DELTA_X)
        {
            vx_set = move->vx_set - NORMAL_MAX_CHASSIS_SPEED_DELTA_X;
        }
        /* forward accelerated speed limit */
        delta = vy_set - move->vy_set;
        if (delta > NORMAL_MAX_CHASSIS_SPEED_DELTA_Y)
        {
            vy_set = move->vy_set + NORMAL_MAX_CHASSIS_SPEED_DELTA_Y;
        }
        else if (delta < -NORMAL_MAX_CHASSIS_SPEED_DELTA_Y)
        {
            vy_set = move->vy_set - NORMAL_MAX_CHASSIS_SPEED_DELTA_Y;
        }

        /* speed limit */
        move->vx_set = LIMIT(vx_set, move->vx_min, move->vx_max);
        move->vy_set = LIMIT(vy_set, move->vy_min, move->vy_max);
    }
}

/**
 * @brief        four mecanum wheels speed is calculated by three param.
 * @param[in]    vx_set: horizontal speed
 * @param[in]    vy_set: vertial speed
 * @param[in]    wz_set: rotation speed
 * @param[out]   wheel_speed: four mecanum wheels speed
*/
static void chassis_mecanum(const float vx_set,
                            const float vy_set,
                            const float wz_set,
                            float       wheel_speed[4])
{
    wheel_speed[0] = +vx_set - vy_set - wz_set * MOTOR_DISTANCE_TO_CENTER;
    wheel_speed[1] = +vx_set + vy_set - wz_set * MOTOR_DISTANCE_TO_CENTER;
    wheel_speed[2] = -vx_set + vy_set - wz_set * MOTOR_DISTANCE_TO_CENTER;
    wheel_speed[3] = -vx_set - vy_set - wz_set * MOTOR_DISTANCE_TO_CENTER;
}

/**
 * @brief        control loop, according to control set-point, calculate
 *               motor current, motor current will be sentto motor
 * @param[in]    move: "move" valiable point
*/
static void chassis_loop(chassis_move_t *move)
{
    float wheel_speed[4] = {0, 0, 0, 0};

    /* mecanum wheel speed calculation */
    chassis_mecanum(move->vx_set,
                    move->vy_set,
                    move->wz_set,
                    wheel_speed);

    /* calculate the max speed in four wheels, limit the max speed */
    float vector_max = 0;
    for (uint8_t i = 0U; i != 4U; ++i)
    {
        move->motor[i].v_set = wheel_speed[i];

        float temp = ABS(move->motor[i].v_set);

        if (vector_max < temp)
        {
            vector_max = temp;
        }
    }

    if (vector_max > MAX_WHEEL_SPEED)
    {
        float vector = MAX_WHEEL_SPEED / vector_max;

        for (uint8_t i = 0U; i != 4U; ++i)
        {
            move->motor[i].v_set *= vector;
        }
    }

    /* calculate pid */
    for (uint8_t i = 0U; i != 4U; ++i)
    {
        move->motor[i].i_current = (int16_t)ca_pid_f32(&move->pid_speed[i],
                                                       move->motor[i].v,
                                                       move->motor[i].v_set);
    }
}

void task_chassis(void *pvParameters)
{
    (void)pvParameters;

    osDelay(CHASSIS_TASK_INIT_TIME);

    chassis_init(&move);

    for (;;)
    {
        chassis_mode_set(&move);

        chassis_update(&move);

        chassis_loop_set(&move);

        chassis_loop(&move);

        chassis_ctrl(move.motor[0].i_current,
                     move.motor[1].i_current,
                     move.motor[2].i_current,
                     move.motor[3].i_current);

#if 0
        os_justfloat(4U,
                     move.motor[0].v,
                     move.motor[1].v,
                     move.motor[2].v,
                     move.motor[3].v);
#elif 0
        os_printf("p:%g,%g\n", move.x, move.y);
#endif

        osDelay(CHASSIS_CONTROL_TIME_MS);
    }
}

/************************ (C) COPYRIGHT ngu ********************END OF FILE****/
