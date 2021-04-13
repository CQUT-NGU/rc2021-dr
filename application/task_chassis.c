/**
 * *****************************************************************************
 * @file         task_chassis.c/h
 * @brief        task chassis
 * @author       ngu
 * @date         20210101
 * @version      1
 * @copyright    Copyright (c) 2021
 * @code         utf-8                                                  @endcode
 * *****************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "task_chassis.h"

/* Private includes ----------------------------------------------------------*/
#include "bsp.h"
#include "cc.h"
#include "ctrl.h"
#include "main.h"

#if USED_OS
#include "cmsis_os.h"
#endif /* USED_OS */

/* Private define ------------------------------------------------------------*/

/* in the beginning of task ,wait a time */
#define CHASSIS_TASK_INIT_TIME 357

/* the channel num of controlling horizontal speed */
#define CHASSIS_Y_CHANNEL 2
/* the channel num of controlling vertial speed */
#define CHASSIS_X_CHANNEL 3

/* in some mode, can use remote control to control rotation speed */
#define CHASSIS_WZ_CHANNEL 0

/* the channel of choosing chassis mode */
#define CHASSIS_MODE_CHANNEL 0
/* rocker value (max 660) change to horizontal speed (m/s) */
#define CHASSIS_VY_RC_SEN 0.005f
/* rocker value (max 660) change to vertial speed (m/s) */
#define CHASSIS_VX_RC_SEN 0.006f
/* in following yaw angle mode, rocker value add to angle */
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
/* in not following yaw angle mode, rocker value change to rotation speed */
#define CHASSIS_WZ_RC_SEN 0.01f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

/* rocker value deadline */
#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

#define MOTOR_DISTANCE_TO_CENTER 0.2f

/* chassis task control time 2ms */
#define CHASSIS_CONTROL_TIME_MS 2
/* chassis task control time 0.002s */
#define CHASSIS_CONTROL_TIME 0.002f
/* chassis control frequence, no use now */
#define CHASSIS_CONTROL_FREQUENCE 500.0f
/* chassis 3508 max motor control current */
#define MAX_MOTOR_CAN_CURRENT 16000.0f
/* press the key, chassis will swing */
#define SWING_KEY KEY_PRESSED_OFFSET_CTRL
/* chassi forward, back, left, right key */
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY  KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY  KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

/* M3508 rmp change to chassis speed */
#define M3508_MOTOR_RPM_TO_VECTOR       0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

/* single chassis motor max speed */
#define MAX_WHEEL_SPEED 4.0f
/* chassis forward or back max speed */
#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f
/* chassis left or right max speed */
#define NORMAL_MAX_CHASSIS_SPEED_Y 1.5f

#define CHASSIS_WZ_SET_SCALE 0.1f

/* when chassis is not set to move, swing max angle */
#define SWING_NO_MOVE_ANGLE 0.7f
/* when chassis is set to move, swing max angle */
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f

/* chassis motor speed PID */
#define M3505_MOTOR_SPEED_PID_KP       15000.0f
#define M3505_MOTOR_SPEED_PID_KI       10.0f
#define M3505_MOTOR_SPEED_PID_KD       0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT  MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

/* chassis follow angle PID */
#define CHASSIS_FOLLOW_GIMBAL_PID_KP       40.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI       0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD       0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT  6.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f

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

static chassis_move_t move;

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
    const static float pid_v[3] = {
        M3505_MOTOR_SPEED_PID_KP,
        M3505_MOTOR_SPEED_PID_KI,
        M3505_MOTOR_SPEED_PID_KD,
    };

    /* chassis angle PID */
    const static float pid_yaw[3] = {
        CHASSIS_FOLLOW_GIMBAL_PID_KP,
        CHASSIS_FOLLOW_GIMBAL_PID_KI,
        CHASSIS_FOLLOW_GIMBAL_PID_KD,
    };

    const static float lpf_x = {CHASSIS_ACCEL_X_NUM};
    const static float lpf_y = {CHASSIS_ACCEL_Y_NUM};

    uint8_t i; /* conut */

    /* in beginning， chassis mode is stop */
    move->mode = CHASSIS_VECTOR_STOP;

    /* get remote control point */
    move->data_rc = ctrl_rc_point();

    /* get pc control point */
    move->data_pc = ctrl_pc_point();

    /* get gyro sensor euler angle point */

    /* get gimbal motor data point */

    /* get chassis motor data point, initialize motor speed PID */
    for (i = 0U; i != 4U; ++i)
    {
        move->motor[i].measure = chassis_point(i);
        cc_pid_position(&move->pid_speed[i],
                        pid_v,
                        M3505_MOTOR_SPEED_PID_MAX_OUT,
                        M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }

    /* initialize angle PID */
    cc_pid_position(&move->pid_angle,
                    pid_yaw,
                    CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT,
                    CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);

    /* first order low-pass filter  replace ramp function */
    cc_lpf_init(&move->vx_slow, lpf_x, CHASSIS_CONTROL_TIME);
    cc_lpf_init(&move->vy_slow, lpf_y, CHASSIS_CONTROL_TIME);

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
    uint8_t i;

    for (i = 0U; i != 4; ++i)
    {
        /* update motor speed, accel is differential of speed PID */
        move->motor[i].v = move->motor[i].measure->v_rpm *
                           CHASSIS_MOTOR_RPM_TO_VECTOR_SEN;

        move->motor[i].accel = move->pid_speed[i].buf_d[0] *
                               CHASSIS_CONTROL_FREQUENCE;
    }

    /**
     * calculate horizontal speed, vertical speed, rotation speed,
     * left hand rule
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
     * calculate chassis euler angle,
     * if chassis add a new gyro sensor,please change this code
    */
#if 0
    move->yaw =
        const_rad(*(move->angle_ins + INS_YAW_ADDRESS_OFFSET));
    // - move->yaw_motor->relative_angle);

    move->pitch =
        const_rad(*(move->angle_ins + INS_PITCH_ADDRESS_OFFSET));
    //- move->pitch_motor->relative_angle);

    move->roll =
        *(move->angle_ins + INS_ROLL_ADDRESS_OFFSET);
#endif
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
    int16_t vx_channel;
    int16_t vy_channel;

    float vx_set_channel;
    float vy_set_channel;

    /**
     * deadline, because some remote control need be calibrated,
     * the value of rocker is not zero in middle place 
    */
    vx_channel = LIMIT_RC(move->data_rc->rc.ch[CHASSIS_Y_CHANNEL],
                          CHASSIS_RC_DEADLINE);

    vy_channel = LIMIT_RC(move->data_rc->rc.ch[CHASSIS_X_CHANNEL],
                          CHASSIS_RC_DEADLINE);

    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * CHASSIS_VY_RC_SEN;

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

    /**
     * first order low-pass replace ramp function,
     * calculate chassis speed set-point to improve control performance
    */

    cc_lpf(&move->vx_slow, vx_set_channel);
    cc_lpf(&move->vy_slow, vy_set_channel);

    /* stop command, need not slow change, set zero derectly */
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN &&
        vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        move->vx_slow.out = 0.0f;
    }

    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN &&
        vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        move->vy_slow.out = 0.0f;
    }

    *vx_set = move->vx_slow.out;
    *vy_set = move->vy_slow.out;
}

/**
 * @brief        set chassis control mode,
 * @param[out]   move: "move" valiable point
*/
static void chassis_mode_set(chassis_move_t *move)
{
    if (move->data_rc->rc.s[!CHASSIS_MODE_CHANNEL] != RC_SW_MID)
    {
        move->mode = CHASSIS_VECTOR_STOP;
        return;
    }

    if (move->data_rc->rc.s[CHASSIS_MODE_CHANNEL] == RC_SW_UP)
    {
        move->mode = CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW;
    }
    else if (move->data_rc->rc.s[CHASSIS_MODE_CHANNEL] == RC_SW_MID)
    {
        move->mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
    }
    else if (move->data_rc->rc.s[CHASSIS_MODE_CHANNEL] == RC_SW_DOWN)
    {
        move->mode = CHASSIS_VECTOR_STOP;
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

        move->data_pc->c = 0;
        move->data_pc->x = 0;
        move->data_pc->y = 0;
        move->data_pc->z = 0;

        break;
    }

    case CHASSIS_VECTOR_NO_FOLLOW_YAW:
    {
        chassis_rc(vx_set, vy_set, move);
        *wz_set = -move->data_rc->rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_WZ_RC_SEN;

        switch (move->data_pc->c)
        {
        case 'v':
        {
            *vx_set = move->data_pc->x;
            *vy_set = move->data_pc->y;
            *wz_set = move->data_pc->z;
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

        *wz_set = -3.14159265f * move->data_rc->rc.ch[CHASSIS_WZ_CHANNEL] / 660;

        switch (move->data_pc->c)
        {
        default:
            break;
        }

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
    float vx_set = 0.0f;
    float vy_set = 0.0f;

    float angle_set = 0.0f;

    /* get three control set-point*/
    chassis_mode_ctrl(&vx_set, &vy_set, &angle_set, move);

    /* follow gimbal mode */
    if (move->mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
#if 0
        float sin_yaw = 0.0f;
        float cos_yaw = 0.0f;
        /* rotate chassis direction, make sure vertial direction follow gimbal */
        sin_yaw = arm_sin_f32(-move->yaw_motor->relative_angle);
        cos_yaw = arm_cos_f32(-move->yaw_motor->relative_angle);
        move->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
        move->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
        move->vx_set = vx_set;
        move->vy_set = vy_set;
#endif

        /* set control relative angle set-point */
        move->angle_set = const_rad(angle_set);
#if 0
        /* calculate ratation speed */
        move->wz_set = -cc_pid(&move->pid_angle, move->yaw_motor->relative_angle, move->angle_set);
#endif
        /* speed limit */
        move->vx_set = LIMIT(move->vx_set, move->vx_min, move->vx_max);
        move->vy_set = LIMIT(move->vy_set, move->vy_min, move->vy_max);
    }
    else if (move->mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        float delat_angle;

        /* set chassis yaw angle set-point */
        move->yaw_set = const_rad(angle_set);
        delat_angle   = const_rad(move->yaw_set - move->yaw);

        /* calculate rotation speed */
        move->wz_set = cc_pid(&move->pid_angle, 0.0f, delat_angle);

        /* speed limit */
        move->vx_set = LIMIT(vx_set, move->vx_min, move->vx_max);
        move->vy_set = LIMIT(vy_set, move->vy_min, move->vy_max);
    }
    else if (move->mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        /* "angle_set" is rotation speed set-point */

        move->wz_set = angle_set;
        move->vx_set = LIMIT(vx_set, move->vx_min, move->vx_max);
        move->vy_set = LIMIT(vy_set, move->vy_min, move->vy_max);
    }
    else if (move->mode == CHASSIS_VECTOR_STOP)
    {
        /* in raw mode, set-point is sent to CAN bus */

        move->vx_set      = vx_set;
        move->vy_set      = vy_set;
        move->wz_set      = angle_set;
        move->vx_slow.out = 0.0f;
        move->vy_slow.out = 0.0f;
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
    wheel_speed[0] = +vx_set - vy_set - MOTOR_DISTANCE_TO_CENTER * wz_set;

    wheel_speed[1] = +vx_set + vy_set - MOTOR_DISTANCE_TO_CENTER * wz_set;

    wheel_speed[2] = -vx_set + vy_set - MOTOR_DISTANCE_TO_CENTER * wz_set;

    wheel_speed[3] = -vx_set - vy_set - MOTOR_DISTANCE_TO_CENTER * wz_set;
}

/**
 * @brief        control loop, according to control set-point, calculate
 *               motor current, motor current will be sentto motor
 * @param[in]    move: "move" valiable point
*/
static void chassis_loop(chassis_move_t *move)
{
    float vector_max = 0.0f;
    float vector     = 0.0f;
    float temp       = 0.0f;

    float wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    uint8_t i;

    /* mecanum wheel speed calculation */
    chassis_mecanum(move->vx_set,
                    move->vy_set,
                    move->wz_set,
                    wheel_speed);

    /* calculate the max speed in four wheels, limit the max speed */
    for (i = 0U; i != 4U; ++i)
    {
        move->motor[i].v_set = wheel_speed[i];

        temp = ABS(move->motor[i].v_set);

        if (vector_max < temp)
        {
            vector_max = temp;
        }
    }

    if (vector_max > MAX_WHEEL_SPEED)
    {
        vector = MAX_WHEEL_SPEED / vector_max;

        for (i = 0U; i != 4U; ++i)
        {
            move->motor[i].v_set *= vector;
        }
    }

    /* calculate pid */
    for (i = 0U; i != 4U; ++i)
    {
        move->motor[i].i_current = (int16_t)cc_pid(&move->pid_speed[i],
                                                   move->motor[i].v,
                                                   move->motor[i].v_set);
    }
}

void task_chassis(void *pvParameters)
{
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

        osDelay(CHASSIS_CONTROL_TIME_MS);
    }

    osThreadExit();
}

/************************ (C) COPYRIGHT ngu ********************END OF FILE****/
