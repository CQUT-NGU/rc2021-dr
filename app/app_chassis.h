/**
 * *****************************************************************************
 * @file         task_chassis.h
 * @brief        task chassis
 * @author       NGU
 * @date         20210509
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
 * *****************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_CHASSIS_H__
#define __APP_CHASSIS_H__

#include "app.h"

typedef enum
{
    /* chassis will stop */
    CHASSIS_VECTOR_STOP,
    /* chassis will have rotation speed control */
    CHASSIS_VECTOR_NORMAL,
    /* chassis will have rotation speed control slowly */
    CHASSIS_VECTOR_SLOW,
    /* chassis will have yaw angle(chassis_yaw) close-looped control */
    CHASSIS_VECTOR_YAW,
} chassis_mode_e;

typedef struct
{
    const motor_t *fb; /* feedback */

    int16_t i;   /* current value */
    float v;     /* velocity */
    float v_set; /* velocity set-point */
    float accel; /* accelerated speed */
} chassis_motor_t;

typedef struct
{
    /* the point to serial */
    ctrl_serial_t *serial;
    /* the point to remote control */
    const ctrl_rc_t *rc;

    chassis_mode_e mode;   /* state machine */
    chassis_motor_t mo[4]; /* chassis motor data */

    ca_pid_f32_t pid_speed[4]; /* motor speed PID */
    ca_pid_f32_t pid_angle;    /* follow angle PID */

    /* chassis horizontal offset, positive means letf,unit m */
    float x;
    /* chassis vertical offset, positive means forward,unit m */
    float y;
    /* chassis rotation offset, positive means counterclockwise,unit rad */
    float z;

    /* chassis horizontal speed, positive means letf,unit m/s */
    float vx;
    /* chassis vertical speed, positive means forward,unit m/s */
    float vy;
    /* chassis rotation speed, positive means counterclockwise,unit rad/s */
    float wz;

    /* use first order filter to slow set-point */
    ca_lpf_f32_t vx_slow;
    /* use first order filter to slow set-point */
    ca_lpf_f32_t vy_slow;
    /* use first order filter to slow set-point */
    ca_lpf_f32_t wz_slow;

    /* chassis set horizontal speed,positive means left,unit m/s */
    float vx_set;
    /* chassis set vertical speed,positive means forward,unit m/s */
    float vy_set;
    /* chassis set rotation speed,positive means counterclockwise,unit rad/s */
    float wz_set;

    /* the point to the euler angle of gyro sensor */
    const float *angle_ins;
    float yaw_set;

    /* the roll angle calculated by gyro sensor */
    float roll;
    /* the pitch angle calculated by gyro sensor */
    float pitch;
    /* the yaw angle calculated by gyro sensor */
    float yaw;
} chassis_move_t;

/* Enddef to prevent recursive inclusion -------------------------------------*/
#endif /* __APP_CHASSIS_H__ */

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
