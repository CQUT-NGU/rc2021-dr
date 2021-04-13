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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TASK_CHASSIS_H__
#define __TASK_CHASSIS_H__

/* Includes ------------------------------------------------------------------*/
#include "cc.h"
#include "ctrl.h"

/* Private includes ----------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#undef __BEGIN_DECLS
#undef __END_DECLS

#if defined(__cplusplus)
#define __BEGIN_DECLS \
    extern "C"        \
    {
#define __END_DECLS }
#else
#define __BEGIN_DECLS
#define __END_DECLS
#endif /* __cplusplus */

/* Exported types ------------------------------------------------------------*/

typedef enum
{
    /* chassis will follow yaw gimbal motor relative angle */
    CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,
    /* chassis will have yaw angle(chassis_yaw) close-looped control */
    CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,
    /* chassis will have rotation speed control */
    CHASSIS_VECTOR_NO_FOLLOW_YAW,
    /* control-current will be sent to CAN bus derectly */
    CHASSIS_VECTOR_STOP,
} chassis_mode_e;

typedef struct
{
    const motor_t *measure;

    float accel;
    float v;
    float v_set;

    int16_t i_current;
} chassis_motor_t;

typedef struct
{
    /* the point to remote control */
    const ctrl_rc_t *data_rc;
    ctrl_pc_t *      data_pc;

    /* the point to the euler angle of gyro sensor */
    //const float *angle_ins;

    chassis_mode_e  mode;     /* state machine */
    chassis_motor_t motor[4]; /* chassis motor data */

    cc_pid_t pid_speed[4]; /* motor speed PID */
    cc_pid_t pid_angle;    /* follow angle PID */

    /* use first order filter to slow set-point */
    cc_lpf_t vx_slow;
    /* use first order filter to slow set-point */
    cc_lpf_t vy_slow;

    /* chassis vertical speed, positive means forward,unit m/s */
    float vx;
    /* chassis horizontal speed, positive means letf,unit m/s */
    float vy;
    /* chassis rotation speed, positive means counterclockwise,unit rad/s */
    float wz;

    /* chassis set vertical speed,positive means forward,unit m/s */
    float vx_set;
    /* chassis set horizontal speed,positive means left,unit m/s */
    float vy_set;
    /* chassis set rotation speed,positive means counterclockwise,unit rad/s */
    float wz_set;

    /* the relative angle between chassis and gimbal */
    float angle;
    /* the set relative angle */
    float angle_set;

    float yaw_set;

    float vx_max; /* max forward speed, unit m/s */
    float vx_min; /* max backward speed, unit m/s */
    float vy_max; /* max letf speed, unit m/s */
    float vy_min; /* max right speed, unit m/s */

    /* the yaw angle calculated by gyro sensor and gimbal motor */
    float yaw;
    /* the pitch angle calculated by gyro sensor and gimbal motor */
    float pitch;
    /* the roll angle calculated by gyro sensor and gimbal motor */
    float roll;

} chassis_move_t;

/* Exported functions prototypes ---------------------------------------------*/

__BEGIN_DECLS

extern void task_chassis(void *pvParameters);

__END_DECLS

/* Private defines -----------------------------------------------------------*/

/* __TASK_CHASSIS_H__ --------------------------------------------------------*/
#endif /* __TASK_CHASSIS_H__ */

/************************ (C) COPYRIGHT ngu ********************END OF FILE****/
