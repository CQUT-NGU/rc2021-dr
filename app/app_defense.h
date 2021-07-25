/**
 * *****************************************************************************
 * @file         app_defense.h
 * @brief        defense application
 * @author       NGU
 * @date         20210427
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
 * *****************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_DEFENSE_H__
#define __APP_DEFENSE_H__

#include "app.h"

/* rpm -> n/s */
#define DEFENSE_MOTOR_RPM_TO_VECTOR_SEN (1.0F / 60 / 19)

typedef struct
{
    const motor_t *fb; /* feedback */

    int16_t i;   /* current value */
    float v;     /* velocity */
    float v_set; /* velocity set-point */
    float accel; /* accelerated speed */
} mo_t;

typedef struct
{
    int aim;     //!< state of aiming
    int jet;     //!< state of jet
    int rotate;  //!< state of turn arrow

    unsigned int jet_count;  //!< count of jet time
    ca_pid_f32_t pid[1];     //!< motor pid
    uint32_t tick;           //!< count of run
    mo_t mo[1];              //!< motor pointer
} defense_t;

extern defense_t defense;

/* Enddef to prevent recursive inclusion -------------------------------------*/
#endif /* __APP_DEFENSE_H__ */

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/