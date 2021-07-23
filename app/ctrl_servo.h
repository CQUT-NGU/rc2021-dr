/**
 * *****************************************************************************
 * @file         ctrl_servo.h
 * @brief        steering gear control
 * @author       NGU
 * @date         20210502
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
 * *****************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CTRL_SERVO_H__
#define __CTRL_SERVO_H__

#include "ctrl.h"

#define SERVO_CONFIG_CLIP_FAST 1

#define SERVO_PWMMID 1500

#define SERVO_POT_PWMMAX 2000
#define SERVO_POT_PWMMID 1500
#define SERVO_POT_PWMMIN 1000

#define SERVO_CLIP_PWMMAX 2000
#define SERVO_CLIP_PWMMID 1500
#define SERVO_CLIP_PWMMIN 1000

#define SERVO_MATCH_POT  (1 << 0)  //!< servo match state for turn pot
#define SERVO_MATCH_CLIP (1 << 1)  //!< servo match state for clamp the arrow

typedef struct
{
    int match;
    uint32_t pot;
    uint32_t pot_set;
    uint32_t clip;
    uint32_t clip_set;
} ctrl_servo_t;

extern ctrl_servo_t servo;

__BEGIN_DECLS

extern void servo_init(void);

/**
 * @brief        Start generating PWM
 * @param[in]    pwm: An array of two PWM values
 * @arg          0 turn pot
 * @arg          1 clamp the arrow
*/
extern void servo_start(uint32_t pwm[2]);

extern void servo_update(uint32_t inc);

extern void pot_set_pwm(uint32_t pwm);
extern void clip_set_pwm(uint32_t pwm);

__END_DECLS

__STATIC_INLINE
void pot_set(uint32_t pwm)
{
    if (servo.pot_set != pwm)
    {
        servo.pot_set = pwm;
        CLEAR_BIT(servo.match, SERVO_MATCH_POT);
    }
    else
    {
        SET_BIT(servo.match, SERVO_MATCH_POT);
    }
}

__STATIC_INLINE
void clip_set(uint32_t pwm)
{
    if (servo.clip_set != pwm)
    {
        servo.clip_set = pwm;
        CLEAR_BIT(servo.match, SERVO_MATCH_CLIP);
    }
    else
    {
        SET_BIT(servo.match, SERVO_MATCH_CLIP);
    }
}

/* Enddef to prevent recursive inclusion -------------------------------------*/
#endif /* __CTRL_SERVO_H__ */

/************************ (C) COPYRIGHT TQFX *******************END OF FILE****/
