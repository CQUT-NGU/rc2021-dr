/**
 * *****************************************************************************
 * @file         bsp_buzzer.h
 * @brief        buzzer of boards
 * @author       NGU
 * @date         20210501
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
 * @details      TIM4_CH3(PD14) 4000Hz
 * *****************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_BUZZER_H__
#define __BSP_BUZZER_H__

#include <stdint.h>

#define BUZZER_PWM_MAX   ((168000000U >> 1U) / 4000U)
#define BUZZER_PWM_DIV1  (BUZZER_PWM_MAX >> 0U)
#define BUZZER_PWM_DIV2  (BUZZER_PWM_MAX >> 1U)
#define BUZZER_PWM_DIV4  (BUZZER_PWM_MAX >> 2U)
#define BUZZER_PWM_DIV8  (BUZZER_PWM_MAX >> 3U)
#define BUZZER_PWM_DIV16 (BUZZER_PWM_MAX >> 4U)
#define BUZZER_PWM_DIV32 (BUZZER_PWM_MAX >> 5U)
#define BUZZER_PWM_DIV64 (BUZZER_PWM_MAX >> 6U)

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

__BEGIN_DECLS

/**
 * @brief        Start buzzer pwm
*/
extern void buzzer_start(void);

/**
 * @brief        Stop buzzer pwm
*/
extern void buzzer_stop(void);

/**
 * @brief        Set pwm of the buzzer
 * @param[in]    psc: 0x0000 ~ 0x0FFF
 * @param[in]    pwm: 0x0000 ~ 0x4FFF
*/
extern void buzzer_set(uint16_t psc,
                       uint16_t pwm);

__END_DECLS

/* Enddef to prevent recursive inclusion ------------------------------------ */
#endif /* __BSP_BUZZER_H__ */

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
