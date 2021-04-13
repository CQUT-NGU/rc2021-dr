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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CC_PID_H__
#define __CC_PID_H__

/* Includes ------------------------------------------------------------------*/
/* Private includes ----------------------------------------------------------*/
#include <stdint.h>

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

/**
 * @enum         cc_pid_mode_e
 * @brief        the mode pid to calculate
*/
typedef enum
{
    CC_PID_POSITION, /*!< position */
    CC_PID_DELTA     /*!< delta */
} cc_pid_mode_e;

/**
 * @struct       cc_pid_t
 * @brief        the structure of pid algorithm
*/
typedef struct
{
    uint8_t mode; /*!< The mode of pid */

    float kp; /*!< Proportional gain */
    float ki; /*!< Integral gain */
    float kd; /*!< Derivative gain */

    float max_out;   /*!< Maximum output */
    float max_out_i; /*!< Maximum intergral output */

    float set; /*!< Target */
    float ref; /*!< Reference */

    float out;   /*!< Output */
    float out_p; /*!< Proportional */
    float out_i; /*!< Integral */
    float out_d; /*!< Derivative */

    float buf_d[3]; /*!< Derivative item 0(latest) 1(last) 2(previous last) */
    float error[3]; /*!< Error item 0(latest) 1(last) 2(previous last) */

} cc_pid_t;

/* Exported functions prototypes ---------------------------------------------*/

__BEGIN_DECLS

/**
 * @brief        PID struct data init
 * @param[out]   pid:       PID struct data point
 * @param[in]    mode:      CC_PID_POSITION: normal pid \n
 *                          CC_PID_DELTA: delta pid
 * @param[in]    kpid:      0: kp, 1: ki, 2:kd
 * @param[in]    max_out:   pid max out
 * @param[in]    max_out_i: pid max iout
*/
extern void cc_pid_init(cc_pid_t *  pid,
                        uint8_t     mode,
                        const float kpid[3],
                        float       max_out,
                        float       max_out_i);

/**
 * @brief        PID struct data normal init
 * @param[in]    pid:       PID struct data point
 * @param[in]    kpid:      0: kp, 1: ki, 2:kd
 * @param[in]    max_out:   pid max out
 * @param[in]    max_out_i: pid max iout
*/
extern void cc_pid_position(cc_pid_t *  pid,
                            const float kpid[3],
                            float       max_out,
                            float       max_out_i);

/**
 * @brief        PID struct data delta init
 * @param[in]    pid:     PID struct data point
 * @param[in]    kpid:    0: kp, 1: ki, 2:kd
 * @param[in]    max_out: pid max out
*/
extern void cc_pid_delta(cc_pid_t *  pid,
                         const float kpid[3],
                         float       max_out);

/**
 * @brief        PID calculate 
 * @param[in]    pid: PID struct data point
 * @param[in]    ref: feedback data 
 * @param[in]    set: set point
 * @return       float pid out
*/
extern float cc_pid(cc_pid_t *pid,
                    float     ref,
                    float     set);

/**
 * @brief        PID out clear
 * @param[in]    pid: PID struct data point
*/
extern void cc_pid_clear(cc_pid_t *pid);

__END_DECLS

/* Private defines -----------------------------------------------------------*/

/* __CC_PID_H__ --------------------------------------------------------------*/
#endif /* __CC_PID_H__ */

/************************ (C) COPYRIGHT tqfx *******************END OF FILE****/
