/**
 * *****************************************************************************
 * @file         ahrs.c/h
 * @brief        AHRS(Automatic Heading Reference System) algorithm library
 * @details      IMU(Inertial Measurement Unit)
 *               Mehony AHRS
 *               Madgwick AHRS
 * @author       tqfx
 * @date         20210101
 * @version      1
 * @copyright    Copyright (C) 2021
 * @code         utf-8                                                  @endcode
 * *****************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AHRS_H__
#define __AHRS_H__

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
/* Exported functions prototypes ---------------------------------------------*/

__BEGIN_DECLS

/**
 * @brief        Initialize quaternion
 * @param[in]    q[4]: quaternionq[0]+q[1]* i +q[2]* j +q[3]* k
 * @param[in]    hx:   x axis direction of Earth's magnetic field
 * @param[in]    hy:   y axis direction of Earth's magnetic field
*/
extern void ahrs_quat_init(float   q[4],
                           int16_t hx,
                           int16_t hy);

/**
 * @brief        Quaternion to Euler Angle
 * @param[in]    q[4]: quaternionq[0]+q[1]* i +q[2]* j +q[3]* k
 * @param[out]   rol:  rotate around the x axis, roll
 * @param[out]   pit:  rotate around the y axis, pitch
 * @param[out]   yaw:  rotate around the z axis, yaw
*/
extern void ahrs_euler_angle(float  q[4],
                             float *rol,
                             float *pit,
                             float *yaw);

/**
 * @brief        Mehony AHRS attitude calculation with magnetometer
 * @param[in]    q[4]: quaternionq[0]+q[1]* i +q[2]* j +q[3]* k
 * @param[in]    g:    x,y,z axis of gyroscope
 * @param[in]    a:    x,y,z axis of accelerometer
 * @param[in]    m:    x,y,z axis of magnetometer
 * @param[in]    ht:   half of sampling period
*/
extern void ahrs_mahony(volatile float q[4],
                        volatile float g[3],
                        volatile float a[3],
                        volatile float m[3],
                        volatile float ht);

/**
 * @brief        Mehony AHRS attitude calculation without magnetometer
 * @param[in]    q[4]: quaternionq[0]+q[1]* i +q[2]* j +q[3]* k
 * @param[in]    g:    x,y,z axis of gyroscope
 * @param[in]    a:    x,y,z axis of accelerometer
 * @param[in]    ht:   half of sampling period
*/
extern void ahrs_mahony_imu(volatile float q[4],
                            volatile float g[3],
                            volatile float a[3],
                            volatile float ht);

/**
 * @brief        Madgwick AHRS attitude calculation with magnetometer
 * @param[in]    q[4]: quaternionq[0]+q[1]* i +q[2]* j +q[3]* k
 * @param[in]    g:    x,y,z axis of gyroscope
 * @param[in]    a:    x,y,z axis of accelerometer
 * @param[in]    m:    x,y,z axis of magnetometer
 * @param[in]    t:    sampling period
*/
extern void ahrs_madgwick(volatile float q[4],
                          volatile float g[3],
                          volatile float a[3],
                          volatile float m[3],
                          volatile float t);

/**
 * @brief        Madgwick AHRS attitude calculation without magnetometer
 * @param[in]    q[4]: quaternionq[0]+q[1]* i +q[2]* j +q[3]* k
 * @param[in]    g:    x,y,z axis of gyroscope
 * @param[in]    a:    x,y,z axis of accelerometer
 * @param[in]    t:    sampling period
*/
extern void ahrs_madgwick_imu(volatile float q[4],
                              volatile float g[3],
                              volatile float a[3],
                              volatile float t);

__END_DECLS

/* Private defines -----------------------------------------------------------*/

/* __AHRS_H__ ----------------------------------------------------------------*/
#endif /* __AHRS_H__ */

/************************ (C) COPYRIGHT tqfx *******************END OF FILE****/
