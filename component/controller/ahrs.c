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

/* Includes ------------------------------------------------------------------*/
#include "ahrs.h"

/* Private includes ----------------------------------------------------------*/
#include "cc_math.h"

#include <math.h>

/* Private define ------------------------------------------------------------*/
#undef KI
#undef KP
#undef KP2

/* Proportional gain governs rate of convergence to accelerometer/magnetometer */
#define KP 2.0f
/* Integral gain governs rate of convergence of gyroscope biases */
#define KI 0.01f
/* 2 * proportional gain (Kp) */
#define KP2 0.1f

/* Private macro -------------------------------------------------------------*/

#undef NORM3
/* Normalise 3 measurement */
#define NORM3(_x, _y, _z)                                   \
                                                            \
    do                                                      \
    {                                                       \
        /* Fast inverse square-root */                      \
        float norm = inv_sqrt(_x * _x + _y * _y + _z * _z); \
        _x *= norm;                                         \
        _y *= norm;                                         \
        _z *= norm;                                         \
    } while (0);

#undef NORM4
/* Normalise 4 measurement */
#define NORM4(_w, _x, _y, _z)                     \
                                                  \
    do                                            \
    {                                             \
        /* Fast inverse square-root */            \
        float norm = inv_sqrt(_w * _w + _x * _x + \
                              _y * _y + _z * _z); \
        _w *= norm;                               \
        _x *= norm;                               \
        _y *= norm;                               \
        _z *= norm;                               \
    } while (0);

/* Private variables ---------------------------------------------------------*/

/* error integral */
static volatile float eix; /* error integral of x axis */
static volatile float eiy; /* error integral of x axis */
static volatile float eiz; /* error integral of x axis */

/* Private function prototypes -----------------------------------------------*/

#ifndef __CC_MATH_H__
/**
 * @brief        fast inverse square-root, to calculate 1/sqrt(x)
 *               http://en.wikipedia.org/wiki/Fast_inverse_square_root
 * @param[in]    x: the number need to be calculated
 * @return       float 1/sqrt(x)
*/
static float inv_sqrt(float x);
#endif /* __CC_MATH_H__ */

/* Private user code ---------------------------------------------------------*/

#ifndef __CC_MATH_H__
float inv_sqrt(float x)
{
    float xh = 0.5f * x;
    long  i  = *(long *)&x;

    i = 0x5f3759df - (i >> 1);
    x = *(float *)&i;
    x *= (1.5f - (xh * x * x));

    return x;
}
#endif /* __CC_MATH_H__ */

/* Private macro -------------------------------------------------------------*/
#undef x /* Remove x */
#undef y /* Remove y */
#undef z /* Remove z */
#define x 0 /* x axis */
#define y 1 /* y axis */
#define z 2 /* z axis */

/* Private user code ---------------------------------------------------------*/

/**
 * @brief        Initialize quaternion
 * @param[in]    q[4]: quaternionq[0]+q[1]* i +q[2]* j +q[3]* k
 * @param[in]    hx:   x axis direction of Earth's magnetic field
 * @param[in]    hy:   y axis direction of Earth's magnetic field
*/
void ahrs_quat_init(float   q[4],
                    int16_t hx,
                    int16_t hy)
{
#ifdef BOARD_IS_DOWN
    if (hx < 0 && hy < 0)
    {
        if (fabsf(hx / hy) >= 1)
        {
            q[0] = -0.005f;
            q[1] = -0.199f;
            q[2] = 0.979f;
            q[3] = -0.0089f;
        }
        else
        {
            q[0] = -0.008f;
            q[1] = -0.555f;
            q[2] = 0.83f;
            q[3] = -0.002f;
        }
    }
    else if (hx < 0 && hy > 0)
    {
        if (fabsf(hx / hy) >= 1)
        {
            q[0] = 0.005f;
            q[1] = -0.199f;
            q[2] = -0.978f;
            q[3] = 0.012f;
        }
        else
        {
            q[0] = 0.005f;
            q[1] = -0.553f;
            q[2] = -0.83f;
            q[3] = -0.0023f;
        }
    }
    else if (hx > 0 && hy > 0)
    {
        if (fabsf(hx / hy) >= 1)
        {
            q[0] = 0.0012f;
            q[1] = -0.978f;
            q[2] = -0.199f;
            q[3] = -0.005f;
        }
        else
        {
            q[0] = 0.0023f;
            q[1] = -0.83f;
            q[2] = -0.553f;
            q[3] = 0.0023f;
        }
    }
    else if (hx > 0 && hy < 0)
    {
        if (fabsf(hx / hy) >= 1)
        {
            q[0] = 0.0025f;
            q[1] = 0.978f;
            q[2] = -0.199f;
            q[3] = 0.008f;
        }
        else
        {
            q[0] = 0.0025f;
            q[1] = 0.83f;
            q[2] = -0.56f;
            q[3] = 0.0045f;
        }
    }
#else
    if (hx < 0 && hy < 0)
    {
        if (fabsf(hx / hy) >= 1)
        {
            q[0] = 0.195f;
            q[1] = -0.015f;
            q[2] = 0.0043f;
            q[3] = 0.979f;
        }
        else
        {
            q[0] = 0.555f;
            q[1] = -0.015f;
            q[2] = 0.006f;
            q[3] = 0.829f;
        }
    }
    else if (hx < 0 && hy > 0)
    {
        if (fabsf(hx / hy) >= 1)
        {
            q[0] = -0.193f;
            q[1] = -0.009f;
            q[2] = -0.006f;
            q[3] = 0.979f;
        }
        else
        {
            q[0] = -0.552f;
            q[1] = -0.0048f;
            q[2] = -0.0115f;
            q[3] = 0.8313f;
        }
    }
    else if (hx > 0 && hy > 0)
    {
        if (fabsf(hx / hy) >= 1)
        {
            q[0] = -0.9785f;
            q[1] = 0.008f;
            q[2] = -0.02f;
            q[3] = 0.195f;
        }
        else
        {
            q[0] = -0.9828f;
            q[1] = 0.002f;
            q[2] = -0.0167f;
            q[3] = 0.5557f;
        }
    }
    else if (hx > 0 && hy < 0)
    {
        if (fabsf(hx / hy) >= 1)
        {
            q[0] = -0.979f;
            q[1] = 0.0116f;
            q[2] = -0.0167f;
            q[3] = -0.195f;
        }
        else
        {
            q[0] = -0.83f;
            q[1] = 0.014f;
            q[2] = -0.012f;
            q[3] = -0.556f;
        }
    }
#endif /* BOARD_IS_DOWN */
    else
    {
        q[0] = 1;
        q[1] = 0;
        q[2] = 0;
        q[3] = 0;
    }
}

/**
 * @brief        Quaternion to Euler Angle
 * @param[in]    q[4]: quaternionq[0]+q[1]* i +q[2]* j +q[3]* k
 * @param[out]   rol:  rotate around the x axis, roll
 * @param[out]   pit:  rotate around the y axis, pitch
 * @param[out]   yaw:  rotate around the z axis, yaw
*/
void ahrs_euler_angle(float  q[4],
                      float *rol,
                      float *pit,
                      float *yaw)
{
    /* yaw   -pi----pi */
    *yaw = atan2f(2 * q[0] * q[3] + 2 * q[1] * q[2],
                  1 - 2 * q[2] * q[2] - 2 * q[3] * q[3]);
    /* pitch -pi/2----pi/2 */
    *pit = asinf(2 * q[0] * q[2] - 2 * q[1] * q[3]);
    /* roll  -pi----pi */
    *rol = atan2f(2 * q[0] * q[1] + 2 * q[2] * q[3],
                  1 - 2 * q[1] * q[1] - 2 * q[2] * q[2]);

    /*$$
    \left[\begin{array}{c}
    \gamma \\
    \theta \\
    \psi
    \end{array}\right] = \left[\begin{array}{c}
    \operatorname{atan} 2\left(2\left(q_{0} q_{1}+q_{2} q_{3}\right), 1-2\left(q_{1}^{2}+q_{2}^{2}\right)\right) \\
    \operatorname{asin} \left(2\left(q_{0} q_{2}-q_{3} q_{1}\right)\right) \\
    \operatorname{atan} 2\left(2\left(q_{0} q_{3}+q_{1} q_{2}\right), 1-2\left(q_{2}^{2}+q_{3}^{2}\right)\right)
    \end{array}\right]
    $$*/
}

/**
 * @brief        Mehony AHRS attitude calculation with magnetometer
 * @param[in]    q[4]: quaternionq[0]+q[1]* i +q[2]* j +q[3]* k
 * @param[in]    g:    x,y,z axis of gyroscope
 * @param[in]    a:    x,y,z axis of accelerometer
 * @param[in]    m:    x,y,z axis of magnetometer
 * @param[in]    ht:   half of sampling period
*/
void ahrs_mahony(volatile float q[4],
                 volatile float g[3],
                 volatile float a[3],
                 volatile float m[3],
                 volatile float ht)
{
    /* Avoids NaN in magnetometer normalisation */
    if (m[x] || m[y] || m[z]) /* mx != 0 && my != 0 && mz != 0 */
    {
        /* Normalise magnetometer measurement */
        NORM3(m[x], m[y], m[z]);
    }
    else /* mx == 0, my = 0, mz = 0 */
    {
        ahrs_mahony_imu(q, g, a, ht);
        return;
    }

    /* Auxiliary variables to avoid repeated arithmetic */
    float q0q0 = q[0] * q[0];
    float q0q1 = q[0] * q[1];
    float q0q2 = q[0] * q[2];
    float q0q3 = q[0] * q[3];
    float q1q1 = q[1] * q[1];
    float q1q2 = q[1] * q[2];
    float q1q3 = q[1] * q[3];
    float q2q2 = q[2] * q[2];
    float q2q3 = q[2] * q[3];
    float q3q3 = q[3] * q[3];

    /* Avoids NaN in accelerometer normalisation */
    if (a[x] || a[y] || a[z]) /* ax != 0 && ay != 0 && az != 0 */
    {
        /* Normalise accelerometer measurement */
        NORM3(a[x], a[y], a[z]);

        /* Reference direction of Earth's magnetic field */
        float hx = 2.0f * (m[x] * (0.5f - q2q2 - q3q3) +
                           m[y] * (q1q2 - q0q3) +
                           m[z] * (q1q3 + q0q2) /**/);
        float hy = 2.0f * (m[x] * (q1q2 + q0q3) +
                           m[y] * (0.5f - q1q1 - q3q3) +
                           m[z] * (q2q3 - q0q1) /**/);
        float bx = sqrtf(hx * hx + hy * hy);
        float bz = 2.0f * (m[x] * (q1q3 - q0q2) +
                           m[y] * (q2q3 + q0q1) +
                           m[z] * (0.5f - q1q1 - q2q2) /**/);

        /*$$
        \left[\begin{array}{ccc}
        1 - 2(q_{2}^{2} + q_{3}^{2}) &
        2\left(q_{1} q_{2}-q_{0} q_{3}\right) &
        2\left(q_{1} q_{3}+q_{0} q_{2}\right) \\
        2\left(q_{1} q_{2}+q_{0} q_{3}\right) &
        1 - 2(q_{1}^{2} + q_{3}^{2}) &
        2\left(q_{2} q_{3}-q_{0} q_{1}\right) \\
        2\left(q_{1} q_{3}-q_{0} q_{2}\right) &
        2\left(q_{2} q_{3}+q_{0} q_{1}\right) &
        1 - 2(q_{1}^{2} + q_{2}^{2})
        \end{array}\right]
        $$*/

        /* Estimated direction of gravity and magnetic field (v and w) */
        float vx = 2.0f * (q1q3 - q0q2);
        float vy = 2.0f * (q0q1 + q2q3);
        //float vz = 2.0f * (q0q0 - 0.5f + q3q3);
        float vz = q0q0 - q1q1 - q2q2 + q3q3;
        float wx = 2.0f * (bx * (0.5f - q2q2 - q3q3) + (q1q3 - q0q2) * bz);
        float wy = 2.0f * (bx * (q1q2 - q0q3) + (q0q1 + q2q3) * bz);
        float wz = 2.0f * (bx * (q0q2 + q1q3) + (0.5f - q1q1 - q2q2) * bz);

        /**
         * Error is sum of cross product between reference
         * direction of fields and direction measured by sensors
        */
        float ex = (a[y] * vz - a[z] * vy) + (m[y] * wz - m[z] * wy);
        float ey = (a[z] * vx - a[x] * vz) + (m[z] * wx - m[x] * wz);
        float ez = (a[x] * vy - a[y] * vx) + (m[x] * wy - m[y] * wx);

        /* PI */
        if (ex || ey || ez) /* ex != 0 && ey != 0 && ez != 0 */
        {
            eix += ex * KI * ht;
            eiy += ey * KI * ht;
            eiz += ez * KI * ht;

            g[x] += KP * ex + eix;
            g[y] += KP * ey + eiy;
            g[z] += KP * ez + eiz;
        }
    }

    float q_0 = q[0];
    float q_1 = q[1];
    float q_2 = q[2];
    /* Integrate quaternion rate */
    q[0] += ht * (-q_1 * g[x] - q_2 * g[y] - q[3] * g[z]);
    q[1] += ht * (+q_0 * g[x] + q_2 * g[z] - q[3] * g[y]);
    q[2] += ht * (+q_0 * g[y] - q_1 * g[z] + q[3] * g[x]);
    q[3] += ht * (+q_0 * g[z] + q_1 * g[y] - q_2 * g[x]);

    /* Normalise quaternion */
    NORM4(q[0], q[1], q[2], q[3]);
}

/**
 * @brief        Mehony AHRS attitude calculation without magnetometer
 * @param[in]    q[4]: quaternionq[0]+q[1]* i +q[2]* j +q[3]* k
 * @param[in]    g:    x,y,z axis of gyroscope
 * @param[in]    a:    x,y,z axis of accelerometer
 * @param[in]    ht:   half of sampling period
*/
void ahrs_mahony_imu(volatile float q[4],
                     volatile float g[3],
                     volatile float a[3],
                     volatile float ht)
{
    /* Auxiliary variables to avoid repeated arithmetic */
    float q0q0 = q[0] * q[0];
    float q0q1 = q[0] * q[1];
    float q0q2 = q[0] * q[2];
    float q1q1 = q[1] * q[1];
    float q1q3 = q[1] * q[3];
    float q2q2 = q[2] * q[2];
    float q2q3 = q[2] * q[3];
    float q3q3 = q[3] * q[3];

    /* Avoids NaN in accelerometer normalisation */
    if (a[x] || a[y] || a[z]) /* ax != 0 && ay != 0 && az != 0 */
    {
        /* Normalise accelerometer measurement */
        NORM3(a[x], a[y], a[z]);

        /* Estimated direction of gravity and magnetic field (v and w) */
        float vx = 2.0f * (q1q3 - q0q2);
        float vy = 2.0f * (q0q1 + q2q3);
        //float vz = 2.0f * (q0q0 - 0.5f + q3q3);
        float vz = q0q0 - q1q1 - q2q2 + q3q3;

        /**
         * Error is sum of cross product between reference
         * direction of fields and direction measured by sensors
        */
        float ex = (a[y] * vz - a[z] * vy);
        float ey = (a[z] * vx - a[x] * vz);
        float ez = (a[x] * vy - a[y] * vx);

        /* PI */
        if (ex || ey || ez) /* ex != 0 && ey != 0 && ez != 0 */
        {
            eix += ex * KI * ht;
            eiy += ey * KI * ht;
            eiz += ez * KI * ht;

            g[x] += KP * ex + eix;
            g[y] += KP * ey + eiy;
            g[z] += KP * ez + eiz;
        }
    }

    float q_0 = q[0];
    float q_1 = q[1];
    float q_2 = q[2];
    /* Integrate quaternion rate */
    q[0] += ht * (-q_1 * g[x] - q_2 * g[y] - q[3] * g[z]);
    q[1] += ht * (+q_0 * g[x] + q_2 * g[z] - q[3] * g[y]);
    q[2] += ht * (+q_0 * g[y] - q_1 * g[z] + q[3] * g[x]);
    q[3] += ht * (+q_0 * g[z] + q_1 * g[y] - q_2 * g[x]);

    /* Normalise quaternion */
    NORM4(q[0], q[1], q[2], q[3]);
}

/**
 * @brief        Madgwick AHRS attitude calculation with magnetometer
 * @param[in]    q[4]: quaternionq[0]+q[1]* i +q[2]* j +q[3]* k
 * @param[in]    g:    x,y,z axis of gyroscope
 * @param[in]    a:    x,y,z axis of accelerometer
 * @param[in]    m:    x,y,z axis of magnetometer
 * @param[in]    t:    sampling period
*/
void ahrs_madgwick(volatile float q[4],
                   volatile float g[3],
                   volatile float a[3],
                   volatile float m[3],
                   volatile float t)
{
    /* Avoids NaN in magnetometer normalisation */
    if (m[x] || m[y] || m[z]) /* mx != 0 && my != 0 && mz != 0 */
    {
        /* Normalise magnetometer measurement */
        NORM3(m[x], m[y], m[z]);
    }
    else /* mx == 0, my = 0, mz = 0 */
    {
        ahrs_madgwick_imu(q, g, a, t);
        return;
    }

    /* Rate of change of quaternion from gyroscope */
    float q_dot1 = 0.5f * (-q[1] * g[x] - q[2] * g[y] - q[3] * g[z]);
    float q_dot2 = 0.5f * (q[0] * g[x] + q[2] * g[z] - q[3] * g[y]);
    float q_dot3 = 0.5f * (q[0] * g[y] - q[1] * g[z] + q[3] * g[x]);
    float q_dot4 = 0.5f * (q[0] * g[z] + q[1] * g[y] - q[2] * g[x]);

    /* Avoids NaN in accelerometer normalisation */
    if (a[x] || a[y] || a[z]) /* ax != 0 && ay != 0 && az != 0 */
    {
        /* Normalise accelerometer measurement */
        NORM3(a[x], a[y], a[z]);

        /* Auxiliary variables to avoid repeated arithmetic */
        float _2q0mx = 2.0f * q[0] * m[x];
        float _2q0my = 2.0f * q[0] * m[y];
        float _2q0mz = 2.0f * q[0] * m[z];
        float _2q1mx = 2.0f * q[1] * m[x];
        float _2q0   = 2.0f * q[0];
        float _2q1   = 2.0f * q[1];
        float _2q2   = 2.0f * q[2];
        float _2q3   = 2.0f * q[3];
        float _2q0q2 = 2.0f * q[0] * q[2];
        float _2q2q3 = 2.0f * q[2] * q[3];
        float q0q0   = q[0] * q[0];
        float q0q1   = q[0] * q[1];
        float q0q2   = q[0] * q[2];
        float q0q3   = q[0] * q[3];
        float q1q1   = q[1] * q[1];
        float q1q2   = q[1] * q[2];
        float q1q3   = q[1] * q[3];
        float q2q2   = q[2] * q[2];
        float q2q3   = q[2] * q[3];
        float q3q3   = q[3] * q[3];

        /* Reference direction of Earth's magnetic field */
        float hx = m[x] * q0q0 -
                   _2q0my * q[3] +
                   _2q0mz * q[2] +
                   m[x] * q1q1 +
                   _2q1 * m[y] * q[2] +
                   _2q1 * m[z] * q[3] -
                   m[x] * q2q2 -
                   m[x] * q3q3;
        float hy = _2q0mx * q[3] +
                   m[y] * q0q0 -
                   _2q0mz * q[1] +
                   _2q1mx * q[2] -
                   m[y] * q1q1 +
                   m[y] * q2q2 +
                   _2q2 * m[z] * q[3] -
                   m[y] * q3q3;
        float _2bx = sqrtf(hx * hx + hy * hy);
        float _2bz = -_2q0mx * q[2] +
                     _2q0my * q[1] +
                     m[z] * q0q0 +
                     _2q1mx * q[3] -
                     m[z] * q1q1 +
                     _2q2 * m[y] * q[3] -
                     m[z] * q2q2 +
                     m[z] * q3q3;
        float _4bx = 2.0f * _2bx;
        float _4bz = 2.0f * _2bz;

        /* Gradient decent algorithm corrective step */
        float s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - a[x]) +
                   _2q1 * (2.0f * q0q1 + _2q2q3 - a[y]) -
                   _2bz * q[2] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m[x]) +
                   (-_2bx * q[3] + _2bz * q[1]) *
                       (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m[y]) +
                   _2bx * q[2] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - m[z]);
        float s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - a[x]) +
                   _2q0 * (2.0f * q0q1 + _2q2q3 - a[y]) -
                   4.0f * q[1] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - a[z]) +
                   _2bz * q[3] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m[x]) +
                   (_2bx * q[2] + _2bz * q[0]) *
                       (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m[y]) +
                   (_2bx * q[3] - _4bz * q[1]) *
                       (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - m[z]);
        float s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - a[x]) +
                   _2q3 * (2.0f * q0q1 + _2q2q3 - a[y]) -
                   4.0f * q[2] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - a[z]) +
                   (-_4bx * q[2] - _2bz * q[0]) *
                       (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m[x]) +
                   (_2bx * q[1] + _2bz * q[3]) *
                       (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m[y]) +
                   (_2bx * q[0] - _4bz * q[2]) *
                       (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - m[z]);
        float s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - a[x]) +
                   _2q2 * (2.0f * q0q1 + _2q2q3 - a[y]) +
                   (-_4bx * q[3] + _2bz * q[1]) *
                       (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m[x]) +
                   (-_2bx * q[0] + _2bz * q[2]) *
                       (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m[y]) +
                   _2bx * q[1] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - m[z]);

        /* normalise step magnitude */
        NORM4(s0, s1, s2, s3);

        /* Apply feedback step */
        q_dot1 -= KP2 * s0;
        q_dot2 -= KP2 * s1;
        q_dot3 -= KP2 * s2;
        q_dot4 -= KP2 * s3;
    }

    /* Integrate rate of change of quaternion to yield quaternion */
    q[0] += q_dot1 * t;
    q[1] += q_dot2 * t;
    q[2] += q_dot3 * t;
    q[3] += q_dot4 * t;

    /* Normalise quaternion */
    NORM4(q[0], q[1], q[2], q[3]);
}

/**
 * @brief        Madgwick AHRS attitude calculation without magnetometer
 * @param[in]    q[4]: quaternionq[0]+q[1]* i +q[2]* j +q[3]* k
 * @param[in]    g:    x,y,z axis of gyroscope
 * @param[in]    a:    x,y,z axis of accelerometer
 * @param[in]    t:    sampling period
*/
void ahrs_madgwick_imu(volatile float q[4],
                       volatile float g[3],
                       volatile float a[3],
                       volatile float t)
{
    /* Rate of change of quaternion from gyroscope */
    float q_dot1 = 0.5f * (-q[1] * g[x] - q[2] * g[y] - q[3] * g[z]);
    float q_dot2 = 0.5f * (q[0] * g[x] + q[2] * g[z] - q[3] * g[y]);
    float q_dot3 = 0.5f * (q[0] * g[y] - q[1] * g[z] + q[3] * g[x]);
    float q_dot4 = 0.5f * (q[0] * g[z] + q[1] * g[y] - q[2] * g[x]);

    /* Avoids NaN in accelerometer normalisation */
    if (a[x] || a[y] || a[z]) /* ax != 0 && ay != 0 && az != 0 */
    {
        /* Normalise accelerometer measurement */
        NORM3(a[x], a[y], a[z]);

        /* Auxiliary variables to avoid repeated arithmetic */
        float _2q0 = 2.0f * q[0];
        float _2q1 = 2.0f * q[1];
        float _2q2 = 2.0f * q[2];
        float _2q3 = 2.0f * q[3];
        float _4q0 = 4.0f * q[0];
        float _4q1 = 4.0f * q[1];
        float _4q2 = 4.0f * q[2];
        float _8q1 = 8.0f * q[1];
        float _8q2 = 8.0f * q[2];
        float q0q0 = q[0] * q[0];
        float q1q1 = q[1] * q[1];
        float q2q2 = q[2] * q[2];
        float q3q3 = q[3] * q[3];

        /* Gradient decent algorithm corrective step */
        float s0 = _4q0 * q2q2 +
                   _2q2 * a[x] +
                   _4q0 * q1q1 -
                   _2q1 * a[y];
        float s1 = _4q1 * q3q3 -
                   _2q3 * a[x] +
                   4.0f * q0q0 * q[1] -
                   _2q0 * a[y] -
                   _4q1 +
                   _8q1 * q1q1 +
                   _8q1 * q2q2 +
                   _4q1 * a[z];
        float s2 = 4.0f * q0q0 * q[2] +
                   _2q0 * a[x] +
                   _4q2 * q3q3 -
                   _2q3 * a[y] -
                   _4q2 +
                   _8q2 * q1q1 +
                   _8q2 * q2q2 +
                   _4q2 * a[z];
        float s3 = 4.0f * q1q1 * q[3] -
                   _2q1 * a[x] +
                   4.0f * q2q2 * q[3] -
                   _2q2 * a[y];

        /* normalise step magnitude */
        NORM4(s0, s1, s2, s3);

        /* Apply feedback step */
        q_dot1 -= KP2 * s0;
        q_dot2 -= KP2 * s1;
        q_dot3 -= KP2 * s2;
        q_dot4 -= KP2 * s3;
    }

    /* Integrate rate of change of quaternion to yield quaternion */
    q[0] += q_dot1 * t;
    q[1] += q_dot2 * t;
    q[2] += q_dot3 * t;
    q[3] += q_dot4 * t;

    /* Normalise quaternion */
    NORM4(q[0], q[1], q[2], q[3]);
}

/* Remove macro --------------------------------------------------------------*/
#undef x /* Remove x */
#undef y /* Remove y */
#undef z /* Remove z */

/************************ (C) COPYRIGHT tqfx *******************END OF FILE****/
