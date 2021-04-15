/**
 * *****************************************************************************
 * @file         cc_math.c/h
 * @brief        Calculate math
 * @author       tqfx
 * @date         20210101
 * @version      1
 * @copyright    Copyright (c) 2021
 * @code         utf-8                                                  @endcode
 * *****************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CC_MATH_H__
#define __CC_MATH_H__

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#ifndef PI
#define PI 3.141592653589793f
#endif /* PI */

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

#undef ABS
#define ABS(x) ((x) < 0 ? -(x) : (x))
#undef LIMIT
#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

/* Exported types ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/

__BEGIN_DECLS

/**
 * @brief        fast inverse square-root, to calculate 1/sqrt(x)
 *               http://en.wikipedia.org/wiki/Fast_inverse_square_root
 * @param[in]    x: the number need to be calculated
 * @return       float 1/sqrt(x)
*/
extern float inv_sqrt(float x);

extern float const_loop(float x,
                        float min,
                        float max);

__END_DECLS

/* Private defines -----------------------------------------------------------*/

#define const_rad(_) const_loop(_, -PI, PI)

/* __CC_MATH_H__ -------------------------------------------------------------*/
#endif /* __CC_MATH_H__ */

/************************ (C) COPYRIGHT tqfx *******************END OF FILE****/
