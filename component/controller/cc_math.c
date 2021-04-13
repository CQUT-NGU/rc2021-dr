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

/* Includes ------------------------------------------------------------------*/
#include "cc_math.h"

/* Private includes ----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/

float inv_sqrt(float x)
{
    float xh = 0.5f * x;
    long  i  = *(long *)&x;

    i = 0x5f3759df - (i >> 1);
    x = *(float *)&i;
    x *= (1.5f - (xh * x * x));

    return x;
}

float const_loop(float x, float min, float max)
{
    if (max < min)
    {
        return x;
    }

    if (x > max)
    {
        float len = max - min;
        while (x > max)
        {
            x -= len;
        }
    }
    else if (x < min)
    {
        float len = max - min;
        while (x < min)
        {
            x += len;
        }
    }

    return x;
}

/************************ (C) COPYRIGHT tqfx *******************END OF FILE****/
