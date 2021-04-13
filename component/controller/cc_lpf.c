/**
 * *****************************************************************************
 * @file         cc_lpf.c/h
 * @brief        Low Pass Filter
 * @author       tqfx
 * @date         20210101
 * @version      1
 * @copyright    Copyright (c) 2021
 * @code         utf-8                                                  @endcode
 * *****************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "cc_lpf.h"

/* Private includes ----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/

void cc_lpf_init(cc_lpf_t *lpf,
                 float     param,
                 float     time)
{
    lpf->t   = time;
    lpf->k   = param;
    lpf->out = lpf->in = 0.0f;
}

void cc_lpf(cc_lpf_t *lpf,
            float     input)
{
    lpf->in  = input;
    lpf->out = lpf->k / (lpf->k + lpf->t) * lpf->out;
    lpf->out += lpf->t / (lpf->k + lpf->t) * lpf->in;
}

void cc_lpf_clear(cc_lpf_t *lpf)
{
    lpf->out = lpf->in = lpf->k = lpf->t = 0.0f;
}

/************************ (C) COPYRIGHT tqfx *******************END OF FILE****/
