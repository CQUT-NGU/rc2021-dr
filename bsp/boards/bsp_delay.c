/**
 * *****************************************************************************
 * @file         bsp_delay.c/h
 * @brief        delay of boards
 * @author       tqfx
 * @date         20210427
 * @version      1
 * @copyright    Copyright (c) 2021
 * @code         utf-8                                                  @endcode
 * *****************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "bsp_delay.h"

/* Private includes ----------------------------------------------------------*/
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static uint8_t  fac_us = 0U;
static uint32_t fac_ms = 0U;

/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/

void delay_init(void)
{
    fac_us = SystemCoreClock / 1000000U;
    fac_ms = SystemCoreClock / 1000U;
}

void delay_us(uint16_t nus)
{
    uint32_t tcnt   = 0U;
    uint32_t tnow   = 0U;
    uint32_t ticks  = nus * fac_us;
    uint32_t reload = SysTick->LOAD; /*!< SysTick Reload Value Register */
    uint32_t told   = SysTick->VAL;  /*!< SysTick Current Value Register */
    do
    {
        tnow = SysTick->VAL;
        if (tnow == told)
        {
            continue;
        }
        if (tnow < told)
        {
            tcnt += told - tnow;
        }
        else
        {
            tcnt += reload + told - tnow;
        }
        told = tnow;
    } while (tcnt < ticks);
}

void delay_ms(uint16_t nms)
{
    uint32_t tcnt   = 0U;
    uint32_t tnow   = 0U;
    uint32_t ticks  = nms * fac_ms;
    uint32_t reload = SysTick->LOAD; /*!< SysTick Reload Value Register */
    uint32_t told   = SysTick->VAL;  /*!< SysTick Current Value Register */
    do
    {
        tnow = SysTick->VAL;
        if (tnow == told)
        {
            continue;
        }
        if (tnow < told)
        {
            tcnt += told - tnow;
        }
        else
        {
            tcnt += reload + told - tnow;
        }
        told = tnow;
    } while (tcnt < ticks);
}

/************************ (C) COPYRIGHT tqfx *******************END OF FILE****/
