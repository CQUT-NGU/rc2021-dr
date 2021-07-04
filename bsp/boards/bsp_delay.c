/**
 * *****************************************************************************
 * @file         bsp_delay.c
 * @brief        delay of boards
 * @author       NGU
 * @date         20210427
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
 * *****************************************************************************
*/

#include "bsp_delay.h"

static uint8_t fac_us = 0;
static uint32_t fac_ms = 0;

void delay_init(void)
{
    fac_us = (uint8_t)(SystemCoreClock / 1000000);
    fac_ms = SystemCoreClock / 1000;
}

void delay_us(uint16_t nus)
{
    uint32_t tcnt = 0;
    uint32_t tnow = 0;
    uint32_t ticks = (uint32_t)nus * fac_us;
    uint32_t reload = SysTick->LOAD; /* SysTick Reload Value Register */
    uint32_t told = SysTick->VAL;    /* SysTick Current Value Register */
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
    uint32_t tcnt = 0;
    uint32_t tnow = 0;
    uint32_t ticks = nms * fac_ms;
    uint32_t reload = SysTick->LOAD; /* SysTick Reload Value Register */
    uint32_t told = SysTick->VAL;    /* SysTick Current Value Register */
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

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
