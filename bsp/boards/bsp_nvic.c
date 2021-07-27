/**
 * *****************************************************************************
 * @file         bsp_nvic.c
 * @brief        nvic of boards
 * @author       NGU
 * @date         20210427
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
 * *****************************************************************************
*/

#include "bsp_nvic.h"

void nvic_reset(void)
{
    __ASM volatile("cpsid i");
    NVIC_SystemReset();
}

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
