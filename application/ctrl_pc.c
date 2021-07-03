/**
 * *****************************************************************************
 * @file         ctrl_pc.c
 * @brief        control by usart
 * @author       NGU
 * @date         20210619
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
 * *****************************************************************************
*/

#include "ctrl_pc.h"

#include "bsp_usart.h"
#include "main.h"

#include <stdlib.h>

#undef PC_IS_FLOAT
#define PC_IS_FLOAT(x) \
    ((x >= '0' &&      \
      x <= '9') ||     \
     x == '+' ||       \
     x == '-' ||       \
     x == '.')

#undef PC_BUF_DEAL
#define PC_BUF_DEAL(x, p, pd)               \
    {                                       \
        while (p != pd && !PC_IS_FLOAT(*p)) \
        {                                   \
            ++p;                            \
        }                                   \
        if (p == pd)                        \
        {                                   \
            break;                          \
        }                                   \
        char tmpbuf[32];                    \
        char *pi = tmpbuf;                  \
        while (p != pd && PC_IS_FLOAT(*p))  \
        {                                   \
            *pi++ = *p++;                   \
        }                                   \
        *pi = 0;                            \
        x = (float)atof((char *)tmpbuf);    \
    }

static ctrl_pc_t pc; /* pc control data */

ctrl_pc_t *ctrl_pc_point(void)
{
    return &pc;
}

void os_rx_irq(volatile void *buf,
               uint16_t len)
{
    char *p = (char *)buf;

    if ('A' <= *p && *p <= 'Z')
    {
        pc.c = *(uint8_t *)buf;
        pc.x = *(float *)(p + 2);
        pc.y = *(float *)(p + 2 + 4);
        pc.z = *(float *)(p + 2 + 4 + 4);
        return;
    }
    else if ('a' <= *p && *p <= 'z')
    {
        do
        {
            char *pd = p + len;
            p += 2;

            PC_BUF_DEAL(pc.x, p, pd);
            PC_BUF_DEAL(pc.y, p, pd);
            PC_BUF_DEAL(pc.z, p, pd);

            pc.c = *(uint8_t *)buf;
            return;
        } while (0);
    }

    pc.c = 0;
}

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
