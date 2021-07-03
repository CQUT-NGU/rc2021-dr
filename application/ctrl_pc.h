/**
 * *****************************************************************************
 * @file         ctrl_pc.h
 * @brief        control by usart
 * @author       NGU
 * @date         20210619
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
 * *****************************************************************************
*/

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __CTRL_PC_H__
#define __CTRL_PC_H__

#include <stdint.h>

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

#undef __packed
#define __packed __attribute__((__packed__))

typedef struct
{
    float x;
    float y;
    float z;

    uint8_t c;
} ctrl_pc_t;

__BEGIN_DECLS

/**
 * @brief        Get pc control data point
 * @return       ctrl_pc_t pc control data point
*/
extern ctrl_pc_t *ctrl_pc_point(void);

__BEGIN_DECLS

/* Enddef to prevent recursive inclusion ------------------------------------ */
#endif /* __CTRL_PC_H__ */

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
