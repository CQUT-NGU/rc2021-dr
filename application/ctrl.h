/**
 * *****************************************************************************
 * @file         ctrl.h
 * @brief        control
 * @author       NGU
 * @date         20210427
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
 * *****************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CTRL_H__
#define __CTRL_H__

#include "ctrl_can.h"
#include "ctrl_pc.h"
#include "ctrl_rc.h"

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

__BEGIN_DECLS

__END_DECLS

/* Enddef to prevent recursive inclusion ------------------------------------ */
#endif /* __CTRL_H__ */

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
