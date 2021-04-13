/**
 * *****************************************************************************
 * @file         ctrl.c/h
 * @brief        control
 * @author       ngu
 * @date         20210101
 * @version      1
 * @copyright    Copyright (c) 2021
 * @code         utf-8                                                  @endcode
 * *****************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CTRL_H__
#define __CTRL_H__

/* Includes ------------------------------------------------------------------*/
#include "ctrl_can.h"
#include "ctrl_pc.h"
#include "ctrl_rc.h"

/* Private includes ----------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
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

/* Exported types ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/

__BEGIN_DECLS

__END_DECLS

/* Private defines -----------------------------------------------------------*/

/* __CTRL_H__ ----------------------------------------------------------------*/
#endif /* __CTRL_H__ */

/************************ (C) COPYRIGHT tqfx *******************END OF FILE****/
