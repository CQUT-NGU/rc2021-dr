/**
 * *****************************************************************************
 * @file         task_test.c/h
 * @brief        test task
 * @author       tqfx
 * @date         20210427
 * @version      0.01
 * @copyright    Copyright (c) 2020-2021
 * *****************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __TASK_TEST_H__
#define __TASK_TEST_H__

/* Includes ------------------------------------------------------------------*/
/* Private includes ----------------------------------------------------------*/

#include <stdint.h>

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

extern void task_test(void *pvParameters);

__END_DECLS

/* Private defines -----------------------------------------------------------*/

#endif /* __TASK_TEST_H__ */

/************************ (C) COPYRIGHT tqfx *******************END OF FILE****/
