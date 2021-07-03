/**
 * *****************************************************************************
 * @file         task_led.h
 * @brief        led task
 * @author       NGU
 * @date         20210427
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
 * *****************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TASK_LED_H__
#define __TASK_LED_H__

#include <stdint.h>

#undef __BEGIN_DECLS
#undef __END_DECLS

#if defined(__cplusplus)
#define __BEGIN_DECLS \
    extern "C"        \
    {
#define __END_DECLS \
    }
#else
#define __BEGIN_DECLS
#define __END_DECLS
#endif /* __cplusplus */

__BEGIN_DECLS

extern void task_led(void *pvParameters);

__END_DECLS

/* Enddef to prevent recursive inclusion ------------------------------------ */
#endif /* __TASK_LED_H__ */

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
