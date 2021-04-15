/**
 * *****************************************************************************
 * @file         task_ins.c/h
 * @brief        use bmi088 to calculate the euler angle. no use ist8310, so only
 *               enable data ready pin to save cpu time.enalbe bmi088 data ready
 *               enable spi DMA to save the time spi transmit
 * @author       ngu
 * @date         20210101
 * @version      1
 * @copyright    Copyright (C) 2021
 * @code         utf-8                                                  @endcode
 * *****************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TASK_INS_H__
#define __TASK_INS_H__

/* Includes ------------------------------------------------------------------*/
/* Private includes ----------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#define INS_ROLL  0
#define INS_PITCH 1
#define INS_YAW   2

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

extern void task_ins(void const *pvParameters);

__END_DECLS

/* Private defines -----------------------------------------------------------*/

/* __TASK_INS_H__ ------------------------------------------------------------*/
#endif /* __TASK_INS_H__ */

/************************ (C) COPYRIGHT tqfx *******************END OF FILE****/
