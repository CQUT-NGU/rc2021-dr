/**
 * *****************************************************************************
 * @file         bsp_delay.h
 * @brief        delay of boards
 * @author       NGU
 * @date         20210427
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
 * *****************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_DELAY_H__
#define __BSP_DELAY_H__

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

__BEGIN_DECLS

/**
 * @brief        Initializes the delay function
*/
extern void delay_init(void);

/**
 * @brief        Delay microseconds
 * @param[in]    nus: 0x0000 ~ 0xFFFF
*/
extern void delay_us(uint16_t nus);

/**
 * @brief        Delay millisecond
 * @param[in]    nms: 0x0000 ~ 0xFFFF
*/
extern void delay_ms(uint16_t nms);

__END_DECLS

/* Enddef to prevent recursive inclusion ------------------------------------ */
#endif /* __BSP_DELAY_H__ */

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
