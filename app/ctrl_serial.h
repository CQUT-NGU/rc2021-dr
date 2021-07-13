/**
 * *****************************************************************************
 * @file         ctrl_serial.h
 * @brief        control by usart
 * @author       NGU
 * @date         20210619
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
 * *****************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CTRL_SERIAL_H__
#define __CTRL_SERIAL_H__

#include "ctrl.h"

#define BUFSIZ_OS 256
extern uint8_t *os_buf;

typedef struct
{
    volatile float x;
    volatile float y;
    volatile float z;

    volatile uint8_t c;
} ctrl_serial_t;

__BEGIN_DECLS

/**
 * @brief        Get serial control data point
 * @return       ctrl_serial_t serial control data point
*/
extern ctrl_serial_t *ctrl_serial_point(void);

/**
 * @brief        print string to USART
 * @param[in]    format: printf format
*/
extern void dma_printf(const char *fmt, ...);

/**
 * @brief        print string to USART
 * @param[in]    format: printf format
*/
extern void os_printf(const char *format, ...);

/**
 * @brief        print float data to USART by little-endian mode
 * @param[in]    n: number of float data
 * @param[in]    ...: float data
*/
extern void os_justfloat(uint8_t n, ...);

extern void os_dma_init(void);

__END_DECLS

/* Enddef to prevent recursive inclusion -------------------------------------*/
#endif /* __CTRL_SERIAL_H__ */

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
