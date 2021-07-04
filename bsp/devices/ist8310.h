/**
 * *****************************************************************************
 * @file         ist8310.h
 * @brief        ist8310 driver
 * @author       NGU
 * @date         20210427
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
 * *****************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IST8310_H__
#define __IST8310_H__

#include "ist8310reg.h"

#include "main.h"

#define IST8310_RSTN_PIN  GPIO_PIN_6
#define IST8310_RSTN_PORT GPIOG
#define IST8310_DRDY_PIN  GPIO_PIN_3
#define IST8310_DRDY_PORT GPIOG
#define IST8310_DRDY_EXTI EXTI3_IRQn

#define IST8310_NO_ERROR  0x00
#define IST8310_NO_SENSOR 0x40

#define IST8310_RST_H \
    (IST8310_RSTN_PORT->BSRR = IST8310_RSTN_PIN)
#define IST8310_RST_L \
    (IST8310_RSTN_PORT->BSRR = (uint32_t)IST8310_RSTN_PIN << 16)

typedef struct
{
    int8_t status;
    float mag[3];
} ist_t;

__BEGIN_DECLS

extern uint8_t ist8310_init(void);

extern void ist8310_read_over(uint8_t *buf,
                              ist_t *ist);
extern void ist8310_read(float mag[3]);

__END_DECLS

/* Enddef to prevent recursive inclusion ------------------------------------ */
#endif /* __IST8310_H__ */

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
