/**
 * *****************************************************************************
 * @file         ist8310reg.h
 * @brief        ist8310 of divices
 * @author       ngu
 * @date         20210427
 * @version      1
 * @copyright    Copyright (c) 2021
 * @code         utf-8                                                  @endcode
 * *****************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IST8310REG_H__
#define __IST8310REG_H__

/* Includes ------------------------------------------------------------------*/
/* Private includes ----------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* IST8310 internal reg addr */
#define IST8310_ADDRESS     0x0E
#define IST8310_DEVICE_ID_A 0x10

/* IST8310 register map */
#define IST8310_WHO_AM_I 0x00
#define IST8310_R_CONFA  0x0A
#define IST8310_R_CONFB  0x0B
#define IST8310_R_MODE   0x02

#define IST8310_R_XL 0x03
#define IST8310_R_XM 0x04
#define IST8310_R_YL 0x05
#define IST8310_R_YM 0x06
#define IST8310_R_ZL 0x07
#define IST8310_R_ZM 0x08

#define IST8310_AVGCNTL 0x41
#define IST8310_PDCNTL  0x42

/* sigle measure mode */
#define IST8310_ODR_MODE 0x01

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
/* Private defines -----------------------------------------------------------*/

/* __IST8310REG_H__ ----------------------------------------------------------*/
#endif /* __IST8310REG_H__ */

/************************ (C) COPYRIGHT ngu ********************END OF FILE****/
