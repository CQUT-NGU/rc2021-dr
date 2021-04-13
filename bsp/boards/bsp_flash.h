/**
 * *****************************************************************************
 * @file         bsp_flash.c/h
 * @brief        flash of boards
 * @author       ngu
 * @date         20210101
 * @version      1
 * @copyright    Copyright (c) 2020-2021
 * @code         utf-8                                                  @endcode
 * *****************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_FLASH_H__
#define __BSP_FLASH_H__

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/

/* Base address of the Flash sectors */

#if FLASH_SECTOR_TOTAL > 0U
#define ADDR_FLASH_SECTOR_0 \
    ((uint32_t)0x08000000) /* Base address of Sector 0 , 16  Kbytes */
#else
#ifndef ADDR_FLASH_END
#define ADDR_FLASH_END ((uint32_t)0x08000000)
#endif /* ADDR_FLASH_END */
#endif /* FLASH_SECTOR_TOTAL */

#if FLASH_SECTOR_TOTAL > 1U
#define ADDR_FLASH_SECTOR_1 \
    ((uint32_t)0x08004000) /* Base address of Sector 1 , 16  Kbytes */
#else
#ifndef ADDR_FLASH_END
#define ADDR_FLASH_END ((uint32_t)0x08004000)
#endif /* ADDR_FLASH_END */
#endif /* FLASH_SECTOR_TOTAL */

#if FLASH_SECTOR_TOTAL > 2U
#define ADDR_FLASH_SECTOR_2 \
    ((uint32_t)0x08008000) /* Base address of Sector 2 , 16  Kbytes */
#else
#ifndef ADDR_FLASH_END
#define ADDR_FLASH_END ((uint32_t)0x08008000)
#endif /* ADDR_FLASH_END */
#endif /* FLASH_SECTOR_TOTAL */

#if FLASH_SECTOR_TOTAL > 3U
#define ADDR_FLASH_SECTOR_3 \
    ((uint32_t)0x0800C000) /* Base address of Sector 3 , 16  Kbytes */
#else
#ifndef ADDR_FLASH_END
#define ADDR_FLASH_END ((uint32_t)0x0800C000)
#endif /* ADDR_FLASH_END */
#endif /* FLASH_SECTOR_TOTAL */

#if FLASH_SECTOR_TOTAL > 4U
#define ADDR_FLASH_SECTOR_4 \
    ((uint32_t)0x08010000) /* Base address of Sector 4 , 64  Kbytes */
#else
#ifndef ADDR_FLASH_END
#define ADDR_FLASH_END ((uint32_t)0x08010000)
#endif /* ADDR_FLASH_END */
#endif /* FLASH_SECTOR_TOTAL */

#if FLASH_SECTOR_TOTAL > 5U
#define ADDR_FLASH_SECTOR_5 \
    ((uint32_t)0x08020000) /* Base address of Sector 5 , 128 Kbytes */
#else
#ifndef ADDR_FLASH_END
#define ADDR_FLASH_END ((uint32_t)0x08020000)
#endif /* ADDR_FLASH_END */
#endif /* FLASH_SECTOR_TOTAL */

#if FLASH_SECTOR_TOTAL > 6U
#define ADDR_FLASH_SECTOR_6 \
    ((uint32_t)0x08040000) /* Base address of Sector 6 , 128 Kbytes */
#else
#ifndef ADDR_FLASH_END
#define ADDR_FLASH_END ((uint32_t)0x08040000)
#endif /* ADDR_FLASH_END */
#endif /* FLASH_SECTOR_TOTAL */

#if FLASH_SECTOR_TOTAL > 7U
#define ADDR_FLASH_SECTOR_7 \
    ((uint32_t)0x08060000) /* Base address of Sector 7 , 128 Kbytes */
#else
#ifndef ADDR_FLASH_END
#define ADDR_FLASH_END ((uint32_t)0x08060000)
#endif /* ADDR_FLASH_END */
#endif /* FLASH_SECTOR_TOTAL */

#if FLASH_SECTOR_TOTAL > 8U
#define ADDR_FLASH_SECTOR_8 \
    ((uint32_t)0x08080000) /* Base address of Sector 8 , 128 Kbytes */
#else
#ifndef ADDR_FLASH_END
#define ADDR_FLASH_END ((uint32_t)0x08080000)
#endif /* ADDR_FLASH_END */
#endif /* FLASH_SECTOR_TOTAL */

#if FLASH_SECTOR_TOTAL > 9U
#define ADDR_FLASH_SECTOR_9 \
    ((uint32_t)0x080A0000) /* Base address of Sector 9 , 128 Kbytes */
#else
#ifndef ADDR_FLASH_END
#define ADDR_FLASH_END ((uint32_t)0x080A0000)
#endif /* ADDR_FLASH_END */
#endif /* FLASH_SECTOR_TOTAL */

#if FLASH_SECTOR_TOTAL > 10U
#define ADDR_FLASH_SECTOR_10 \
    ((uint32_t)0x080C0000) /* Base address of Sector 10, 128 Kbytes */
#else
#ifndef ADDR_FLASH_END
#define ADDR_FLASH_END ((uint32_t)0x080C0000)
#endif /* ADDR_FLASH_END */
#endif /* FLASH_SECTOR_TOTAL */

#if FLASH_SECTOR_TOTAL > 11U
#define ADDR_FLASH_SECTOR_11 \
    ((uint32_t)0x080E0000) /* Base address of Sector 11, 128 Kbytes */
#else
#ifndef ADDR_FLASH_END
#define ADDR_FLASH_END ((uint32_t)0x080E0000)
#endif /* ADDR_FLASH_END */
#endif /* FLASH_SECTOR_TOTAL */

#if FLASH_SECTOR_TOTAL > 12U
#define ADDR_FLASH_SECTOR_12 \
    ((uint32_t)0x08100000) /* Base address of Sector 12, 16  Kbytes */
#else
#ifndef ADDR_FLASH_END
#define ADDR_FLASH_END ((uint32_t)0x08100000)
#endif /* ADDR_FLASH_END */
#endif /* FLASH_SECTOR_TOTAL */

#if FLASH_SECTOR_TOTAL > 13U
#define ADDR_FLASH_SECTOR_13 \
    ((uint32_t)0x08104000) /* Base address of Sector 13, 16  Kbytes */
#else
#ifndef ADDR_FLASH_END
#define ADDR_FLASH_END ((uint32_t)0x08104000)
#endif /* ADDR_FLASH_END */
#endif /* FLASH_SECTOR_TOTAL */

#if FLASH_SECTOR_TOTAL > 14U
#define ADDR_FLASH_SECTOR_14 \
    ((uint32_t)0x08108000) /* Base address of Sector 14, 16  Kbytes */
#else
#ifndef ADDR_FLASH_END
#define ADDR_FLASH_END ((uint32_t)0x08108000)
#endif /* ADDR_FLASH_END */
#endif /* FLASH_SECTOR_TOTAL */

#if FLASH_SECTOR_TOTAL > 15U
#define ADDR_FLASH_SECTOR_15 \
    ((uint32_t)0x0810C000) /* Base address of Sector 15, 16  Kbytes */
#else
#ifndef ADDR_FLASH_END
#define ADDR_FLASH_END ((uint32_t)0x0810C000)
#endif /* ADDR_FLASH_END */
#endif /* FLASH_SECTOR_TOTAL */

#if FLASH_SECTOR_TOTAL > 16U
#define ADDR_FLASH_SECTOR_16 \
    ((uint32_t)0x08110000) /* Base address of Sector 16, 64  Kbytes */
#else
#ifndef ADDR_FLASH_END
#define ADDR_FLASH_END ((uint32_t)0x08110000)
#endif /* ADDR_FLASH_END */
#endif /* FLASH_SECTOR_TOTAL */

#if FLASH_SECTOR_TOTAL > 17U
#define ADDR_FLASH_SECTOR_17 \
    ((uint32_t)0x08120000) /* Base address of Sector 17, 128 Kbytes */
#else
#ifndef ADDR_FLASH_END
#define ADDR_FLASH_END ((uint32_t)0x08120000)
#endif /* ADDR_FLASH_END */
#endif /* FLASH_SECTOR_TOTAL */

#if FLASH_SECTOR_TOTAL > 18U
#define ADDR_FLASH_SECTOR_18 \
    ((uint32_t)0x08140000) /* Base address of Sector 18, 128 Kbytes */
#else
#ifndef ADDR_FLASH_END
#define ADDR_FLASH_END ((uint32_t)0x08140000)
#endif /* ADDR_FLASH_END */
#endif /* FLASH_SECTOR_TOTAL */

#if FLASH_SECTOR_TOTAL > 19U
#define ADDR_FLASH_SECTOR_19 \
    ((uint32_t)0x08160000) /* Base address of Sector 19, 128 Kbytes */
#else
#ifndef ADDR_FLASH_END
#define ADDR_FLASH_END ((uint32_t)0x08160000)
#endif /* ADDR_FLASH_END */
#endif /* FLASH_SECTOR_TOTAL */

#if FLASH_SECTOR_TOTAL > 20U
#define ADDR_FLASH_SECTOR_20 \
    ((uint32_t)0x08180000) /* Base address of Sector 20, 128 Kbytes */
#else
#ifndef ADDR_FLASH_END
#define ADDR_FLASH_END ((uint32_t)0x08180000)
#endif /* ADDR_FLASH_END */
#endif /* FLASH_SECTOR_TOTAL */

#if FLASH_SECTOR_TOTAL > 21U
#define ADDR_FLASH_SECTOR_21 \
    ((uint32_t)0x081A0000) /* Base address of Sector 21, 128 Kbytes */
#else
#ifndef ADDR_FLASH_END
#define ADDR_FLASH_END ((uint32_t)0x081A0000)
#endif /* ADDR_FLASH_END */
#endif /* FLASH_SECTOR_TOTAL */

#if FLASH_SECTOR_TOTAL > 22U
#define ADDR_FLASH_SECTOR_22 \
    ((uint32_t)0x081C0000) /* Base address of Sector 22, 128 Kbytes */
#else
#ifndef ADDR_FLASH_END
#define ADDR_FLASH_END ((uint32_t)0x081C0000)
#endif /* ADDR_FLASH_END */
#endif /* FLASH_SECTOR_TOTAL */

#if FLASH_SECTOR_TOTAL > 23U
#define ADDR_FLASH_SECTOR_23 \
    ((uint32_t)0x081E0000) /* Base address of Sector 23, 128 Kbytes */
#else
#ifndef ADDR_FLASH_END
#define ADDR_FLASH_END ((uint32_t)0x081E0000)
#endif /* ADDR_FLASH_END */
#endif /* FLASH_SECTOR_TOTAL */

#ifndef ADDR_FLASH_END
#define ADDR_FLASH_END ((uint32_t)0x08200000)
#endif /* ADDR_FLASH_END */

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

/**
 * @brief        get the sector number of flash
 * @param[in]    addr: flash address
 * @return       uint32_t sector number
*/
extern uint32_t flash_sector(uint32_t addr);

/**
 * @brief        get the next page flash address
 * @param[in]    addr: flash address
 * @return       uin32_t next page flash address
*/
extern uint32_t flash_addr_next(uint32_t addr);

/**
 * @brief        erase flash
 * @param[in]    addr: flash address
 * @param[in]    len: page num
*/
extern void flash_erase(uint32_t addr,
                        uint16_t len);

/**
 * @brief        read data for flash
 * @param[in]    addr: flash address
 * @param[out]   buf:  data point
 * @param[in]    len:  data num
*/
extern void flash_read(uint32_t  addr,
                       uint32_t *buf,
                       uint32_t  len);

/**
 * @brief        write data to one page of flash
 * @param[in]    addr: flash address
 * @param[in]    buf:  data point
 * @param[in]    len:  data num
 * @return       int8_t success 0, failure -1
*/
extern int8_t flash_write(uint32_t  addr,
                          uint32_t *buf,
                          uint32_t  len);

/**
 * @brief        write data to some pages of flash
 * @param[in]    addr:     flash start address
 * @param[in]    addr_end: flash end address
 * @param[in]    buf:      data point
 * @param[in]    len:      data num
 * @return       int8_t success 0, failure -1
*/
extern int8_t flash_writen(uint32_t  addr,
                           uint32_t  addr_end,
                           uint32_t *buf,
                           uint32_t  len);

__END_DECLS

/* Private defines -----------------------------------------------------------*/

/* __BSP_FLASH_H__ -----------------------------------------------------------*/
#endif /* __BSP_FLASH_H__ */

/************************ (C) COPYRIGHT ngu ********************END OF FILE****/
