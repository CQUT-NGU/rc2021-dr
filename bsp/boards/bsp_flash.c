/**
 * *****************************************************************************
 * @file         bsp_flash.c
 * @brief        flash of boards
 * @author       NGU
 * @date         20210427
 * @version      1
 * @copyright    Copyright (C) 2020-2021
 * *****************************************************************************
*/

#include "bsp_flash.h"

#include "main.h"

#include <string.h>

uint32_t flash_sector(uint32_t addr)
{
    uint32_t sector = 0U;

#if FLASH_SECTOR_TOTAL > 1U
    if ((addr < ADDR_FLASH_SECTOR_1) && (addr >= ADDR_FLASH_SECTOR_0))
    {
        sector = FLASH_SECTOR_0;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 2U
    else if ((addr < ADDR_FLASH_SECTOR_2) && (addr >= ADDR_FLASH_SECTOR_1))
    {
        sector = FLASH_SECTOR_1;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 3U
    else if ((addr < ADDR_FLASH_SECTOR_3) && (addr >= ADDR_FLASH_SECTOR_2))
    {
        sector = FLASH_SECTOR_2;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 4U
    else if ((addr < ADDR_FLASH_SECTOR_4) && (addr >= ADDR_FLASH_SECTOR_3))
    {
        sector = FLASH_SECTOR_3;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 5U
    else if ((addr < ADDR_FLASH_SECTOR_5) && (addr >= ADDR_FLASH_SECTOR_4))
    {
        sector = FLASH_SECTOR_4;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 6U
    else if ((addr < ADDR_FLASH_SECTOR_6) && (addr >= ADDR_FLASH_SECTOR_5))
    {
        sector = FLASH_SECTOR_5;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 7U
    else if ((addr < ADDR_FLASH_SECTOR_7) && (addr >= ADDR_FLASH_SECTOR_6))
    {
        sector = FLASH_SECTOR_6;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 8U
    else if ((addr < ADDR_FLASH_SECTOR_8) && (addr >= ADDR_FLASH_SECTOR_7))
    {
        sector = FLASH_SECTOR_7;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 9U
    else if ((addr < ADDR_FLASH_SECTOR_9) && (addr >= ADDR_FLASH_SECTOR_8))
    {
        sector = FLASH_SECTOR_8;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 10U
    else if ((addr < ADDR_FLASH_SECTOR_10) && (addr >= ADDR_FLASH_SECTOR_9))
    {
        sector = FLASH_SECTOR_9;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 11U
    else if ((addr < ADDR_FLASH_SECTOR_11) && (addr >= ADDR_FLASH_SECTOR_10))
    {
        sector = FLASH_SECTOR_10;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 12U
    else if ((addr < ADDR_FLASH_SECTOR_12) && (addr >= ADDR_FLASH_SECTOR_11))
    {
        sector = FLASH_SECTOR_11;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 13U
    else if ((addr < ADDR_FLASH_SECTOR_13) && (addr >= ADDR_FLASH_SECTOR_12))
    {
        sector = FLASH_SECTOR_12;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 14U
    else if ((addr < ADDR_FLASH_SECTOR_14) && (addr >= ADDR_FLASH_SECTOR_13))
    {
        sector = FLASH_SECTOR_13;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 15U
    else if ((addr < ADDR_FLASH_SECTOR_15) && (addr >= ADDR_FLASH_SECTOR_14))
    {
        sector = FLASH_SECTOR_14;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 16U
    else if ((addr < ADDR_FLASH_SECTOR_16) && (addr >= ADDR_FLASH_SECTOR_15))
    {
        sector = FLASH_SECTOR_15;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 17U
    else if ((addr < ADDR_FLASH_SECTOR_17) && (addr >= ADDR_FLASH_SECTOR_16))
    {
        sector = FLASH_SECTOR_16;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 18U
    else if ((addr < ADDR_FLASH_SECTOR_18) && (addr >= ADDR_FLASH_SECTOR_17))
    {
        sector = FLASH_SECTOR_17;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 19U
    else if ((addr < ADDR_FLASH_SECTOR_19) && (addr >= ADDR_FLASH_SECTOR_18))
    {
        sector = FLASH_SECTOR_18;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 20U
    else if ((addr < ADDR_FLASH_SECTOR_20) && (addr >= ADDR_FLASH_SECTOR_19))
    {
        sector = FLASH_SECTOR_19;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 21U
    else if ((addr < ADDR_FLASH_SECTOR_21) && (addr >= ADDR_FLASH_SECTOR_20))
    {
        sector = FLASH_SECTOR_20;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 22U
    else if ((addr < ADDR_FLASH_SECTOR_22) && (addr >= ADDR_FLASH_SECTOR_21))
    {
        sector = FLASH_SECTOR_21;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 23U
    else if ((addr < ADDR_FLASH_SECTOR_23) && (addr >= ADDR_FLASH_SECTOR_22))
    {
        sector = FLASH_SECTOR_22;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 1U
    else
    {
        sector = FLASH_SECTOR_TOTAL - 1U;
    }
#endif /* FLASH_SECTOR_TOTAL */

    return sector;
}

uint32_t flash_addr_next(uint32_t addr)
{
    uint32_t sector = 0U;

#if FLASH_SECTOR_TOTAL > 1U
    if ((addr < ADDR_FLASH_SECTOR_1) && (addr >= ADDR_FLASH_SECTOR_0))
    {
        sector = ADDR_FLASH_SECTOR_1;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 2U
    else if ((addr < ADDR_FLASH_SECTOR_2) && (addr >= ADDR_FLASH_SECTOR_1))
    {
        sector = ADDR_FLASH_SECTOR_2;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 3U
    else if ((addr < ADDR_FLASH_SECTOR_3) && (addr >= ADDR_FLASH_SECTOR_2))
    {
        sector = ADDR_FLASH_SECTOR_3;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 4U
    else if ((addr < ADDR_FLASH_SECTOR_4) && (addr >= ADDR_FLASH_SECTOR_3))
    {
        sector = ADDR_FLASH_SECTOR_4;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 5U
    else if ((addr < ADDR_FLASH_SECTOR_5) && (addr >= ADDR_FLASH_SECTOR_4))
    {
        sector = ADDR_FLASH_SECTOR_5;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 6U
    else if ((addr < ADDR_FLASH_SECTOR_6) && (addr >= ADDR_FLASH_SECTOR_5))
    {
        sector = ADDR_FLASH_SECTOR_6;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 7U
    else if ((addr < ADDR_FLASH_SECTOR_7) && (addr >= ADDR_FLASH_SECTOR_6))
    {
        sector = ADDR_FLASH_SECTOR_7;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 8U
    else if ((addr < ADDR_FLASH_SECTOR_8) && (addr >= ADDR_FLASH_SECTOR_7))
    {
        sector = ADDR_FLASH_SECTOR_8;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 9U
    else if ((addr < ADDR_FLASH_SECTOR_9) && (addr >= ADDR_FLASH_SECTOR_8))
    {
        sector = ADDR_FLASH_SECTOR_9;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 10U
    else if ((addr < ADDR_FLASH_SECTOR_10) && (addr >= ADDR_FLASH_SECTOR_9))
    {
        sector = ADDR_FLASH_SECTOR_10;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 11U
    else if ((addr < ADDR_FLASH_SECTOR_11) && (addr >= ADDR_FLASH_SECTOR_10))
    {
        sector = ADDR_FLASH_SECTOR_11;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 12U
    else if ((addr < ADDR_FLASH_SECTOR_12) && (addr >= ADDR_FLASH_SECTOR_11))
    {
        sector = ADDR_FLASH_SECTOR_12;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 13U
    else if ((addr < ADDR_FLASH_SECTOR_13) && (addr >= ADDR_FLASH_SECTOR_12))
    {
        sector = ADDR_FLASH_SECTOR_13;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 14U
    else if ((addr < ADDR_FLASH_SECTOR_14) && (addr >= ADDR_FLASH_SECTOR_13))
    {
        sector = ADDR_FLASH_SECTOR_14;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 15U
    else if ((addr < ADDR_FLASH_SECTOR_15) && (addr >= ADDR_FLASH_SECTOR_14))
    {
        sector = ADDR_FLASH_SECTOR_15;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 16U
    else if ((addr < ADDR_FLASH_SECTOR_16) && (addr >= ADDR_FLASH_SECTOR_15))
    {
        sector = ADDR_FLASH_SECTOR_16;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 17U
    else if ((addr < ADDR_FLASH_SECTOR_17) && (addr >= ADDR_FLASH_SECTOR_16))
    {
        sector = ADDR_FLASH_SECTOR_17;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 18U
    else if ((addr < ADDR_FLASH_SECTOR_18) && (addr >= ADDR_FLASH_SECTOR_17))
    {
        sector = ADDR_FLASH_SECTOR_18;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 19U
    else if ((addr < ADDR_FLASH_SECTOR_19) && (addr >= ADDR_FLASH_SECTOR_18))
    {
        sector = ADDR_FLASH_SECTOR_19;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 20U
    else if ((addr < ADDR_FLASH_SECTOR_20) && (addr >= ADDR_FLASH_SECTOR_19))
    {
        sector = ADDR_FLASH_SECTOR_20;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 21U
    else if ((addr < ADDR_FLASH_SECTOR_21) && (addr >= ADDR_FLASH_SECTOR_20))
    {
        sector = ADDR_FLASH_SECTOR_21;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 22U
    else if ((addr < ADDR_FLASH_SECTOR_22) && (addr >= ADDR_FLASH_SECTOR_21))
    {
        sector = ADDR_FLASH_SECTOR_22;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 12U
    else if ((addr < ADDR_FLASH_SECTOR_23) && (addr >= ADDR_FLASH_SECTOR_22))
    {
        sector = ADDR_FLASH_SECTOR_23;
    }
#endif /* FLASH_SECTOR_TOTAL */
#if FLASH_SECTOR_TOTAL > 1U
    else
    {
        sector = ADDR_FLASH_END;
    }
#endif /* FLASH_SECTOR_TOTAL */

    return sector;
}

void flash_erase(uint32_t addr,
                 uint16_t len)
{
    FLASH_EraseInitTypeDef flash_erase = {0};

    uint32_t error;

    /*!< Initial FLASH sector to erase when Mass erase is disabled */
    flash_erase.Sector = flash_sector(addr);
    /*!< Mass erase or sector Erase */
    flash_erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    /*!< The device voltage range which defines the erase parallelism */
    flash_erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    /*!< Number of sectors to be erased */
    flash_erase.NbSectors = len;

    /*!< Unlock the FLASH control register access */
    (void)HAL_FLASH_Unlock();
    /*!< Perform a mass erase or erase the specified FLASH memory sectors */
    (void)HAL_FLASHEx_Erase(&flash_erase, &error);
    /*!< Locks the FLASH control register access */
    (void)HAL_FLASH_Lock();
}

void flash_read(uint32_t addr,
                uint32_t *buf,
                uint32_t len)
{
    (void)memcpy(buf, (void *)addr, len * sizeof(len));
}

int8_t flash_write(uint32_t addr, uint32_t *buf, uint32_t len)
{
    int8_t ret = 0;

    uint32_t n = 0;

    uint32_t addr_end = flash_addr_next(addr);

    /*!< Unlock the FLASH control register access */
    (void)HAL_FLASH_Unlock();

    while (addr <= addr_end)
    {
        /*!< Program word word at a specified address */
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, *buf) == HAL_OK)
        {
            addr += sizeof(addr);
            buf++;
            n++;
            if (n == len)
            {
                break;
            }
        }
        else
        {
            ret = -1;
            break;
        }
    }

    /*!< Locks the FLASH control register access */
    (void)HAL_FLASH_Lock();

    return ret;
}

int8_t flash_writen(uint32_t addr,
                    uint32_t addr_end,
                    uint32_t *buf,
                    uint32_t len)
{
    int8_t ret = 0;

    uint32_t n = 0U;

    HAL_FLASH_Unlock();

    /*!< Unlock the FLASH control register access */
    (void)HAL_FLASH_Unlock();

    while (addr <= addr_end)
    {
        /*!< Program word word at a specified address */
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, *buf) == HAL_OK)
        {
            addr += sizeof(addr);
            buf++;
            n++;
            if (n == len)
            {
                break;
            }
        }
        else
        {
            ret = -1;
            break;
        }
    }

    /*!< Locks the FLASH control register access */
    (void)HAL_FLASH_Lock();

    return ret;
}

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
