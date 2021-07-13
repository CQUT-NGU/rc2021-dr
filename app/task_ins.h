/**
 * *****************************************************************************
 * @file         task_ins.h
 * @brief        use bmi088 to calculate the euler angle. no use ist8310, so only
 *               enable data ready pin to save cpu time.enalbe bmi088 data ready
 *               enable spi DMA to save the time spi transmit
 * @author       NGU
 * @date         20210101
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
 * *****************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_INS_H__
#define __APP_INS_H__

#include "bsp.h"

__BEGIN_DECLS

extern void task_ins(void const *pvParameters);

__END_DECLS

/* Enddef to prevent recursive inclusion ------------------------------------ */
#endif /* __APP_INS_H__ */

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
