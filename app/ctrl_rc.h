/**
 * *****************************************************************************
 * @file         ctrl_rc.h
 * @brief        control by remote
 * @author       NGU
 * @date         20210427
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
 * *****************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CTRL_RC_H__
#define __CTRL_RC_H__

#include "ctrl.h"

#define SBUS_RX_BUF_NUM 36
#define RC_FRAME_LENGTH 18

#define RC_CH_VALUE_MIN    364
#define RC_CH_VALUE_OFFSET 1024
#define RC_CH_VALUE_MAX    1684

#define RC_ROCKER_MAX (RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET)
#define RC_ROCKER_MIN (RC_CH_VALUE_MIN - RC_CH_VALUE_OFFSET)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_R 0 /* switch right */
#define RC_SW_L 1 /* switch left  */

#define RC_CH_RH 0 /* channel rigth horizontal */
#define RC_CH_RV 1 /* channel right vertical   */
#define RC_CH_LH 2 /* channel left horizontal  */
#define RC_CH_LV 3 /* channel left vertical    */
#define RC_CH_S  4 /* channel rotation         */

#define RC_SW_UP   1
#define RC_SW_MID  3
#define RC_SW_DOWN 2

#define switch_is_down(s) ((s) == RC_SW_DOWN)
#define switch_is_mid(s)  ((s) == RC_SW_MID)
#define switch_is_up(s)   ((s) == RC_SW_UP)

/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W     (1 << 0)
#define KEY_PRESSED_OFFSET_S     (1 << 1)
#define KEY_PRESSED_OFFSET_A     (1 << 2)
#define KEY_PRESSED_OFFSET_D     (1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT (1 << 4)
#define KEY_PRESSED_OFFSET_CTRL  (1 << 5)
#define KEY_PRESSED_OFFSET_Q     (1 << 6)
#define KEY_PRESSED_OFFSET_E     (1 << 7)
#define KEY_PRESSED_OFFSET_R     (1 << 8)
#define KEY_PRESSED_OFFSET_F     (1 << 9)
#define KEY_PRESSED_OFFSET_G     (1 << 10)
#define KEY_PRESSED_OFFSET_Z     (1 << 11)
#define KEY_PRESSED_OFFSET_X     (1 << 12)
#define KEY_PRESSED_OFFSET_C     (1 << 13)
#define KEY_PRESSED_OFFSET_V     (1 << 14)
#define KEY_PRESSED_OFFSET_B     (1 << 15)

/**
 * @struct       ctrl_rc_t
 * @brief        remote control data
*/
typedef struct
{
    struct
    {
        int16_t ch[5];
        char s[2];
    } __packed rc;

    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } __packed mouse;

    struct
    {
        uint16_t v;
    } __packed key;

} __packed ctrl_rc_t;

__BEGIN_DECLS

/**
 * @brief        Remote control init
*/
extern void ctrl_rc_init(void);

/**
 * @brief        Get remote control data point
 * @return       ctrl_rc_t remote control data point
*/
extern ctrl_rc_t *ctrl_rc_point(void);

__END_DECLS

/* Enddef to prevent recursive inclusion ------------------------------------ */
#endif /* __CTRL_RC_H__ */

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
