/**
 * *****************************************************************************
 * @file         ctrl_rc.c/h
 * @brief        control by remote
 * @author       ngu
 * @date         20210427
 * @version      1
 * @copyright    Copyright (c) 2021
 * @code         utf-8                                                  @endcode
 * *****************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CTRL_RC_H__
#define __CTRL_RC_H__

/* Includes ------------------------------------------------------------------*/
/* Private includes ----------------------------------------------------------*/
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/

#define SBUS_RX_BUF_NUM 36U
#define RC_FRAME_LENGTH 18U

#define RC_CH_VALUE_MIN    ((uint16_t)364U)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024U)
#define RC_CH_VALUE_MAX    ((uint16_t)1684U)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP          ((uint16_t)1U)
#define RC_SW_MID         ((uint16_t)3U)
#define RC_SW_DOWN        ((uint16_t)2U)
#define switch_is_down(s) ((s) == RC_SW_DOWN)
#define switch_is_mid(s)  ((s) == RC_SW_MID)
#define switch_is_up(s)   ((s) == RC_SW_UP)

/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W     ((uint16_t)1U << 0)
#define KEY_PRESSED_OFFSET_S     ((uint16_t)1U << 1)
#define KEY_PRESSED_OFFSET_A     ((uint16_t)1U << 2)
#define KEY_PRESSED_OFFSET_D     ((uint16_t)1U << 3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1U << 4)
#define KEY_PRESSED_OFFSET_CTRL  ((uint16_t)1U << 5)
#define KEY_PRESSED_OFFSET_Q     ((uint16_t)1U << 6)
#define KEY_PRESSED_OFFSET_E     ((uint16_t)1U << 7)
#define KEY_PRESSED_OFFSET_R     ((uint16_t)1U << 8)
#define KEY_PRESSED_OFFSET_F     ((uint16_t)1U << 9)
#define KEY_PRESSED_OFFSET_G     ((uint16_t)1U << 10)
#define KEY_PRESSED_OFFSET_Z     ((uint16_t)1U << 11)
#define KEY_PRESSED_OFFSET_X     ((uint16_t)1U << 12)
#define KEY_PRESSED_OFFSET_C     ((uint16_t)1U << 13)
#define KEY_PRESSED_OFFSET_V     ((uint16_t)1U << 14)
#define KEY_PRESSED_OFFSET_B     ((uint16_t)1U << 15)

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

#undef __packed
#define __packed __attribute__((__packed__))

/* Exported types ------------------------------------------------------------*/

/**
 * @struct       ctrl_rc_t
 * @brief        remote control data
*/
typedef struct
{
    struct
    {
        int16_t ch[5];
        char    s[2];
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

/* Exported functions prototypes ---------------------------------------------*/

__BEGIN_DECLS

/**
 * @brief        Remote control init
*/
extern void ctrl_rc_init(void);

/**
 * @brief        Get remote control data point
 * @return       ctrl_rc_t remote control data point
*/
extern const ctrl_rc_t *ctrl_rc_point(void);

__END_DECLS

/* Private defines -----------------------------------------------------------*/

/* __CTRL_RC_H__ -------------------------------------------------------------*/
#endif /* __CTRL_RC_H__ */

/************************ (C) COPYRIGHT ngu ********************END OF FILE****/
