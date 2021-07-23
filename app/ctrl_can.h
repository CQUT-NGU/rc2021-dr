/**
 * *****************************************************************************
 * @file         ctrl_can.h
 * @brief        can control
 * @author       NGU
 * @date         20210427
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
 * @details      there is CAN interrupt function to receive motor data, and CAN
 *               send function to send motor current to control motor.
 * *****************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CTRL_CAN_H__
#define __CTRL_CAN_H__

#include "ctrl.h"

#define CHASSIS_CAN hcan1

/**
 * @enum         can_msg_id_enum
 * @brief        CAN send and receive ID
*/
typedef enum
{
    CAN_ID_OTHER_ALL = 0x1FF,
    CAN_ID_CHASSIS_ALL = 0x200,
    CAN_ID_3508_M1 = 0x201,
    CAN_ID_3508_M2 = 0x202,
    CAN_ID_3508_M3 = 0x203,
    CAN_ID_3508_M4 = 0x204,
    CAN_ID_3508_M5 = 0x205,
    CAN_ID_3508_M6 = 0x206,
    CAN_ID_3508_M7 = 0x207,
    CAN_ID_3508_M8 = 0x208,
    CAN_ID_3508_RESET = 0x700,
} can_msg_id_e;

/**
 * @struct       motor_t
 * @brief        motor data
*/
typedef struct
{
    uint16_t ecd_last;
    uint16_t ecd;
    int16_t v_rpm;
    int16_t i_current;
    uint8_t temperate;
} motor_t;

/* 3508 motor ecd */
#define ECD_RANGE_HALF 4096
#define ECD_RANGE      8192

/* 3508 max motor control current */
#define MAX_MOTOR_CAN_CURRENT 0x4000

#define MOTOR_ECD_TO_RAD 0.0007669903939428206F  //!< 2 * PI / 8192
#define MOTOR_ROLLER_RAD (MOTOR_ECD_TO_RAD / 19)

__BEGIN_DECLS

/**
 * @brief        return the chassis 3508 motor data point
 * @param[in]    i: motor number,range [0,7]
 * @return       motor data point
*/
extern const motor_t *chassis_point(int i);
extern const motor_t *defense_point(void);

/**
 * @brief        send control current of motor (0x201, 0x202, 0x203, 0x204)
 * @param[in]    mo1: (0x201) 3508 motor control current, range[-16384,16384]
 * @param[in]    mo2: (0x202) 3508 motor control current, range[-16384,16384]
 * @param[in]    mo3: (0x203) 3508 motor control current, range[-16384,16384]
 * @param[in]    mo4: (0x204) 3508 motor control current, range[-16384,16384]
*/
extern void chassis_ctrl(int16_t mo1,
                         int16_t mo2,
                         int16_t mo3,
                         int16_t mo4);

/**
 * @brief        send control current of motor (0x205, 0x206, 0x207, 0x208)
 * @param[in]    mo1: (0x205) 3508 motor control current, range[-16384,16384]
 * @param[in]    mo2: (0x206) 3508 motor control current, range[-16384,16384]
 * @param[in]    mo3: (0x207) 3508 motor control current, range[-16384,16384]
 * @param[in]    mo4: (0x208) 3508 motor control current, range[-16384,16384]
*/
extern void other_ctrl(int16_t mo1,
                       int16_t mo2,
                       int16_t mo3,
                       int16_t mo4);

/**
 * @brief        send CAN packet of ID 0x700,
 *               it will set chassis motor 3508 to quick ID setting
*/
extern void chassis_reset(void);

__END_DECLS

/* Enddef to prevent recursive inclusion ------------------------------------ */
#endif /* __CTRL_CAN_H__ */

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
