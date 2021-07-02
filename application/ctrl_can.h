/**
 * *****************************************************************************
 * @file         ctrl_can.c/h
 * @brief        can control
 * @author       ngu
 * @date         20210427
 * @version      1
 * @copyright    Copyright (c) 2021
 * @code         utf-8                                                  @endcode
 * @details      there is CAN interrupt function to receive motor data, and CAN
 *               send function to send motor current to control motor.
 * *****************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CTRL_CAN_H__
#define __CTRL_CAN_H__

/* Includes ------------------------------------------------------------------*/
/* Private includes ----------------------------------------------------------*/
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/

#define CHASSIS_CAN hcan1
#define OTHER_CAN   hcan2

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

/**
 * @enum         can_msg_id_enum
 * @brief        CAN send and receive ID
*/
typedef enum
{
    CAN_ID_OTHER_ALL   = 0x1FFU,
    CAN_ID_CHASSIS_ALL = 0x200U,
    CAN_ID_3508_M1     = 0x201U,
    CAN_ID_3508_M2     = 0x202U,
    CAN_ID_3508_M3     = 0x203U,
    CAN_ID_3508_M4     = 0x204U,
    CAN_ID_3508_SHOOT  = 0x205U,
} can_msg_id_e;

/**
 * @struct       motor_t
 * @brief        motor data
*/
typedef struct
{
    uint16_t ecd_last;
    uint16_t ecd;
    int16_t  v_rpm;
    int16_t  i_current;
    uint8_t  temperate;
} motor_t;

#define ECD_RANGE_HALF 4096
#define ECD_RANGE      8191

#define MOTOR_ECD_TO_RAD 0.0007669903939428206F  //!< 2 * PI / 8192
#define MOTOR_ROLLER_RAD (MOTOR_ECD_TO_RAD / 19)

/* Exported functions prototypes ---------------------------------------------*/

__BEGIN_DECLS

/**
 * @brief        return the chassis 3508 motor data point
 * @param[in]    i: motor number,range [0,3]
 * @return       motor data point
*/
extern const motor_t *chassis_point(uint8_t i);

/**
 * @brief        send control current of motor (0x201, 0x202, 0x203, 0x204)
 * @param[in]    motor1: (0x201) 3508 motor control current, range[-16384,16384]
 * @param[in]    motor2: (0x202) 3508 motor control current, range[-16384,16384]
 * @param[in]    motor3: (0x203) 3508 motor control current, range[-16384,16384]
 * @param[in]    motor4: (0x204) 3508 motor control current, range[-16384,16384]
*/
extern void chassis_ctrl(int16_t motor1,
                         int16_t motor2,
                         int16_t motor3,
                         int16_t motor4);

/**
 * @brief        send control current of motor (0x205, 0x206, 0x207, 0x208)
 * @param[in]    motor1: (0x205) 3508 motor control current, range[-16384,16384]
 * @param[in]    motor2: (0x206) 3508 motor control current, range[-16384,16384]
 * @param[in]    motor3: (0x207) 3508 motor control current, range[-16384,16384]
 * @param[in]    motor4: (0x208) 3508 motor control current, range[-16384,16384]
*/
extern void other_ctrl(int16_t motor1,
                       int16_t motor2,
                       int16_t motor3,
                       int16_t motor4);

/**
 * @brief        send CAN packet of ID 0x700,
 *               it will set chassis motor 3508 to quick ID setting
*/
extern void chassis_reset(void);

extern void shoot_angle_reset(void);

__END_DECLS

/* Private defines -----------------------------------------------------------*/

/* __CTRL_CAN_H__ ------------------------------------------------------------*/
#endif /* __CTRL_CAN_H__ */

/************************ (C) COPYRIGHT ngu ********************END OF FILE****/
