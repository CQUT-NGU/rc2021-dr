/**
 * *****************************************************************************
 * @file         bmi088.h
 * @brief        The driver of bmi088
 * @author       NGU
 * @date         20210427
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
 * *****************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BMI088_H__
#define __BMI088_H__

#include "bmi088reg.h"

#include <stdint.h>

#define CS1_ACCEL_PIN  GPIO_PIN_4
#define CS1_ACCEL_PORT GPIOA
#define CS1_GYRO_PIN   GPIO_PIN_0
#define CS1_GYRO_PORT  GPIOB

#define INT1_ACCEL_PIN  GPIO_PIN_4
#define INT1_ACCEL_PORT GPIOC
#define INT1_ACCEL_EXTI EXTI4_IRQn
#define INT1_GYRO_PIN   GPIO_PIN_5
#define INT1_GYRO_PORT  GPIOC
#define INT1_GYRO_EXTI  EXTI9_5_IRQn

#undef __packed
#define __packed __attribute__((__packed__))

#define BMI088_ACCEL_NS_L (CS1_ACCEL_PORT->BSRR = (uint32_t)CS1_ACCEL_PIN << 16)
#define BMI088_ACCEL_NS_H (CS1_ACCEL_PORT->BSRR = CS1_ACCEL_PIN)
#define BMI088_GYRO_NS_L  (CS1_GYRO_PORT->BSRR = (uint32_t)CS1_GYRO_PIN << 16)
#define BMI088_GYRO_NS_H  (CS1_GYRO_PORT->BSRR = CS1_GYRO_PIN)

typedef struct
{
    uint8_t status;
    int16_t accel[3];
    int16_t temp;
    int16_t gyro[3];
} __packed bmi088_raw_data_t;

typedef struct
{
    uint8_t status;
    float accel[3];
    float temp;
    float gyro[3];
    float time;
} bmi_t;

enum
{
    BMI088_NO_ERROR = 0x00,
    BMI088_ACC_PWR_CTRL_ERROR = 0x01,
    BMI088_ACC_PWR_CONF_ERROR = 0x02,
    BMI088_ACC_CONF_ERROR = 0x03,
    BMI088_ACC_SELF_TEST_ERROR = 0x04,
    BMI088_ACC_RANGE_ERROR = 0x05,
    BMI088_INT1_IO_CTRL_ERROR = 0x06,
    BMI088_INT_MAP_DATA_ERROR = 0x07,
    BMI088_GYRO_RANGE_ERROR = 0x08,
    BMI088_GYRO_BANDWIDTH_ERROR = 0x09,
    BMI088_GYRO_LPM1_ERROR = 0x0A,
    BMI088_GYRO_CTRL_ERROR = 0x0B,
    BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
    BMI088_GYRO_INT3_INT4_IO_MAP_ERROR = 0x0D,

    BMI088_SELF_TEST_ACCEL_ERROR = 0x80,
    BMI088_SELF_TEST_GYRO_ERROR = 0x40,
    BMI088_NO_SENSOR = 0xFF,
};

__BEGIN_DECLS

extern uint8_t bmi088_init(void);
extern uint8_t bmi088_accel_init(void);
extern uint8_t bmi088_gyro_init(void);

extern void bmi088_read_over_gyro(uint8_t *buf,
                                  float gyro[3]);
extern void bmi088_read_over_accel(uint8_t *buf,
                                   float accel[3],
                                   float *time);
extern void bmi088_read_over_temp(uint8_t *buf,
                                  float *temperate);

extern void bmi088_read(float gyro[3],
                        float accel[3],
                        float *temperate);
extern void bmi088_gyro(float gyro[3]);
extern void bmi088_accel(float accel[3]);

extern float bmi088_temperate(void);

extern uint32_t bmi088_sensor_time(void);

__END_DECLS

/* Enddef to prevent recursive inclusion ------------------------------------ */
#endif /* __BMI088_H__ */

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
