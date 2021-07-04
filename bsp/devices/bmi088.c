/**
 * *****************************************************************************
 * @file         bmi088.c
 * @brief        The driver of bmi088
 * @author       NGU
 * @date         20210427
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
 * @details      CS1_ACCEL      PA4
 *               CS1_GYRO       PB0
 *               INT1_ACCEL     PC4
 *               INT1_GYRO      PC5
 * *****************************************************************************
*/

#include "bmi088.h"

#include "bsp.h"
#include "main.h"

#undef hspi
#define hspi hspi1

#define BMI088_TEMP_FACTOR 0.125F
#define BMI088_TEMP_OFFSET 23.0F

#define BMI088_WRITE_ACCEL_REG_NUM 6
#define BMI088_WRITE_GYRO_REG_NUM  6

#define BMI088_GYRO_DATA_READY_BIT       0
#define BMI088_ACCEL_DATA_READY_BIT      1
#define BMI088_ACCEL_TEMP_DATA_READY_BIT 2

#define BMI088_LONG_DELAY_TIME      80
#define BMI088_COM_WAIT_SENSOR_TIME 150

#define BMI088_ACCEL_IIC_ADDRESSE (0x18 << 1)
#define BMI088_GYRO_IIC_ADDRESSE  (0x68 << 1)

#define BMI088_ACCEL_RANGE_3G
//#define BMI088_ACCEL_RANGE_6G
//#define BMI088_ACCEL_RANGE_12G
//#define BMI088_ACCEL_RANGE_24G

#define BMI088_GYRO_RANGE_2000
//#define BMI088_GYRO_RANGE_1000
//#define BMI088_GYRO_RANGE_500
//#define BMI088_GYRO_RANGE_250
//#define BMI088_GYRO_RANGE_125

#define BMI088_ACCEL_3G_SEN  0.0008974358974F
#define BMI088_ACCEL_6G_SEN  0.00179443359375F
#define BMI088_ACCEL_12G_SEN 0.0035888671875F
#define BMI088_ACCEL_24G_SEN 0.007177734375F
#define BMI088_ACCEL_SEN     BMI088_ACCEL_3G_SEN

#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381F
#define BMI088_GYRO_1000_SEN 0.00053263221801584764920766930190693F
#define BMI088_GYRO_500_SEN  0.00026631610900792382460383465095346F
#define BMI088_GYRO_250_SEN  0.00013315805450396191230191732547673F
#define BMI088_GYRO_125_SEN  0.000066579027251980956150958662738366F
#define BMI088_GYRO_SEN      BMI088_GYRO_2000_SEN

#define BMI088_USE_SPI
#undef BMI088_USE_IIC

extern SPI_HandleTypeDef hspi;

#define bmi_accel_w(reg, data)   \
    do                           \
    {                            \
        BMI088_ACCEL_NS_L;       \
        (void)bmi_byte_rw(reg);  \
        (void)bmi_byte_rw(data); \
        BMI088_ACCEL_NS_H;       \
    } while (0)

#define bmi_accel_r(reg, data)           \
    do                                   \
    {                                    \
        BMI088_ACCEL_NS_L;               \
        (void)bmi_byte_rw((reg) | 0x80); \
        (void)bmi_byte_rw(0x55);         \
        (data) = bmi_byte_rw(0x55);      \
        BMI088_ACCEL_NS_H;               \
    } while (0)

#define bmi_accel_muli_w(reg, data, len)      \
    do                                        \
    {                                         \
        BMI088_ACCEL_NS_L;                    \
        bmi_reg_muli_w((reg), (data), (len)); \
        BMI088_ACCEL_NS_H;                    \
    } while (0)

#define bmi_accel_muli_r(reg, data, len)      \
    {                                         \
        BMI088_ACCEL_NS_L;                    \
        (void)bmi_byte_rw((reg) | 0x80);      \
        bmi_reg_muli_r((reg), (data), (len)); \
        BMI088_ACCEL_NS_H;                    \
    }

#define bmi_gyro_w(reg, data)    \
    {                            \
        BMI088_GYRO_NS_L;        \
        (void)bmi_byte_rw(reg);  \
        (void)bmi_byte_rw(data); \
        BMI088_GYRO_NS_H;        \
    }

#define bmi_gyro_r(reg, data)            \
    {                                    \
        BMI088_GYRO_NS_L;                \
        (void)bmi_byte_rw((reg) | 0x80); \
        (data) = bmi_byte_rw(0x55);      \
        BMI088_GYRO_NS_H;                \
    }

#define bmi_gyro_muli_w(reg, data, len)       \
    {                                         \
        BMI088_GYRO_NS_L;                     \
        bmi_reg_muli_w((reg), (data), (len)); \
        BMI088_GYRO_NS_H;                     \
    }

#define bmi_gyro_muli_r(reg, data, len)       \
    {                                         \
        BMI088_GYRO_NS_L;                     \
        bmi_reg_muli_r((reg), (data), (len)); \
        BMI088_GYRO_NS_H;                     \
    }

static const uint8_t bmi088_accel_error[BMI088_WRITE_ACCEL_REG_NUM][3] = {
    {
        BMI088_ACC_PWR_CTRL,
        BMI088_ACC_ENABLE_ACC_ON,
        BMI088_ACC_PWR_CTRL_ERROR,
    },
    {
        BMI088_ACC_PWR_CONF,
        BMI088_ACC_PWR_ACTIVE_MODE,
        BMI088_ACC_PWR_CONF_ERROR,
    },
    {
        BMI088_ACC_CONF,
        BMI088_ACC_NORMAL |
            BMI088_ACC_800_HZ |
            BMI088_ACC_CONF_MUST_Set,
        BMI088_ACC_CONF_ERROR,
    },
    {
        BMI088_ACC_RANGE,
        BMI088_ACC_RANGE_3G,
        BMI088_ACC_RANGE_ERROR,
    },
    {
        BMI088_INT1_IO_CTRL,
        BMI088_ACC_INT1_IO_ENABLE |
            BMI088_ACC_INT1_GPIO_PP |
            BMI088_ACC_INT1_GPIO_LOW,
        BMI088_INT1_IO_CTRL_ERROR,
    },
    {
        BMI088_INT_MAP_DATA,
        BMI088_ACC_INT1_DRDY_INTERRUPT,
        BMI088_INT_MAP_DATA_ERROR,
    },
};

static const uint8_t bmi088_gyro_error[BMI088_WRITE_GYRO_REG_NUM][3] = {
    {
        BMI088_GYRO_RANGE,
        BMI088_GYRO_2000,
        BMI088_GYRO_RANGE_ERROR,
    },
    {
        BMI088_GYRO_BANDWIDTH,
        BMI088_GYRO_1000_116_HZ |
            BMI088_GYRO_BANDWIDTH_MUST_Set,
        BMI088_GYRO_BANDWIDTH_ERROR,
    },
    {
        BMI088_GYRO_LPM1,
        BMI088_GYRO_NORMAL_MODE,
        BMI088_GYRO_LPM1_ERROR,
    },
    {
        BMI088_GYRO_CTRL,
        BMI088_DRDY_ON,
        BMI088_GYRO_CTRL_ERROR,
    },
    {
        BMI088_GYRO_INT3_INT4_IO_CONF,
        BMI088_GYRO_INT3_GPIO_PP |
            BMI088_GYRO_INT3_GPIO_LOW,
        BMI088_GYRO_INT3_INT4_IO_CONF_ERROR,
    },
    {
        BMI088_GYRO_INT3_INT4_IO_MAP,
        BMI088_GYRO_DRDY_IO_INT3,
        BMI088_GYRO_INT3_INT4_IO_MAP_ERROR,
    },
};

static uint8_t bmi_byte_rw(uint8_t data)
{
    uint8_t ret;

    HAL_SPI_TransmitReceive(&hspi, &data, &ret, 1, 1000);

    return ret;
}

#if 0
static void bmi_reg_w(uint8_t reg, uint8_t data)
{
    (void)bmi_byte_rw(reg);
    (void)bmi_byte_rw(data);
}

static uint8_t bmi_reg_r(uint8_t reg)
{
    (void)bmi_byte_rw(reg | 0x80);
    return bmi_byte_rw(0x55);
}

static void bmi_reg_muli_w(uint8_t reg, uint8_t *buf, uint8_t len)
{
    (void)bmi_byte_rw(reg);

    while (len-- != 0)
    {
        (void)bmi_byte_rw(*buf++);
    }
}
#endif

static void bmi_reg_muli_r(uint8_t reg, uint8_t *buf, uint8_t len)
{
    (void)bmi_byte_rw(reg | 0x80);

    while (len-- != 0)
    {
        *buf++ = bmi_byte_rw(0x55);
    }
}

uint8_t bmi088_accel_init(void)
{
    uint8_t res = 0;
    uint8_t num = 0;

    /* check commiunication */
    bmi_accel_r(BMI088_ACC_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    bmi_accel_r(BMI088_ACC_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* accel software reset */
    bmi_accel_w(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    delay_ms(BMI088_LONG_DELAY_TIME);

    /* check commiunication is normal after reset */
    bmi_accel_r(BMI088_ACC_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    bmi_accel_r(BMI088_ACC_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* check the "who am I" */
    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    /* set accel sonsor config and check */
    for (num = 0; num != BMI088_WRITE_ACCEL_REG_NUM; ++num)
    {
        bmi_accel_w(bmi088_accel_error[num][0], bmi088_accel_error[num][1]);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        bmi_accel_r(bmi088_accel_error[num][0], res);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != bmi088_accel_error[num][1])
        {
            return bmi088_accel_error[num][2];
        }
    }

    return BMI088_NO_ERROR;
}

uint8_t bmi088_gyro_init(void)
{
    uint8_t res = 0;
    uint8_t num = 0;

    /* check commiunication */
    bmi_gyro_r(BMI088_GYRO_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    bmi_gyro_r(BMI088_GYRO_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* reset the gyro sensor */
    bmi_gyro_w(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    delay_ms(BMI088_LONG_DELAY_TIME);

    /* check commiunication is normal after reset */
    bmi_gyro_r(BMI088_GYRO_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    bmi_gyro_r(BMI088_GYRO_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* check the "who am I" */
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    /* set gyro sonsor config and check */
    for (num = 0; num != BMI088_WRITE_GYRO_REG_NUM; ++num)
    {
        bmi_gyro_w(bmi088_gyro_error[num][0], bmi088_gyro_error[num][1]);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        bmi_gyro_r(bmi088_gyro_error[num][0], res);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != bmi088_gyro_error[num][1])
        {
            return bmi088_gyro_error[num][2];
        }
    }

    return BMI088_NO_ERROR;
}

uint8_t bmi088_init(void)
{
    uint8_t error = BMI088_NO_ERROR;

    error |= bmi088_accel_init();
    error |= bmi088_gyro_init();

    return error;
}

void bmi088_read_over_temp(uint8_t *buf, float *temperate)
{
    int16_t temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (temp > 1023)
    {
        temp = (int16_t)(temp - 2048);
    }

    *temperate = BMI088_TEMP_FACTOR * temp + BMI088_TEMP_OFFSET;
}

void bmi088_read_over_accel(uint8_t *buf, float accel[3], float *time)
{
    int16_t temp;
    uint32_t sensor;

    temp = (int16_t)((buf[1] << 8) | buf[0]);
    accel[0] = BMI088_ACCEL_SEN * temp;
    temp = (int16_t)((buf[3] << 8) | buf[2]);
    accel[1] = BMI088_ACCEL_SEN * temp;
    temp = (int16_t)((buf[5] << 8) | buf[4]);
    accel[2] = BMI088_ACCEL_SEN * temp;

    sensor = (uint32_t)((buf[8] << 16) | (buf[7] << 8) | buf[6]);
    *time = (float)sensor * 39.0625F;
}

void bmi088_read_over_gyro(uint8_t *buf, float gyro[3])
{
    int16_t temp;

    temp = (int16_t)((buf[1] << 8) | buf[0]);
    gyro[0] = BMI088_GYRO_SEN * temp;
    temp = (int16_t)((buf[3] << 8) | buf[2]);
    gyro[1] = BMI088_GYRO_SEN * temp;
    temp = (int16_t)((buf[5] << 8) | buf[4]);
    gyro[2] = BMI088_GYRO_SEN * temp;
}

void bmi088_read(float gyro[3], float accel[3], float *temperate)
{
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    int16_t temp;

    bmi_accel_muli_r(BMI088_ACCEL_XOUT_L, buf, 6);

    temp = (int16_t)((buf[1] << 8) | buf[0]);
    accel[0] = BMI088_ACCEL_SEN * temp;
    temp = (int16_t)((buf[3] << 8) | buf[2]);
    accel[1] = BMI088_ACCEL_SEN * temp;
    temp = (int16_t)((buf[5] << 8) | buf[4]);
    accel[2] = BMI088_ACCEL_SEN * temp;

    bmi_gyro_muli_r(BMI088_GYRO_CHIP_ID, buf, 8);

    if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        temp = (int16_t)((buf[3] << 8) | buf[2]);
        gyro[0] = BMI088_GYRO_SEN * temp;
        temp = (int16_t)((buf[5] << 8) | buf[4]);
        gyro[1] = BMI088_GYRO_SEN * temp;
        temp = (int16_t)((buf[7] << 8) | buf[6]);
        gyro[2] = BMI088_GYRO_SEN * temp;
    }

    bmi_accel_muli_r(BMI088_TEMP_M, buf, 2);

    temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (temp > 1023)
    {
        temp = (int16_t)(temp - 2048);
    }

    *temperate = BMI088_TEMP_FACTOR * temp + BMI088_TEMP_OFFSET;
}

uint32_t bmi088_sensor_time(void)
{
    uint8_t buf[3];

    bmi_accel_muli_r(BMI088_SENSORTIME_DATA_L, buf, 3);

    return (uint32_t)((buf[2] << 16) | (buf[1] << 8) | (buf[0]));
}

float bmi088_temperate(void)
{
    uint8_t buf[2];
    int16_t temp;

    bmi_accel_muli_r(BMI088_TEMP_M, buf, 2);

    temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (temp > 1023)
    {
        temp = (int16_t)(temp - 2048);
    }

    return BMI088_TEMP_FACTOR * temp + BMI088_TEMP_OFFSET;
}

void bmi088_gyro(float gyro[3])
{
    uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
    int16_t temp;

    bmi_gyro_muli_r(BMI088_GYRO_X_L, buf, 6);

    temp = (int16_t)((buf[1] << 8) | buf[0]);
    gyro[0] = BMI088_GYRO_SEN * temp;
    temp = (int16_t)((buf[3] << 8) | buf[2]);
    gyro[1] = BMI088_GYRO_SEN * temp;
    temp = (int16_t)((buf[5] << 8) | buf[4]);
    gyro[2] = BMI088_GYRO_SEN * temp;
}

void bmi088_accel(float accel[3])
{
    uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
    int16_t temp;

    bmi_accel_muli_r(BMI088_ACCEL_XOUT_L, buf, 6);

    temp = (int16_t)((buf[1] << 8) | buf[0]);
    accel[0] = BMI088_ACCEL_SEN * temp;
    temp = (int16_t)((buf[3] << 8) | buf[2]);
    accel[1] = BMI088_ACCEL_SEN * temp;
    temp = (int16_t)((buf[5] << 8) | buf[4]);
    accel[2] = BMI088_ACCEL_SEN * temp;
}

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
