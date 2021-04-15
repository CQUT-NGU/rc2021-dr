/**
 * *****************************************************************************
 * @file         ist8310.c/h
 * @brief        ist8310 driver
 * @author       ngu
 * @date         20210427
 * @version      1
 * @copyright    Copyright (c) 2021
 * @code         utf-8                                                  @endcode
 * *****************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "ist8310.h"

/* Private includes ----------------------------------------------------------*/
#include "bsp.h"
#include "main.h"

#if USED_OS
#include "cmsis_os.h"
#endif /* USED_OS */

/* Private define ------------------------------------------------------------*/

#undef hi2c
#define hi2c hi2c3

#define IST8310_IIC_ADDRESS  (0x0E << 1)
#define IST8310_IIC_READ_MSB (0x80)

#define IST8310_DATA_READY_BIT 2

#define MAG_SEN 0.3f /* to uT */

#define IST8310_WHO_AM_I       0x00 /* ist8310 who am I */
#define IST8310_WHO_AM_I_VALUE 0x10

#define IST8310_WRITE_REG_NUM 4

/* Private includes ----------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c;

/* Private macro -------------------------------------------------------------*/

#define ist_delay_ms(ms) osDelay(ms)
#define ist_delay_us(us) delay_us(us)

/* Private typedef -----------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static const uint8_t ist8310_error[IST8310_WRITE_REG_NUM][3] = {
    {0x0B, 0x08, 0x01},
    {0x41, 0x09, 0x02},
    {0x42, 0xC0, 0x03},
    {0x0A, 0x0B, 0x04},
};

/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/

uint8_t ist_reg_r(uint8_t reg)
{
    uint8_t res;
    HAL_I2C_Mem_Read(&hi2c,
                     IST8310_IIC_ADDRESS,
                     reg,
                     I2C_MEMADD_SIZE_8BIT,
                     &res,
                     1,
                     100);
    return res;
}

void ist_reg_w(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c,
                      IST8310_IIC_ADDRESS,
                      reg,
                      I2C_MEMADD_SIZE_8BIT,
                      &data,
                      1,
                      100);
}

void ist_reg_muli_r(uint8_t  reg,
                    uint8_t *buf,
                    uint8_t  len)
{
    HAL_I2C_Mem_Read(&hi2c,
                     IST8310_IIC_ADDRESS,
                     reg,
                     I2C_MEMADD_SIZE_8BIT,
                     buf,
                     len,
                     100);
}

void ist_reg_muli_w(uint8_t  reg,
                    uint8_t *data,
                    uint8_t  len)
{
    HAL_I2C_Mem_Write(&hi2c,
                      IST8310_IIC_ADDRESS,
                      reg,
                      I2C_MEMADD_SIZE_8BIT,
                      data,
                      len,
                      100);
}

uint8_t ist8310_init(void)
{
    static const uint8_t wait_time  = 1;
    static const uint8_t sleep_time = 50;

    uint8_t res = 0;
    uint8_t num = 0;

    IST8310_RST_L;
    ist_delay_ms(sleep_time);
    IST8310_RST_H;
    ist_delay_ms(sleep_time);

    res = ist_reg_r(IST8310_WHO_AM_I);
    if (res != IST8310_WHO_AM_I_VALUE)
    {
        return IST8310_NO_SENSOR;
    }
    ist_delay_ms(wait_time);

    /* set mpu6500 sonsor config and check */
    for (num = 0; num != IST8310_WRITE_REG_NUM; ++num)
    {
        ist_reg_w(ist8310_error[num][0], ist8310_error[num][1]);
        ist_delay_ms(wait_time);
        res = ist_reg_r(ist8310_error[num][0]);
        ist_delay_ms(wait_time);

        if (res != ist8310_error[num][1])
        {
            return ist8310_error[num][2];
        }
    }

    return IST8310_NO_ERROR;
}

void ist8310_read_over(uint8_t *buf,
                       ist_t *  ist)
{
    if (buf[0] & 0x01)
    {
        int16_t temp = 0;
        ist->status |= (1 << IST8310_DATA_READY_BIT);

        temp        = (int16_t)((buf[2] << 8) | buf[1]);
        ist->mag[0] = MAG_SEN * temp;
        temp        = (int16_t)((buf[4] << 8) | buf[3]);
        ist->mag[1] = MAG_SEN * temp;
        temp        = (int16_t)((buf[6] << 8) | buf[5]);
        ist->mag[2] = MAG_SEN * temp;
    }
    else
    {
        ist->status &= ~(1 << IST8310_DATA_READY_BIT);
    }
}

void ist8310_read(float mag[3])
{
    uint8_t buf[6];
    int16_t temp = 0;

    ist_reg_muli_r(0x03, buf, 6);

    temp   = (int16_t)((buf[1] << 8) | buf[0]);
    mag[0] = MAG_SEN * temp;
    temp   = (int16_t)((buf[3] << 8) | buf[2]);
    mag[1] = MAG_SEN * temp;
    temp   = (int16_t)((buf[5] << 8) | buf[4]);
    mag[2] = MAG_SEN * temp;
}

/************************ (C) COPYRIGHT ngu ********************END OF FILE****/
