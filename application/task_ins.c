/**
 * *****************************************************************************
 * @file         task_ins.c/h
 * @brief        use bmi088 to calculate the euler angle. no use ist8310, so only
 *               enable data ready pin to save cpu time.enalbe bmi088 data ready
 *               enable spi DMA to save the time spi transmit
 * @author       ngu
 * @date         20210101
 * @version      1
 * @copyright    Copyright (C) 2021
 * @code         utf-8                                                  @endcode
 * *****************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "task_ins.h"

/* Private includes ----------------------------------------------------------*/
#include "bsp.h"
#include "cc.h"
#include "main.h"

/* ahrs */
#include "ahrs.h"
#include "bmi088.h"
#include "ist8310.h"

#if USED_OS
#include "cmsis_os.h"
#endif /* USED_OS */

/* Private define ------------------------------------------------------------*/

#define SPI_DMA_LENGHT_GYRO  8
#define SPI_DMA_LENGHT_ACCEL 9
#define SPI_DMA_LENGHT_TEMP  4

#define IMU_SHFITS_DR     0 /* Data Recorder */
#define IMU_SHFITS_SPI    1 /* SPI is running */
#define IMU_SHFITS_UPDATE 2 /* Data is updated */

#define BMI088_GYRO_RX_BUF_DATA_OFFSET  1
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET 2
#define IST8310_RX_BUF_DATA_OFFSET      16

#define TEMPERATURE_PID_KP       1600.0f
#define TEMPERATURE_PID_KI       0.2f
#define TEMPERATURE_PID_KD       0.0f
#define TEMPERATURE_PID_MAX_OUT  4500.0f
#define TEMPERATURE_PID_MAX_IOUT 4400.0f

#define MPU6500_TEMP_PWM_MAX 5000

#define INS_TASK_INIT_TIME 7

#define INS_GYRO_X 0
#define INS_GYRO_Y 1
#define INS_GYRO_Z 2

#define INS_ACCEL_X 0
#define INS_ACCEL_Y 1
#define INS_ACCEL_Z 2

#define INS_MAG_X 0
#define INS_MAG_Y 1
#define INS_MAG_Z 2

#define BMI088_BOARD_INSTALL_SPIN_MATRIX \
    {                                    \
        {0.0f, 1.0f, 0.0f},              \
            {-1.0f, 0.0f, 0.0f},         \
            {0.0f, 0.0f, 1.0f},          \
    }

#define IST8310_BOARD_INSTALL_SPIN_MATRIX \
    {                                     \
        {1.0f, 0.0f, 0.0f},               \
            {0.0f, 1.0f, 0.0f},           \
            {0.0f, 0.0f, 1.0f},           \
    }

/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static TaskHandle_t ins_task_local_handler;

static uint8_t dma_rx_buf_gyro[SPI_DMA_LENGHT_GYRO];
static uint8_t dma_tx_buf_gyro[SPI_DMA_LENGHT_GYRO] = {
    0x82,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
};

static uint8_t dma_rx_buf_accel[SPI_DMA_LENGHT_ACCEL];
static uint8_t dma_tx_buf_accel[SPI_DMA_LENGHT_ACCEL] = {
    0x92,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
};

static uint8_t dma_rx_buf_temp[SPI_DMA_LENGHT_TEMP];
static uint8_t dma_tx_buf_temp[SPI_DMA_LENGHT_TEMP] = {
    0xA2,
    0xFF,
    0xFF,
    0xFF,
};

volatile uint8_t flag_update_gyro  = 0; /* flag which update gyroscope */
volatile uint8_t flag_update_accel = 0; /* flag which update accelerometer */
volatile uint8_t flag_update_temp  = 0; /* flag which update temperature */
volatile uint8_t flag_update_mag   = 0; /* flag which update magnetometer */

volatile uint8_t flag_imu_dma = 0; /* flag which can dma tx */

bmi_t bmi;
ist_t ist;

/* Scale factor for gyroscope calibration */
float scale_factor_gyro[3][3] = BMI088_BOARD_INSTALL_SPIN_MATRIX;
/* Offset of gyroscope data */
float offset_gyro[3];
/* Offset of gyroscope data calibration */
float offset_cali_gyro[3];

/* Scale factor for accelerometer calibration */
float scale_factor_accel[3][3] = BMI088_BOARD_INSTALL_SPIN_MATRIX;
/* Offset of accelerometer data */
float offset_accel[3];
/* Offset of accelerometer data calibration */
float offset_cali_accel[3];

/* Scale factor for magnetometer calibration */
float scale_factor_mag[3][3] = IST8310_BOARD_INSTALL_SPIN_MATRIX;
/* Offset of magnetometer data */
float offset_mag[3];
/* Offset of magnetometer data calibration */
float offset_cali_mag[3];

static const float PID_temp[3] = {
    TEMPERATURE_PID_KP,
    TEMPERATURE_PID_KI,
    TEMPERATURE_PID_KD,
};
static cc_pid_t pid_temp;
static uint8_t  temp_first; /* the flag of the first temperature */

//static const float timing_time = 0.001f; /* task run time , unit s */

static float accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static float accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static float accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};

/* the number of fliter */
static const float fliter_num[3] = {
    1.929454039488895f,
    -0.93178349823448126f,
    0.002329458745586203f,
};

/* x,y,z axis of gyroscope */
static float ins_gyro[3] = {0.0f, 0.0f, 0.0f};
/* x,y,z axis of accelerometer */
static float ins_accel[3] = {0.0f, 0.0f, 0.0f};
/* x,y,z axis of magnetometer */
static float ins_mag[3] = {0.0f, 0.0f, 0.0f};
/* quaternionq[0]+q[1]* i +q[2]* j +q[3]* k */
static float ins_quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};

/* euler angle, unit rad */
float ins_angle[3] = {0.0f, 0.0f, 0.0f};

/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/

/**
 * @brief        open the SPI DMA accord to the value of flag_update_imu
*/
static void imu_spi_dma(void)
{
    /* Data is being transmitted. DMA is being used */
    if ((hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) ||
        (hspi1.hdmarx->Instance->CR & DMA_SxCR_EN))
    {
        return;
    }

    /* Start reading accelerometer's data */
    if ((flag_update_gyro & (1 << IMU_SHFITS_DR)) &&
        !(flag_update_accel & (1 << IMU_SHFITS_SPI)) &&
        !(flag_update_temp & (1 << IMU_SHFITS_SPI)))
    {
        /* Clear the flag of data recorder */
        flag_update_gyro &= ~(1 << IMU_SHFITS_DR);
        /* Set the flag that SPI is running */
        flag_update_gyro |= (1 << IMU_SHFITS_SPI);

        BMI088_GYRO_NS_L;
        spi_dma_start(&hspi1,
                      (uint32_t)dma_tx_buf_gyro,
                      (uint32_t)dma_rx_buf_gyro,
                      SPI_DMA_LENGHT_GYRO);
        return;
    }

    /* Start reading accelerometer's temperature */
    if ((flag_update_accel & (1 << IMU_SHFITS_DR)) &&
        !(flag_update_gyro & (1 << IMU_SHFITS_SPI)) &&
        !(flag_update_temp & (1 << IMU_SHFITS_SPI)))
    {
        /* Clear the flag of data recorder */
        flag_update_accel &= ~(1 << IMU_SHFITS_DR);
        /* Set the flag that SPI is running */
        flag_update_accel |= (1 << IMU_SHFITS_SPI);

        BMI088_ACCEL_NS_L;
        spi_dma_start(&hspi1,
                      (uint32_t)dma_tx_buf_accel,
                      (uint32_t)dma_rx_buf_accel,
                      SPI_DMA_LENGHT_ACCEL);
        return;
    }

    /* Start reading gyroscope's data */
    if ((flag_update_temp & (1 << IMU_SHFITS_DR)) &&
        !(flag_update_gyro & (1 << IMU_SHFITS_SPI)) &&
        !(flag_update_accel & (1 << IMU_SHFITS_SPI)))
    {
        /* Clear the flag of data recorder */
        flag_update_temp &= ~(1 << IMU_SHFITS_DR);
        /* Set the flag that SPI is running */
        flag_update_temp |= (1 << IMU_SHFITS_SPI);

        BMI088_ACCEL_NS_L;
        spi_dma_start(&hspi1,
                      (uint32_t)dma_tx_buf_temp,
                      (uint32_t)dma_rx_buf_temp,
                      SPI_DMA_LENGHT_TEMP);
        return;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == INT1_ACCEL_PIN)
    {
        /* Set the flag that accelerometer's data is update */
        flag_update_accel |= (1 << IMU_SHFITS_DR);
        /* Set the flag that accelerometer's temperature is update */
        flag_update_temp |= (1 << IMU_SHFITS_DR);

        if (flag_imu_dma)
        {
            imu_spi_dma();
        }
    }
    else if (GPIO_Pin == INT1_GYRO_PIN)
    {
        /* Set the flag that gyroscope's data is update */
        flag_update_gyro |= (1 << IMU_SHFITS_DR);

        if (flag_imu_dma)
        {
            imu_spi_dma();
        }
    }
    else if (GPIO_Pin == IST8310_DRDY_PIN)
    {
        /* Set the flag that magnetometer's data is update */
        flag_update_mag |= (1 << IMU_SHFITS_DR);
    }
    else if (GPIO_Pin == GPIO_PIN_0)
    {
        /* Wake up the task */
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR(ins_task_local_handler, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

void DMA2_Stream2_IRQHandler(void)
{
    /* Complete Transmit */
    if (BSP_DMA_FLAG_TC(hspi1.hdmarx))
    {
        /* Clear Complete Transmit flag */
        BSP_DMA_CLEAR_TC(hspi1.hdmarx);

        /* read over gyroscope */
        if (flag_update_gyro & (1 << IMU_SHFITS_SPI))
        {
            /* Clear the flag that SPI is running */
            flag_update_gyro &= ~(1 << IMU_SHFITS_SPI);
            /* Set the flag that gyroscope's data is updated */
            flag_update_gyro |= (1 << IMU_SHFITS_UPDATE);
            /* Release the slave machine */
            BMI088_GYRO_NS_H;
        }

        /* read over accelerometer */
        if (flag_update_accel & (1 << IMU_SHFITS_SPI))
        {
            /* Clear the flag that SPI is running */
            flag_update_accel &= ~(1 << IMU_SHFITS_SPI);
            /* Set the flag that accelerometer's data is updated */
            flag_update_accel |= (1 << IMU_SHFITS_UPDATE);
            /* Release the slave machine */
            BMI088_ACCEL_NS_H;
        }

        /* read over temperature */
        if (flag_update_temp & (1 << IMU_SHFITS_SPI))
        {
            /* Clear the flag that SPI is running */
            flag_update_temp &= ~(1 << IMU_SHFITS_SPI);
            /* Set the flag that accelerometer's temperature is updated */
            flag_update_temp |= (1 << IMU_SHFITS_UPDATE);
            /* Release the slave machine */
            BMI088_ACCEL_NS_H;
        }

        /* open the SPI DMA accord to the value of flag_update_imu */
        imu_spi_dma();

        if (flag_update_gyro & (1 << IMU_SHFITS_UPDATE))
        {
            /* update magnetometer' data by EXTI0 */
            __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
        }
    }
}

/**
 * @brief          rotate the gyro, accel and mag, and calculate the zero drift, 
 *                 because sensors have different install derection.
 * @param[out]     gyro:  after plus zero drift and rotate
 * @param[out]     accel: after plus zero drift and rotate
 * @param[out]     mag:   after plus zero drift and rotate
 * @param[in]      bmi:   gyro and accel data
 * @param[in]      ist:   mag data
*/
static void imu_cali_slove(float  gyro[3],
                           float  accel[3],
                           float  mag[3],
                           bmi_t *bmi,
                           ist_t *ist)
{
    for (uint8_t i = 0U; i != 3U; ++i)
    {
        /* calculate the zero drift of gyroscope */
        gyro[i] = bmi->gyro[INS_GYRO_X] * scale_factor_gyro[i][INS_GYRO_X] +
                  bmi->gyro[INS_GYRO_Y] * scale_factor_gyro[i][INS_GYRO_Y] +
                  bmi->gyro[INS_GYRO_Z] * scale_factor_gyro[i][INS_GYRO_Z] +
                  offset_gyro[i];

        /* calculate the zero drift of accelerometer */
        accel[i] = bmi->accel[INS_ACCEL_X] * scale_factor_accel[i][INS_ACCEL_X] +
                   bmi->accel[INS_ACCEL_Y] * scale_factor_accel[i][INS_ACCEL_Y] +
                   bmi->accel[INS_ACCEL_Z] * scale_factor_accel[i][INS_ACCEL_Z] +
                   offset_accel[i];

        /* calculate the zero drift of magnetometer */
        mag[i] = ist->mag[INS_MAG_X] * scale_factor_mag[i][INS_MAG_X] +
                 ist->mag[INS_MAG_Y] * scale_factor_mag[i][INS_MAG_Y] +
                 ist->mag[INS_MAG_Z] * scale_factor_mag[i][INS_MAG_Z] +
                 offset_mag[i];
    }
}

/**
 * @brief          calculate gyro zero drift
 * @param[out]     offset: zero drift
 * @param[in]      gyro:   gyro data
 * @param[out]     count:  +1 auto
*/
void offset_gyro_calc(float     offset[3],
                      float     gyro[3],
                      uint16_t *count)
{
    offset[INS_GYRO_X] -= 0.0003f * gyro[INS_GYRO_X];
    offset[INS_GYRO_Y] -= 0.0003f * gyro[INS_GYRO_Y];
    offset[INS_GYRO_Z] -= 0.0003f * gyro[INS_GYRO_Z];

    (*count)++;
}

/**
 * @brief          calculate gyro zero drift
 * @param[out]     scale:  scale, default 1.0
 * @param[out]     offset: zero drift, collect the gyro ouput when in still
 * @param[out]     count:  time, when call offset_gyro_calc 
*/
void ins_cali_gyro(float     scale[3],
                   float     offset[3],
                   uint16_t *count)
{
    if (*count == 0)
    {
        offset_gyro[INS_GYRO_X] = offset_cali_gyro[INS_GYRO_X];
        offset_gyro[INS_GYRO_Y] = offset_cali_gyro[INS_GYRO_Y];
        offset_gyro[INS_GYRO_Z] = offset_cali_gyro[INS_GYRO_Z];
    }

    offset_gyro_calc(offset_gyro, ins_gyro, count);

    offset[INS_GYRO_X] = offset_gyro[INS_GYRO_X];
    offset[INS_GYRO_Y] = offset_gyro[INS_GYRO_Y];
    offset[INS_GYRO_Z] = offset_gyro[INS_GYRO_Z];

    scale[INS_GYRO_X] = 1.0f;
    scale[INS_GYRO_Y] = 1.0f;
    scale[INS_GYRO_Z] = 1.0f;
}

/**
 * @brief          get gyro zero drift from flash
 * @param[in]      scale:  scale, default 1.0
 * @param[in]      offset: zero drift, 
*/
void ins_cali_gyro_set(float scale[3], float offset[3])
{
    offset_cali_gyro[INS_GYRO_X] = offset[INS_GYRO_X];
    offset_cali_gyro[INS_GYRO_Y] = offset[INS_GYRO_Y];
    offset_cali_gyro[INS_GYRO_Z] = offset[INS_GYRO_Z];

    offset_gyro[INS_GYRO_X] = offset_cali_gyro[INS_GYRO_X];
    offset_gyro[INS_GYRO_Y] = offset_cali_gyro[INS_GYRO_Y];
    offset_gyro[INS_GYRO_Z] = offset_cali_gyro[INS_GYRO_Z];
}

/**
 * @brief        control the temperature of bmi088
 * @param[in]    temp: the temperature of bmi088
*/
static void imu_temp_control(float temp)
{
    uint16_t temp_pwm;

    static uint8_t temp_time = 0;

    if (temp_first)
    {
        cc_pid(&pid_temp, temp, 40);
        if (pid_temp.out < 0.0f)
        {
            pid_temp.out = 0.0f;
        }

        temp_pwm = (uint16_t)pid_temp.out;
        imu_pwm_set(temp_pwm);
    }
    else
    {
        /* in beginning, max power */
        if (temp > 40)
        {
            temp_time++;
            if (temp_time > 200)
            {
                temp_first = 1;

                pid_temp.out_i = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }

        imu_pwm_set(MPU6500_TEMP_PWM_MAX - 1);
    }
}

/**
 * @brief        Get the half of sampling period
 * @return       float 
*/
static float ht_get(void)
{
    static volatile uint32_t update_last, update_now;

    update_now = HAL_GetTick(); /* ms */

    float ret = (float)(update_now - update_last) / 2000.0f;

    update_last = update_now;

    return ret;
}

/**
 * @brief          imu task, init bmi088, ist8310, calculate the euler angle
 * @param[in]      pvParameters: NULL
*/
void task_ins(void const *pvParameters)
{
    /* wait a time */
    osDelay(INS_TASK_INIT_TIME);

    while (bmi088_init())
    {
        osDelay(100);
    }
    while (ist8310_init())
    {
        osDelay(100);
    }

    imu_pwm_start();

    bmi088_read(bmi.gyro, bmi.accel, &bmi.temp);
    /* rotate and zero drift */
    imu_cali_slove(ins_gyro, ins_accel, ins_mag, &bmi, &ist);

    ahrs_quat_init(ins_quat, ins_mag[INS_MAG_X], ins_mag[INS_MAG_Y]);

    cc_pid_position(&pid_temp,
                    PID_temp,
                    TEMPERATURE_PID_MAX_OUT,
                    TEMPERATURE_PID_MAX_IOUT);

    accel_fliter_1[INS_ACCEL_X] =
        accel_fliter_2[INS_ACCEL_X] =
            accel_fliter_3[INS_ACCEL_X] = ins_accel[INS_ACCEL_X];
    accel_fliter_1[INS_ACCEL_Y] =
        accel_fliter_2[INS_ACCEL_Y] =
            accel_fliter_3[INS_ACCEL_Y] = ins_accel[INS_ACCEL_Y];
    accel_fliter_1[INS_ACCEL_Z] =
        accel_fliter_2[INS_ACCEL_Z] =
            accel_fliter_3[INS_ACCEL_Z] = ins_accel[INS_ACCEL_Z];

    /* get the handle of task */
    ins_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));

    spi_dma_start(&hspi1,
                  (uint32_t)dma_tx_buf_gyro,
                  (uint32_t)dma_rx_buf_gyro,
                  SPI_DMA_LENGHT_GYRO);

    flag_imu_dma = 1;

    for (;;)
    {
        /* Wait spi DMA tansmit done */
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
        {
        }

        /* read over gyroscope */
        if (flag_update_gyro & (1 << IMU_SHFITS_UPDATE))
        {
            /* Clear the flag that data is updated */
            flag_update_gyro &= ~(1 << IMU_SHFITS_UPDATE);

            bmi088_read_over_gyro(dma_rx_buf_gyro + BMI088_GYRO_RX_BUF_DATA_OFFSET,
                                  bmi.gyro);
        }

        /* read over accelerometer */
        if (flag_update_accel & (1 << IMU_SHFITS_UPDATE))
        {
            /* Clear the flag that data is updated */
            flag_update_accel &= ~(1 << IMU_SHFITS_UPDATE);

            bmi088_read_over_accel(dma_rx_buf_accel + BMI088_ACCEL_RX_BUF_DATA_OFFSET,
                                   bmi.accel,
                                   &bmi.time);
        }

        /* read over temperature */
        if (flag_update_temp & (1 << IMU_SHFITS_UPDATE))
        {
            /* Clear the flag that data is updated */
            flag_update_temp &= ~(1 << IMU_SHFITS_UPDATE);

            bmi088_read_over_temp(dma_rx_buf_temp + BMI088_ACCEL_RX_BUF_DATA_OFFSET,
                                  &bmi.temp);

            imu_temp_control(bmi.temp);
        }

        /* read magnetometer */
        if (flag_update_mag &= 1 << IMU_SHFITS_DR)
        {
            flag_update_mag &= ~(1 << IMU_SHFITS_DR);
            flag_update_mag |= (1 << IMU_SHFITS_SPI);

            ist8310_read(ist.mag);
        }

        /* rotate and zero drift */
        imu_cali_slove(ins_gyro, ins_accel, ins_mag, &bmi, &ist);

        /* accel low-pass filter */
        accel_fliter_1[INS_ACCEL_X] = accel_fliter_2[INS_ACCEL_X];
        accel_fliter_2[INS_ACCEL_X] = accel_fliter_3[INS_ACCEL_X];
        accel_fliter_3[INS_ACCEL_X] = accel_fliter_2[INS_ACCEL_X] * fliter_num[0] +
                                      accel_fliter_1[INS_ACCEL_X] * fliter_num[1] +
                                      ins_accel[INS_ACCEL_X] * fliter_num[2];

        accel_fliter_1[INS_ACCEL_Y] = accel_fliter_2[INS_ACCEL_Y];
        accel_fliter_2[INS_ACCEL_Y] = accel_fliter_3[INS_ACCEL_Y];
        accel_fliter_3[INS_ACCEL_Y] = accel_fliter_2[INS_ACCEL_Y] * fliter_num[0] +
                                      accel_fliter_1[INS_ACCEL_Y] * fliter_num[1] +
                                      ins_accel[INS_ACCEL_Y] * fliter_num[2];

        accel_fliter_1[INS_ACCEL_Z] = accel_fliter_2[INS_ACCEL_Z];
        accel_fliter_2[INS_ACCEL_Z] = accel_fliter_3[INS_ACCEL_Z];
        accel_fliter_3[INS_ACCEL_Z] = accel_fliter_2[INS_ACCEL_Z] * fliter_num[0] +
                                      accel_fliter_1[INS_ACCEL_Z] * fliter_num[1] +
                                      ins_accel[INS_ACCEL_Z] * fliter_num[2];

        /* Mehony AHRS attitude calculation with magnetometer */
        ahrs_mahony(ins_quat, ins_gyro, accel_fliter_3, ins_mag, ht_get());
        /* Quaternion to Euler Angle */
        ahrs_euler_angle(ins_quat,
                         &ins_angle[INS_ROLL],
                         &ins_angle[INS_PITCH],
                         &ins_angle[INS_YAW]);
#if 0
        os_putf(ins_angle[0], 5);
        os_printf(",");
        os_putf(ins_angle[1], 5);
        os_printf(",");
        os_putf(ins_angle[2], 5);
        os_printf(",");
        os_putf(bmi.temp, 5);
        os_printf("\r\n");
#elif 1
        os_justfloat(4, ins_angle[0], ins_angle[1], ins_angle[2], bmi.temp);
#endif
    }
}

/************************ (C) COPYRIGHT ngu ********************END OF FILE****/
