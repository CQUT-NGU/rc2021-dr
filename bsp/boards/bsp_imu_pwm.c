/**
 * *****************************************************************************
 * @file         bsp_imu_pwm.c
 * @brief        The IMU temperature is controlled by PWM
 * @author       NGU
 * @date         20210427
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
 * *****************************************************************************
*/

#include "bsp_imu_pwm.h"

#include "main.h"

#undef htim
#define htim htim10
#undef IMU_CHANNEL
#define IMU_CHANNEL TIM_CHANNEL_1

extern TIM_HandleTypeDef htim;

void imu_pwm_start(void)
{
    /*!< Set the TIM Prescaler on runtime */
    __HAL_TIM_SET_PRESCALER(&htim, 0U);
    /*!< Set the TIM Autoreload Register value on runtime */
    __HAL_TIM_SetAutoreload(&htim, 4999U);
    /*!< Set the TIM Clock Division value on runtime */
    __HAL_TIM_SetClockDivision(&htim, TIM_CLOCKDIVISION_DIV1);
    /*!< Starts the PWM signal generation */
    HAL_TIM_PWM_Start(&htim, IMU_CHANNEL);
}

void imu_pwm_stop(void)
{
    /*!< Stops the PWM signal generation */
    HAL_TIM_PWM_Stop(&htim, IMU_CHANNEL);
}

void imu_pwm_set(uint16_t pwm)
{
    /*!< Set the TIM Capture Compare Register value on runtime */
    __HAL_TIM_SetCompare(&htim, IMU_CHANNEL, pwm);
}

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
