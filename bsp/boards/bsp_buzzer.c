/**
 * *****************************************************************************
 * @file         bsp_buzzer.c/h
 * @brief        buzzer of boards
 * @author       ngu
 * @date         20210501
 * @version      1
 * @copyright    Copyright (c) 2021
 * @code         utf-8                                                  @endcode
 * *****************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "bsp_buzzer.h"

/* Private includes ----------------------------------------------------------*/
#include "main.h"

/* Private define ------------------------------------------------------------*/

#undef htim
#define htim htim4
#undef BUZZER_CHANNEL
#define BUZZER_CHANNEL TIM_CHANNEL_3

/* Private define ------------------------------------------------------------*/
extern TIM_HandleTypeDef htim;

/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/

void buzzer_start(void)
{
    /*!< Set the TIM Autoreload Register value on runtime */
    __HAL_TIM_SetAutoreload(&htim, BUZZER_PWM_MAX - 1U);
    /*!< Set the TIM Clock Division value on runtime */
    __HAL_TIM_SetClockDivision(&htim, TIM_CLOCKDIVISION_DIV1);
    /*!< Starts the PWM signal generation */
    HAL_TIM_PWM_Start(&htim, BUZZER_CHANNEL);
}

void buzzer_stop(void)
{
    /*!< Stops the PWM signal generation */
    HAL_TIM_PWM_Stop(&htim, BUZZER_CHANNEL);
}

void buzzer_set(uint16_t psc,
                uint16_t pwm)
{
    /*!< Set the TIM Prescaler on runtime */
    __HAL_TIM_SET_PRESCALER(&htim, 0xFFFU & psc);
    /*!< Set the TIM Capture Compare Register value on runtime */
    __HAL_TIM_SetCompare(&htim, BUZZER_CHANNEL, 0x4FFFU & pwm);
}

/************************ (C) COPYRIGHT ngu ********************END OF FILE****/
