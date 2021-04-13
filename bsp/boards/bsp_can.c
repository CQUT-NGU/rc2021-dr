/**
 * *****************************************************************************
 * @file         bsp_can.c/h
 * @brief        can of boards
 * @author       ngu
 * @date         20210101
 * @version      1
 * @copyright    Copyright (c) 2021
 * @code         utf-8                                                  @endcode
 * @details
 *               CAN1_RX ------> PD0
 *               CAN1_TX ------> PD1
 *               CAN2_RX ------> PB5
 *               CAN2_TX ------> PB6
 * *****************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "bsp_can.h"

/* Private includes ----------------------------------------------------------*/
#include "main.h"

extern CAN_HandleTypeDef hcan1;
#if CAN2_IS_ENABLE
extern CAN_HandleTypeDef hcan2;
#endif /* CAN2_IS_ENABLE */

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/

void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter = {0};
    /*!< Enable filter  */
    can_filter.FilterActivation = ENABLE;
    /*!< Identifier mask mode */
    can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    /*!< One 32-bit filter  */
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
    /*!< CAN receive FIFO 0 */
    can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    /*!< Specifies the filter identification number */
    can_filter.FilterIdHigh = 0x0000;
    /*!< Specifies the filter identification number */
    can_filter.FilterIdLow = 0x0000;
    /*!< Specifies the filter mask number or identification number */
    can_filter.FilterMaskIdHigh = 0x0000;
    /*!< Specifies the filter mask number or identification number */
    can_filter.FilterMaskIdLow = 0x0000;
    /*!< Specifies the filter bank which will be initialized */
    can_filter.FilterBank = 0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter);
    HAL_CAN_Start(&hcan1);

    /*!< FIFO 0 message pending interrupt */
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

#if CAN2_IS_ENABLE
    /*!< Select the start filter bank for the slave CAN instance */
    can_filter.SlaveStartFilterBank = 14;
    /*!< Specifies the filter bank which will be initialized */
    can_filter.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter);
    HAL_CAN_Start(&hcan2);

    /*!< FIFO 0 message pending interrupt */
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
#endif /* CAN2_IS_ENABLE */
}

/************************ (C) COPYRIGHT ngu ********************END OF FILE****/
