/**
 ******************************************************************************
 * @file    can.c
 * @brief   This file provides code for the configuration
 *          of the CAN instances.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* CAN1 init function */
void MX_CAN1_Init(void) {
	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 4;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_4TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = ENABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = ENABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK) {
		Error_Handler();
	}

	CAN_FilterTypeDef sFilterConfig;

	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}
}

/* CAN2 init function */
void MX_CAN2_Init(void) {
	hcan2.Instance = CAN2;
	hcan2.Init.Prescaler = 4;
	hcan2.Init.Mode = CAN_MODE_NORMAL;
	hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan2.Init.TimeSeg1 = CAN_BS1_4TQ;
	hcan2.Init.TimeSeg2 = CAN_BS2_3TQ;
	hcan2.Init.TimeTriggeredMode = DISABLE;
	hcan2.Init.AutoBusOff = ENABLE;
	hcan2.Init.AutoWakeUp = DISABLE;
	hcan2.Init.AutoRetransmission = DISABLE;
	hcan2.Init.ReceiveFifoLocked = DISABLE;
	hcan2.Init.TransmitFifoPriority = ENABLE;
	if (HAL_CAN_Init(&hcan2) != HAL_OK) {
		Error_Handler();
	}

	CAN_FilterTypeDef sFilterConfig;

	sFilterConfig.FilterBank = 14;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x000;
	sFilterConfig.FilterIdLow = 0x000;
	sFilterConfig.FilterMaskIdHigh = 0x000;
	sFilterConfig.FilterMaskIdLow = 0x000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED = 0;

void HAL_CAN_MspInit(CAN_HandleTypeDef *canHandle) {

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (canHandle->Instance == CAN1) {
		/* USER CODE BEGIN CAN1_MspInit 0 */

		/* USER CODE END CAN1_MspInit 0 */
		/* CAN1 clock enable */
		HAL_RCC_CAN1_CLK_ENABLED++;
		if (HAL_RCC_CAN1_CLK_ENABLED == 1) {
			__HAL_RCC_CAN1_CLK_ENABLE();
		}

		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**CAN1 GPIO Configuration
		 PA11     ------> CAN1_RX
		 PA12     ------> CAN1_TX
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_11;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_12;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* CAN1 interrupt Init */
		HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
		HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
		HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
		/* USER CODE BEGIN CAN1_MspInit 1 */

		/* USER CODE END CAN1_MspInit 1 */
	} else if (canHandle->Instance == CAN2) {
		/* USER CODE BEGIN CAN2_MspInit 0 */

		/* USER CODE END CAN2_MspInit 0 */
		/* CAN2 clock enable */
		__HAL_RCC_CAN2_CLK_ENABLE();
		HAL_RCC_CAN1_CLK_ENABLED++;
		if (HAL_RCC_CAN1_CLK_ENABLED == 1) {
			__HAL_RCC_CAN1_CLK_ENABLE();
		}

		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**CAN2 GPIO Configuration
		 PB5     ------> CAN2_RX
		 PB6     ------> CAN2_TX
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_5;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_6;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		__HAL_AFIO_REMAP_CAN2_ENABLE();

		HAL_NVIC_SetPriority(CAN2_TX_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
		HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
		HAL_NVIC_SetPriority(CAN2_SCE_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(CAN2_SCE_IRQn);
	}
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef *canHandle) {

	if (canHandle->Instance == CAN1) {
		/* USER CODE BEGIN CAN1_MspDeInit 0 */

		/* USER CODE END CAN1_MspDeInit 0 */
		/* Peripheral clock disable */
		HAL_RCC_CAN1_CLK_ENABLED--;
		if (HAL_RCC_CAN1_CLK_ENABLED == 0) {
			__HAL_RCC_CAN1_CLK_DISABLE();
		}

		/**CAN1 GPIO Configuration
		 PA11     ------> CAN1_RX
		 PA12     ------> CAN1_TX
		 */
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);

		/* CAN1 interrupt Deinit */
		HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
		HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
		/* USER CODE BEGIN CAN1_MspDeInit 1 */

		/* USER CODE END CAN1_MspDeInit 1 */
	} else if (canHandle->Instance == CAN2) {
		/* USER CODE BEGIN CAN2_MspDeInit 0 */

		/* USER CODE END CAN2_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_CAN2_CLK_DISABLE();
		HAL_RCC_CAN1_CLK_ENABLED--;
		if (HAL_RCC_CAN1_CLK_ENABLED == 0) {
			__HAL_RCC_CAN1_CLK_DISABLE();
		}

		/**CAN2 GPIO Configuration
		 PB5     ------> CAN2_RX
		 PB6     ------> CAN2_TX
		 */
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5 | GPIO_PIN_6);

		HAL_NVIC_DisableIRQ(CAN2_TX_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX1_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_SCE_IRQn);
	}
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
