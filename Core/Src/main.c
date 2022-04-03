/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
#include "main.h"
#include "can.h"
#include "stdio.h"

CAN_TxHeaderTypeDef TxHeaderCan1;
CAN_RxHeaderTypeDef RxHeaderCan1;
CAN_TxHeaderTypeDef TxHeaderCan2;
CAN_RxHeaderTypeDef RxHeaderCan2;

uint8_t TxDataCan1[8];
uint8_t RxDataCan1[8];
uint8_t TxDataCan2[8];
uint8_t RxDataCan2[8];

uint32_t TxMailboxCan1;
uint32_t TxMailboxCan2;

void SystemClock_Config(void);
void CAN1_Transmit_manual(uint32_t ID_CAN, uint32_t DLC_CAN, uint8_t *DATA_CAN);
void CAN2_Transmit_manual(uint32_t ID_CAN, uint32_t DLC_CAN, uint8_t *DATA_CAN);

int main(void) {

	HAL_Init();

	SystemClock_Config();

	MX_CAN1_Init();
	MX_CAN2_Init();

	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan1,
			CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF
					| CAN_IT_LAST_ERROR_CODE) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_CAN_Start(&hcan2) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan2,
			CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF
					| CAN_IT_LAST_ERROR_CODE) != HAL_OK) {
		Error_Handler();
	}

	while (1) {
	}
}

void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_PLL2;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
	RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL2_ON;
	RCC_OscInitStruct.PLL2.PLL2MUL = RCC_PLL2_MUL8;
	RCC_OscInitStruct.PLL2.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV5;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure the Systick interrupt time
	 */
	__HAL_RCC_PLLI2S_ENABLE();
}

// can fifo0 can1 callback
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeaderCan1, RxDataCan1)
			== HAL_OK) {
		if (RxHeaderCan1.StdId == 0x350 && RxDataCan1[0] == 0xc7) {
			RxDataCan1[0] = 0xc6;
		}

		CAN2_Transmit_manual(RxHeaderCan1.StdId, RxHeaderCan1.DLC, RxDataCan1);
	}
}

// can fifo1 for can2 callback
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeaderCan2, RxDataCan2)
			== HAL_OK) {
		CAN1_Transmit_manual(RxHeaderCan2.StdId, RxHeaderCan2.DLC, RxDataCan2);
	}
}

void CAN1_Transmit_manual(uint32_t ID_CAN, uint32_t DLC_CAN, uint8_t *DATA_CAN) {
	//wait while mailbox will be free
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {

	}

	TxHeaderCan1.StdId = ID_CAN;
	TxHeaderCan1.DLC = DLC_CAN;

	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeaderCan1, DATA_CAN, &TxMailboxCan1)
			!= HAL_OK) {
		Error_Handler();
	}
}
void CAN2_Transmit_manual(uint32_t ID_CAN, uint32_t DLC_CAN, uint8_t *DATA_CAN) {
	//wait while mailbox will be free
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0) {

	}

	TxHeaderCan2.StdId = ID_CAN;
	TxHeaderCan2.DLC = DLC_CAN;

	if (HAL_CAN_AddTxMessage(&hcan2, &TxHeaderCan2, DATA_CAN, &TxMailboxCan2)
			!= HAL_OK) {
		Error_Handler();
	}
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
	uint32_t er = HAL_CAN_GetError(hcan);
	HAL_CAN_ResetError(hcan);
}

void Error_Handler(void) {
	__disable_irq();

	while (1) {

	}
}
