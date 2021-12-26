/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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

#include "main.h"
#include "can.h"
#include "i2c.h"
#include "gpio.h"
#include "mpu6050.h"
#include "BUTTON.h"
#include "flash_memory.h"
#include "stdio.h"

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
CAN_TxHeaderTypeDef gyroHeader;

uint8_t TxData[8];
uint8_t RxData[8];
uint8_t gyroData[8];
uint32_t TxMailbox;

MPU6050_t MPU6050;

void SystemClock_Config(void);
void CAN1_Transmit_manual(uint16_t ID_CAN, uint8_t DLC_CAN, uint8_t *DATA_CAN);
void CAN2_Transmit_manual(uint16_t ID_CAN, uint8_t DLC_CAN, uint8_t *DATA_CAN);
void sendGyroData(int x, int y);

int _write(int32_t file, uint8_t *ptr, int32_t len) {
	/* Implement your write code here, this is used by puts and printf for example */
	int i = 0;
	for (i = 0; i < len; i++)
		ITM_SendChar((*ptr++));
	return len;

}

int main(void) {

	HAL_Init();

	SystemClock_Config();

	MX_GPIO_Init();
	MX_CAN1_Init();
	MX_CAN2_Init();
	MX_I2C1_Init();
	flashMemoryInit();

	printf("okokokokoo");

	//MPU initialize
	while (MPU6050_Init(&hi2c1) == 1)
		;

	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan1,
	CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_CAN_Start(&hcan2) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan2,
	CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK) {
		Error_Handler();
	}

	//loop
	while (1) {
		MPU6050_Read_All(&hi2c1, &MPU6050);

		double realX = MPU6050.KalmanAngleX;
		double realY = MPU6050.KalmanAngleY;

		double storedX = readXFromFlash();
		double storedY = readYFromFlash();

		double pitch = storedX - realX;
		double roll = storedY - realY;

		if (pitch > 40) {
			pitch = 40;
		}
		if (pitch < -40) {
			pitch = -40;
		}
		if (roll > 60) {
			roll = 60;
		}
		if (roll < -60) {
			roll = -60;
		}

		//20 & -20 degree max
		pitch += 0x78;
		//29 & -29 degree max
		roll += 0x78;

		if (isCorrectionAssign) {
			sendGyroData(pitch, roll);
		}

		if (BUTTON_STATE(CALIBRATE_MPU_BUTTON) == 1 || !isCorrectionAssign) {
			saveGyroData(realX, realY);
		}

		HAL_Delay(200);
	}
}

void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 25;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
	/** Configure the Systick interrupt time
	 */
	__HAL_RCC_PLLI2S_ENABLE();
}

// can fifo callback
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {

		switch ((uint32_t) hcan->Instance) {
		case (uint32_t) CAN2:
			CAN1_Transmit_manual(RxHeader.StdId, RxHeader.DLC, RxData);
			break;
		case (uint32_t) CAN1:
			if (RxHeader.StdId == 0x350 && RxData[0] == 0xc7) {
				RxData[0] = 0xc6;
			}

			CAN2_Transmit_manual(RxHeader.StdId, RxHeader.DLC, RxData);
			break;
		}

	}

}

void CAN1_Transmit_manual(uint16_t ID_CAN, uint8_t DLC_CAN, uint8_t *DATA_CAN) {
	TxHeader.StdId = RxHeader.StdId;
	TxHeader.DLC = RxHeader.DLC;
	TxData[0] = DATA_CAN[0];
	TxData[1] = DATA_CAN[1];
	TxData[2] = DATA_CAN[2];
	TxData[3] = DATA_CAN[3];
	TxData[4] = DATA_CAN[4];
	TxData[5] = DATA_CAN[5];
	TxData[6] = DATA_CAN[6];
	TxData[7] = DATA_CAN[7];
	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
		Error_Handler();
	}
}
void CAN2_Transmit_manual(uint16_t ID_CAN, uint8_t DLC_CAN, uint8_t *DATA_CAN) {
	TxHeader.StdId = RxHeader.StdId;
	TxHeader.DLC = RxHeader.DLC;
	TxData[0] = DATA_CAN[0];
	TxData[1] = DATA_CAN[1];
	TxData[2] = DATA_CAN[2];
	TxData[3] = DATA_CAN[3];
	TxData[4] = DATA_CAN[4];
	TxData[5] = DATA_CAN[5];
	TxData[6] = DATA_CAN[6];
	TxData[7] = DATA_CAN[7];
	if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
		Error_Handler();
	}
}
void sendGyroData(int x, int y) {
	gyroHeader.StdId = 0x685;
	gyroHeader.DLC = 8;
	gyroData[0] = y;
	gyroData[1] = x;
	gyroData[3] = 0x00;
	gyroData[4] = 0x00;
	gyroData[5] = 0x00;
	gyroData[6] = 0x00;
	gyroData[7] = 0x00;

	HAL_StatusTypeDef res = HAL_CAN_AddTxMessage(&hcan1, &gyroHeader, gyroData,
			&TxMailbox);

	if (res != HAL_OK) {
		Error_Handler();
	}
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
	uint32_t er = HAL_CAN_GetError(hcan);
	const char trans_str[] = "can error";
}

void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
