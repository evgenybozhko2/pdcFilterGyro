#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "button.h"

enum BUTTON num;

uint8_t BUTTON_STATE(uint8_t num) {
	switch (num) {

	case CALIBRATE_MPU_BUTTON:

		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_SET) {
			return 0;
		} else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET) {
			return 1;
		}
		break;
	}
}
