// Class to work with local memory

#include <stdio.h>
#include "stm32f1xx_hal.h"

uint32_t xAddress = (uint32_t) 0x08008000;
uint32_t yAddress = (uint32_t) 0x09009000;

extern void saveX(double x);
extern void saveY(double y);
extern double readX();
extern double readY();
double readFlash(uint32_t address);
void writeFlash(uint32_t address, double data);

void saveX(double x) {
	writeFlash(xAddress, x);
}

void saveY(double y) {
	writeFlash(yAddress, y);
}

double readX() {
	return readFlash(xAddress);
}

double readY() {
	return readFlash(yAddress);
}

void writeFlash(uint32_t address, double data) {
	HAL_FLASH_Unlock();

	uint32_t pageError = 0;
	HAL_FLASHEx_Erase(address, pageError);

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data);

	HAL_FLASH_Lock();
}

double readFlash(uint32_t address) {
	return *(__IO uint32_t*) address;
}
