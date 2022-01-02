// Class to work with local memory

#include "stdio.h"
#include "stdbool.h"
#include "stm32f1xx_hal.h"
#include "eeprom.h"

bool isCorrectionAssign = false;
uint16_t xAddress = 0;
uint16_t yAddress = 1;

extern void saveGyroData(double x, double y);
extern double readXFromFlash();
extern double readYFromFlash();
double readFlash(uint16_t address);
void writeFlash(uint32_t address, double data);

void flashMemoryInit() {
	//initialize EEPROM
	EEPROM_Init();

	//set default value if variable not assigned
	EEPROM_Value value;
	for (uint16_t i = 0; i < EEPROM_VARIABLE_COUNT ; i++) {
		if (EEPROM_ReadVariable(i, &value) == EEPROM_NOT_ASSIGNED) {
			isCorrectionAssign = false;

			switch (i) {
			case 0:
				EEPROM_WriteVariable(i, (EEPROM_Value) (double) 0.00001,
						EEPROM_SIZE64);
				break;
			case 1:
				EEPROM_WriteVariable(i, (EEPROM_Value) (double) 0.00001,
						EEPROM_SIZE64);
				break;
			}
		} else {
			isCorrectionAssign = true;
		}
	}
}

void saveGyroData(double x, double y) {
	isCorrectionAssign = true;
	writeFlash(xAddress, x);
	writeFlash(yAddress, y);
}

double readXFromFlash() {
	return readFlash(xAddress);
}

double readYFromFlash() {
	return readFlash(yAddress);
}

void writeFlash(uint32_t address, double data) {
	EEPROM_Value value;
	value.Double = data;
	EEPROM_WriteVariable(address, value, EEPROM_SIZE64);
}

double readFlash(uint16_t address) {
	EEPROM_Value value;
	EEPROM_ReadVariable(address, &value);
	return value.Double;
}
