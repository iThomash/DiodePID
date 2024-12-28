/*
 * eeprom.h
 *
 *  Created on: Dec 28, 2024
 *      Author: tomek
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"

#define EEPROM_ADDRESS 0x52

HAL_StatusTypeDef EEPROM_WriteIntArray(uint16_t memAddress, uint8_t *data, uint16_t size);
HAL_StatusTypeDef EEPROM_ReadIntArray(uint16_t memAddress, uint8_t *data, uint16_t size);
HAL_StatusTypeDef EEPROM_WriteFloat(uint16_t memAddress, float value);
HAL_StatusTypeDef EEPROM_ReadFloat(uint16_t memAddress, float *value);
HAL_StatusTypeDef EEPROM_WriteFloatArray(uint16_t memAddress, const float *values, size_t length);
HAL_StatusTypeDef EEPROM_ReadFloatArray(uint16_t memAddress, float *values, size_t length);

#endif /* INC_EEPROM_H_ */
