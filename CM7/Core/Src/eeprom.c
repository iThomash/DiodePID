/*
 * eeprom.c
 *
 *  Created on: Dec 28, 2024
 *      Author: tomek
 */

#include "eeprom.h"
#include "stm32h7xx_hal.h"
#include "i2c.h"

HAL_StatusTypeDef EEPROM_WriteIntArray(uint16_t memAddress, uint8_t *data, uint16_t size) {
    uint8_t buffer[size + 2];
    buffer[0] = (uint8_t)(memAddress >> 8);
    buffer[1] = (uint8_t)(memAddress & 0xFF);
    memcpy(&buffer[2], data, size);

    return HAL_I2C_Master_Transmit(&hi2c2, EEPROM_ADDRESS << 1, buffer, size + 2, HAL_MAX_DELAY);
}

HAL_StatusTypeDef EEPROM_ReadIntArray(uint16_t memAddress, uint8_t *data, uint16_t size) {
    uint8_t addressBuffer[2];
    addressBuffer[0] = (uint8_t)(memAddress >> 8);
    addressBuffer[1] = (uint8_t)(memAddress & 0xFF);

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c2, EEPROM_ADDRESS << 1, addressBuffer, 2, HAL_MAX_DELAY);
    if (status == HAL_OK) {
//        SendDebugMessage("{\"operation\": \"read\", \"message\": \"success\"}\r\n");
    	HAL_Delay(10); //10 ms delay out of black hole, otherwise it doesn't work
    } else {
        char buffer[50];
        snprintf(buffer, sizeof(buffer), "{\"operation\": \"read\", \"message\": \"success\", \"status\" = %d}\r\n", status);
    	HAL_Delay(10); //10 ms delay out of black hole, otherwise it doesn't work
//        SendDebugMessage(buffer);
    }
    return HAL_I2C_Master_Receive(&hi2c2, EEPROM_ADDRESS << 1, data, size, HAL_MAX_DELAY);
}

HAL_StatusTypeDef EEPROM_WriteFloat(uint16_t memAddress, float value) {
    uint8_t buffer[4];
    memcpy(buffer, &value, sizeof(float));
    return EEPROM_Write(memAddress, buffer, sizeof(buffer));
}

HAL_StatusTypeDef EEPROM_ReadFloat(uint16_t memAddress, float *value) {
    uint8_t buffer[4];
    HAL_StatusTypeDef status = EEPROM_Read(memAddress, buffer, sizeof(buffer));
    if (status == HAL_OK) {
        memcpy(value, buffer, sizeof(float));
    }
    return status;
}


// Functions for writing and reading an array of floats are not working properly
// The behavior needs further investigation
// The below functions use functions that could have been removed or changed!
HAL_StatusTypeDef EEPROM_WriteFloatArray(uint16_t memAddress, const float *values, size_t length) {
    if (values == NULL || length == 0) {
        return HAL_ERROR;
    }

    for (size_t i = 0; i < length; i++) {
        uint8_t buffer[sizeof(float)];
        memcpy(buffer, &values[i], sizeof(float));
        HAL_StatusTypeDef status = EEPROM_Write(memAddress + i * sizeof(float), buffer, sizeof(buffer));
        if (status != HAL_OK) {
            return status;
        }

        HAL_Delay(5);
    }
    return HAL_OK;
}

HAL_StatusTypeDef EEPROM_ReadFloatArray(uint16_t memAddress, float *values, size_t length) {
    if (values == NULL || length == 0) {
        return HAL_ERROR;
    }

    for (size_t i = 0; i < length; i++) {
        uint8_t buffer[sizeof(float)];
        HAL_StatusTypeDef status = EEPROM_Read(memAddress + i * sizeof(float), buffer, sizeof(buffer));
        if (status != HAL_OK) {
            return status;
        }
        memcpy(&values[i], buffer, sizeof(float));
    }
    return HAL_OK;
}
