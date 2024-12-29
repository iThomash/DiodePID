/*
 * eeprom.c
 *
 *  Created on: Dec 28, 2024
 *      Author: tomek
 */

#include "eeprom.h"
#include "stm32h7xx_hal.h"
#include "i2c.h"

/**
 * @brief Writes an array of integers to the EEPROM.
 *
 * This function writes a block of data (integer array) to the EEPROM starting from the specified memory address.
 *
 * @param memAddress The starting memory address in the EEPROM to write the data.
 * @param data Pointer to the array of data to be written.
 * @param size The size of the data array (in bytes).
 *
 * @return HAL_StatusTypeDef The status of the EEPROM write operation (e.g., HAL_OK, HAL_ERROR).
 */
HAL_StatusTypeDef EEPROM_WriteIntArray(uint16_t memAddress, uint8_t *data, uint16_t size) {
    uint8_t buffer[size + 2];
    buffer[0] = (uint8_t)(memAddress >> 8);
    buffer[1] = (uint8_t)(memAddress & 0xFF);
    memcpy(&buffer[2], data, size);

    return HAL_I2C_Master_Transmit(&hi2c2, EEPROM_ADDRESS << 1, buffer, size + 2, HAL_MAX_DELAY);
}

/**
 * @brief Reads an array of integers from the EEPROM.
 *
 * This function reads a block of data (integer array) from the EEPROM starting from the specified memory address.
 *
 * @param memAddress The starting memory address in the EEPROM to read the data from.
 * @param data Pointer to the buffer where the read data will be stored.
 * @param size The size of the data array (in bytes).
 *
 * @return HAL_StatusTypeDef The status of the EEPROM read operation (e.g., HAL_OK, HAL_ERROR).
 */
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

/**
 * @brief Writes a floating-point value to the EEPROM.
 *
 * This function writes a single float value to the EEPROM at the specified memory address.
 *
 * @param memAddress The memory address in the EEPROM to write the float value.
 * @param value The float value to be written.
 *
 * @return HAL_StatusTypeDef The status of the EEPROM write operation (e.g., HAL_OK, HAL_ERROR).
 */
HAL_StatusTypeDef EEPROM_WriteFloat(uint16_t memAddress, float value) {
    uint8_t buffer[4];
    memcpy(buffer, &value, sizeof(float));
    return EEPROM_Write(memAddress, buffer, sizeof(buffer));
}

/**
 * @brief Reads a floating-point value from the EEPROM.
 *
 * This function reads a single float value from the EEPROM at the specified memory address.
 *
 * @param memAddress The memory address in the EEPROM to read the float value from.
 * @param value Pointer to the variable where the read float value will be stored.
 *
 * @return HAL_StatusTypeDef The status of the EEPROM read operation (e.g., HAL_OK, HAL_ERROR).
 */
HAL_StatusTypeDef EEPROM_ReadFloat(uint16_t memAddress, float *value) {
    uint8_t buffer[4];
    HAL_StatusTypeDef status = EEPROM_Read(memAddress, buffer, sizeof(buffer));
    if (status == HAL_OK) {
        memcpy(value, buffer, sizeof(float));
    }
    return status;
}


/**
 * @brief Writes an array of floating-point values to the EEPROM.
 *
 * This function attempts to write a block of float values to the EEPROM starting from the specified memory address.
 * However, it is currently known to not work properly.
 *
 * @param memAddress The starting memory address in the EEPROM to write the float values.
 * @param values Pointer to the array of float values to be written.
 * @param length The number of float values in the array.
 *
 * @return HAL_StatusTypeDef The status of the EEPROM write operation (e.g., HAL_OK, HAL_ERROR).
 *
 * @note This function is not working correctly and may fail to write the data properly.
 */
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

/**
 * @brief Reads an array of floating-point values from the EEPROM.
 *
 * This function attempts to read a block of float values from the EEPROM starting from the specified memory address.
 * However, it is currently known to not work properly.
 *
 * @param memAddress The starting memory address in the EEPROM to read the float values from.
 * @param values Pointer to the array where the read float values will be stored.
 * @param length The number of float values to read.
 *
 * @return HAL_StatusTypeDef The status of the EEPROM read operation (e.g., HAL_OK, HAL_ERROR).
 *
 * @note This function is not working correctly and may fail to read the data properly.
 */
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
