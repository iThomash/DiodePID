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


// EEPROM address, A0 - low, A1 - high, A2 - low
// 0b1010010 - 0x52
#define EEPROM_ADDRESS 0x52

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
HAL_StatusTypeDef EEPROM_WriteIntArray(uint16_t memAddress, uint8_t *data, uint16_t size);

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
HAL_StatusTypeDef EEPROM_ReadIntArray(uint16_t memAddress, uint8_t *data, uint16_t size);

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
HAL_StatusTypeDef EEPROM_WriteFloat(uint16_t memAddress, float value);

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
HAL_StatusTypeDef EEPROM_ReadFloat(uint16_t memAddress, float *value);

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
HAL_StatusTypeDef EEPROM_WriteFloatArray(uint16_t memAddress, const float *values, size_t length);

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
HAL_StatusTypeDef EEPROM_ReadFloatArray(uint16_t memAddress, float *values, size_t length);

#endif /* INC_EEPROM_H_ */
