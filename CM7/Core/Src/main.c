/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "lcd_i2c.h"
#include "string.h"
#include "eeprom.h"
#include "jsmn.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//LCD struct for handling display data
struct lcd_disp disp;

// Variables that are changed during the process
// max_lux_value and min_lux_value are responsible for limiting the lux value reachable by the circuit
// set_value and pwm_width are describing the set value for pwm width or desired lux value.
// Dominating value is defined by "priority" variable.
uint16_t max_lux_value = 10000;
uint16_t min_lux_value = 0;
uint16_t set_value = 100;  // LUX
uint16_t pwm_width = 100;  // PERCENT

//Priority explaination
// 0 - Prioritize to change the pwm width to meet the "set_value" value
// 1 - Set the PWM width value, to read the lux value, set by defualt
volatile uint8_t priority = 1;

//Single click detection variable
volatile bool userButtonPressed = false;

// Buffer and variables needed for USART communication
uint8_t rx_buffer[50];
uint8_t rx_data;
uint8_t rx_index = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief Sends a debug message over UART. Can be used as a medium for communication
 * with other device. Allows sending strings or even a JSON.
 *
 * @param message The message string to send.
 */
void SendDebugMessage(const char* message) {
    HAL_UART_Transmit(&huart3, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
}

/**
 * @brief Sends an integer as a debug message.
 *
 * Converts an integer to a string and sends it using the SendDebugMessage function.
 *
 * @param value The integer value to send.
 */
void SendDebugInt(int value) {
    char buffer[20];
    snprintf(buffer, sizeof(buffer), "%d", value);
    SendDebugMessage(buffer);
}

/**
 * @brief Sends a float as a debug message.
 *
 * Converts a float to a string with two decimal precision and sends it using the SendDebugMessage function.
 *
 * @param value The float value to send.
 */
void SendDebugFloat(float value) {
    char buffer[20];
    snprintf(buffer, sizeof(buffer), "%.2f", value);
    SendDebugMessage(buffer);
}

/**
 * @brief UART receive complete callback.
 *
 * This function is called when a UART receive operation completes. It processes
 * incoming data character by character and triggers JSON parsing when a complete
 * JSON message is received.
 *
 * @param huart The UART handle.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (rx_data != '}') {
        rx_buffer[rx_index++] = rx_data;
    } else {
        rx_buffer[rx_index++] = rx_data;
        rx_buffer[rx_index] = '\0';
        rx_index = 0;
        ParseJson((char*)rx_buffer);
    }
    HAL_UART_Receive_IT(&huart3, &rx_data, 1);
}

/**
 * @brief GPIO external interrupt callback.
 *
 * This function is triggered when an external interrupt is generated on a specific GPIO pin.
 * It detects user button presses and sets a flag.
 *
 * @param GPIO_Pin The pin number that triggered the interrupt.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == USER_BUTTON_Pin) {
		userButtonPressed = true;
	}
}

/**
 * @brief Measures the light intensity in lux.
 *
 * This function starts an ADC conversion, calculates the voltage from the ADC value,
 * computes the resistance of the light sensor, and calculates the lux value using a formula.
 *
 * @return The calculated lux value as a float.
 */
float measureLux() {
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,20);
	uint16_t v = HAL_ADC_GetValue(&hadc1);

	float voltage = 3.3 * v / 65535;
	float resistance = (( voltage)/(3.3 - voltage) * 4700);
	float lux = (10 * pow(8000, 1/0.6)) / pow(resistance, 1/0.6);
	return lux;
}

/**
 * @brief Compares a JSON key to a given string.
 *
 * This utility function checks if a JSON key matches a specified string.
 *
 * @param json The JSON string.
 * @param token The JSON token to compare.
 * @param s The string to compare against.
 *
 * @return 0 if the JSON key matches the string, 1 otherwise.
 */
int jsoneq(const char *json, jsmntok_t *token, const char *s) {
    if (token->type != JSMN_STRING) {
        return 1;
    }
    int token_length = token->end - token->start;
    return (strlen(s) == token_length) && (strncmp(json + token->start, s, token_length) == 0);
}

/**
 * @brief Parses a JSON string and processes specific keys.
 *
 * This function parses a JSON string using the JSMN library, identifies specific keys,
 * and updates corresponding variables based on their values.
 *
 * Recognized keys:
 * - "LED": Sets the LED state (integer).
 * - "PWM": Sets the PWM width (integer).
 * - "PRIORITY": Sets the priority level (integer).
 *
 * Sends a success message as a JSON response after processing.
 *
 * @param json The JSON string to parse.
 */
void ParseJson(const char* json) {
    jsmn_parser parser;
    jsmntok_t tokens[20];
    jsmn_init(&parser);

    int r = jsmn_parse(&parser, json, strlen(json), tokens, sizeof(tokens) / sizeof(tokens[0]));
    if (r < 0) { return; }

    for (int i = 1; i < r; i++) {
        if (tokens[i].type == JSMN_STRING) {
            char key[10];
            snprintf(key, sizeof(key), "%.*s", tokens[i].end - tokens[i].start, json + tokens[i].start);
            if (i + 1 < r) {
            	if (tokens[i + 1].type == JSMN_PRIMITIVE) {
            		char value[10];
					snprintf(value, sizeof(value), "%.*s", tokens[i + 1].end - tokens[i + 1].start, json + tokens[i + 1].start);
					char *endptr;
					int num_value = (int) strtol(value, &endptr, 10);
					if (strcmp(key, "LED") == 0) {
						if (*endptr == '\0') {
							set_value = num_value;
						}
					} else if (strcmp(key, "PWM") == 0) {
						if (*endptr == '\0' ) {
							pwm_width = num_value;
						}
					} else if (strcmp(key, "PRIORITY") == 0) {
						if (*endptr == '\0' ) {
							priority = num_value;
						}
					}
				}
            }
            i++;
        }
    }

	SendDebugMessage("{\"success\": 1, \"message\": \"Success!\"}");
}

// Variables required for PID controller
// prev_value is used to store the previous error value
int prev_value = 0;

// Controller settings, selected on the basis of the experiments
float kp = 1, ki = 0.3, kd = 0;

// Variable for storing data related to the integrating part of PID
float u_i = 0;

/**
 * @brief Saturates a value within a specified range.
 *
 * Ensures the input value remains within the range of 1 to 999.
 *
 * @param value The input value to be saturated.
 *
 * @return The saturated value within the range [1, 999].
 */
int saturate(uint16_t value) {
	if (value > 999) {
		value = 999;
	}
	if (value < 1) {
		value = 1;
	}
	return value;
}

/**
 * @brief Performs PID control calculation for light intensity adjustment.
 *
 * Computes the PID control output(PWM width) based on the current and desired lux values.
 * - Proportional term: Adjusts the output based on the current error.
 * - Integral term: Accumulates the error over time.
 * - Derivative term: Predicts the future trend of the error.
 *
 * The computed output is then saturated to ensure it remains within valid bounds.
 *
 * @param current_lux The current measured lux value.
 * @param set_lux_value The desired lux value to achieve.
 *
 * @return The PID control output value, saturated within the range [1, 999].
 */
uint16_t PID_calc(uint16_t current_lux, uint16_t set_lux_value) {
    int e = set_lux_value - current_lux;
    float u_k = kp * (float)e;
    u_i = u_i + (int)(ki * (float)e);
    float u_d = kd * ((float)(e - prev_value) / 2.0);
    int u_pid = (int)(u_k + u_i + u_d);
    prev_value = e;
    return saturate(u_pid);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  //LCD initialization
	disp.addr = (0x27 << 1);
	disp.bl = true;
	lcd_init(&disp);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//Starting the required services
	TIM2->CCR1 = 100;
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_UART_Receive_IT(&huart3, &rx_data, 3);
	//Sending single informative command
	char buffer[100];
	ParseJson("{\"LED\": 100, \"PWM\": 100, \"PRIORITY\": 1}\r\n");
  while (1)
  {
	// Measure lux in the area
	float lux = measureLux();

	//LCD handler
	sprintf((char *)disp.f_line, "Current: %.2f", lux);
	sprintf((char *)disp.s_line, "Set value: %d", set_value);
	lcd_display(&disp);

	// Change PWM width with respect to the priority
	// 0 - Prioritize to change the pwm width to meet the "set_value" value - Yes, PID
	// 1 - Set the PWM width value, to read the lux value, set by defualt
	if (priority == 1) {
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_width);
	} else {
		uint16_t counter = PID_calc(lux, set_value);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, counter);
	}

	//Sending current data to desktop app for visualization
	char buffer[100];
	snprintf(buffer, sizeof(buffer), "{\"operation\": \"data\", \"data\": %.2f}\r\n", lux);
	SendDebugMessage(buffer);

	HAL_Delay(500);

	// User Button click handler
	// This function is called once after clicking the button, and performs
	// the configuration while stopping all other things and then resuming after it's done
	if (userButtonPressed) {
		userButtonPressed = false;
		uint32_t backup_value = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1);
		//Measuring lowest and highest possible lux value at output
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
		HAL_Delay(500);
		uint16_t low_lux = measureLux();
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 999);
		HAL_Delay(500);
		uint16_t high_lux = measureLux();

		//YES I COULD HAVE CHANGED THE VALUES OF HIGH AND LOW LUX HERE

		// Writing the acquired data to EEPROM
		uint16_t measures[2] = {low_lux, high_lux};
		if (EEPROM_WriteIntArray(0x0000, measures, sizeof(measures)) == HAL_OK) {
			SendDebugMessage("{\"operation\": \"write\", \"message\": \"success\"}\r\n");
		} else {
			SendDebugMessage("{\"message\": \"failure\"}\r\n");
		}
		HAL_Delay(10);

		//Reading the data from EEPROM
		uint16_t readMeasurements[2] = {0,0};
		if (EEPROM_ReadIntArray(0x0000, &readMeasurements, sizeof(readMeasurements)) == HAL_OK) {
			char buffer[100];
			snprintf(buffer, sizeof(buffer), "{\"operation\": \"read\", \"message\": \"success\", \"low_lux\": %d, \"high_lux\": %d}\r\n", low_lux, high_lux);
			// HERE WE READ THE MIN AND MAX VALUE FOR LED C:
			min_lux_value = readMeasurements[0];
			max_lux_value = readMeasurements[1];
			SendDebugMessage(buffer);
		} else {
			SendDebugMessage("{\"operation\": \"read\", \"message\": \"failure\"}\r\n");
		}
		//Restoring the previous value
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, backup_value);
		HAL_Delay(300);

	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
