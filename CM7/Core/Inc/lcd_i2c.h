/*
 * lcd_i2c.h
 *
 *  Created on: Dec 25, 2024
 *      Author: tomek
 */

#ifndef INC_LCD_I2C_H_
#define INC_LCD_I2C_H_

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Pin mappings between PCF8574 and HD44780.
 *
 * PCF8574 pin mapping:
 * P7 -> D7, P6 -> D6, P5 -> D5, P4 -> D4, P3 -> Backlight (BL),
 * P2 -> Enable (EN), P1 -> Read/Write (RW), P0 -> Register Select (RS).
 */

/** I2C handle to use by default for LCD communication. */
#define HI2C_DEF hi2c2
/** Register Select (RS) pin mask. */
#define RS_PIN 0x01
/** Read/Write (RW) pin mask. */
#define RW_PIN 0x02
/** Enable (EN) pin mask. */
#define EN_PIN 0x04
/** Backlight (BL) pin mask. */
#define BL_PIN 0x08

/** Command for initializing the LCD in 4 or 8-bit mode. */
#define INIT_8_BIT_MODE	0x30
#define INIT_4_BIT_MODE	0x02

/** Command for clearing the LCD display. */
#define CLEAR_LCD	0x01

/** LCD display control commands. */
#define UNDERLINE_OFF_BLINK_OFF		0x0C /**< Underline off, blink off. */
#define UNDERLINE_OFF_BLINK_ON		0x0D /**< Underline off, blink on. */
#define UNDERLINE_ON_BLINK_OFF		0x0E /**< Underline on, blink off. */
#define UNDERLINE_ON_BLINK_ON		0x0F /**< Underline on, blink on. */

/** Address for the first character of line 1 and 2 on the LCD. */
#define FIRST_CHAR_LINE_1	0x80
#define FIRST_CHAR_LINE_2	0xC0

/**
 * @brief LCD display structure.
 *
 * Represents the configuration and state of an LCD display.
 */
struct lcd_disp {
	uint8_t addr; 		/**< I2C address of the LCD. */
	char f_line[17];	/**< First line of the display (16 characters max + null terminator). */
	char s_line[17];	/**< Second line of the display (16 characters max + null terminator). */
	bool bl;			/**< Backlight status (true for on, false for off). */
};

/**
 * @brief Initializes the LCD display.
 *
 * Sends initialization commands to the LCD via I2C and prepares it for operation.
 *
 * @param lcd Pointer to the lcd_disp structure representing the LCD configuration.
 */
void lcd_init(struct lcd_disp * lcd);

/**
 * @brief Writes data to the LCD via I2C.
 *
 * Sends a single byte of data or a command to the LCD using the specified pins.
 *
 * @param addr I2C address of the LCD.
 * @param data The data or command byte to be written.
 * @param xpin Pin configuration (RS, RW, EN).
 */
void lcd_write(uint8_t addr, uint8_t data, uint8_t xpin);

/**
 * @brief Updates the LCD display with the current content of the lcd_disp structure.
 *
 * Sends the contents of `f_line` and `s_line` to the LCD display.
 *
 * @param lcd Pointer to the lcd_disp structure containing the display data.
 */
void lcd_display(struct lcd_disp * lcd);

/**
 * @brief Clears the LCD display.
 *
 * Sends a clear command to the LCD and resets the content of the lcd_disp structure.
 *
 * @param lcd Pointer to the lcd_disp structure representing the LCD configuration.
 */
void lcd_clear(struct lcd_disp * lcd);

#endif /* INC_LCD_I2C_H_ */
