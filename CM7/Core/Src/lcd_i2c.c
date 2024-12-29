/*
 * lcd_i2c.c
 *
 *  Created on: Dec 25, 2024
 *      Author: tomek
 */

#include "lcd_i2c.h"
#include "stm32h7xx_hal.h"
#include "i2c.h"


/**
 * @brief Initializes the LCD display.
 *
 * Sends initialization commands to the LCD via I2C and prepares it for operation.
 *
 * @param lcd Pointer to the lcd_disp structure representing the LCD configuration.
 */
void lcd_init(struct lcd_disp * lcd)
{
	uint8_t xpin = 0;
	/* set backlight */
	if(lcd->bl)
	{
		xpin = BL_PIN;
	}

	/* init sequence */
	HAL_Delay(40);
	lcd_write(lcd->addr, INIT_8_BIT_MODE, xpin);
	HAL_Delay(5);
	lcd_write(lcd->addr, INIT_8_BIT_MODE, xpin);
	HAL_Delay(1);
	lcd_write(lcd->addr, INIT_8_BIT_MODE, xpin);

	/* set 4-bit mode */
	lcd_write(lcd->addr, INIT_4_BIT_MODE, xpin);

	/* set cursor mode */
	lcd_write(lcd->addr, UNDERLINE_OFF_BLINK_OFF, xpin);

	/* clear */
	lcd_clear(lcd);

}

/**
 * @brief Writes data to the LCD via I2C.
 *
 * Sends a single byte of data or a command to the LCD using the specified pins.
 *
 * @param addr I2C address of the LCD.
 * @param data The data or command byte to be written.
 * @param xpin Pin configuration (RS, RW, EN).
 */
void lcd_write(uint8_t addr, uint8_t data, uint8_t xpin)
{
	uint8_t tx_data[4];

	/* split data */
	tx_data[0] = (data & 0xF0) | EN_PIN | xpin;
	tx_data[1] = (data & 0xF0) | xpin;
	tx_data[2] = (data << 4) | EN_PIN | xpin;
	tx_data[3] = (data << 4) | xpin;

	/* send data via i2c */
	HAL_I2C_Master_Transmit(&HI2C_DEF, addr, tx_data, 4, 100);

	HAL_Delay(5);
}

/**
 * @brief Updates the LCD display with the current content of the lcd_disp structure.
 *
 * Sends the contents of `f_line` and `s_line` to the LCD display.
 *
 * @param lcd Pointer to the lcd_disp structure containing the display data.
 */
void lcd_display(struct lcd_disp * lcd)
{
	uint8_t xpin = 0, i = 0;

	/* set backlight */
	if(lcd->bl)
	{
		xpin = BL_PIN;
	}

	lcd_clear(lcd);

	/* send first line data */
	lcd_write(lcd->addr, FIRST_CHAR_LINE_1, xpin);
	while(lcd->f_line[i])
	{
		lcd_write(lcd->addr, lcd->f_line[i], (xpin | RS_PIN));
		i++;
	}

	/* send second line data */
	i = 0;
	lcd_write(lcd->addr, FIRST_CHAR_LINE_2, xpin);
	while(lcd->s_line[i])
	{
		lcd_write(lcd->addr, lcd->s_line[i], (xpin | RS_PIN));
		i++;
	}
}

/**
 * @brief Clears the LCD display.
 *
 * Sends a clear command to the LCD and resets the content of the lcd_disp structure.
 *
 * @param lcd Pointer to the lcd_disp structure representing the LCD configuration.
 */
void lcd_clear(struct lcd_disp * lcd)
{
	uint8_t xpin = 0;

	/* set backlight */
	if(lcd->bl)
	{
		xpin = BL_PIN;
	}

	/* clear display */
	lcd_write(lcd->addr, CLEAR_LCD, xpin);
}
