#ifndef __I2C_LCD_H__
#define __I2C_LCD_H__

#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>

#define LCD_ADDR (0x27 << 1)  // Ð?a ch? I2C c?a PCF8574, shift 1 bit trái vì HAL dùng 8-bit address

extern I2C_HandleTypeDef hi2c1;

void lcd_init(void);
void lcd_send_cmd(char cmd);
void lcd_send_data(char data);
void lcd_send_string(char *str);
void lcd_clear(void);
void lcd_put_cur(int row, int col);
void lcd_send_string_pos(char *str, int row, int col);

#endif
