#include "i2c-lcd.h"
#include "stm32f1xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

static void lcd_send(uint8_t data, uint8_t mode);

#define LCD_BACKLIGHT 0x08
#define ENABLE 0x04
#define RW 0x00
#define RS 0x01

void lcd_send_cmd(char cmd) {
    lcd_send(cmd, 0);
}

void lcd_send_data(char data) {
    lcd_send(data, 1);
}

static void lcd_send(uint8_t data, uint8_t mode) {
    uint8_t high = data & 0xF0;
    uint8_t low = (data << 4) & 0xF0;
    uint8_t data_arr[4];

    data_arr[0] = high | LCD_BACKLIGHT | mode | ENABLE;
    data_arr[1] = high | LCD_BACKLIGHT | mode;
    data_arr[2] = low | LCD_BACKLIGHT | mode | ENABLE;
    data_arr[3] = low | LCD_BACKLIGHT | mode;

    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, data_arr, 4, 100);
    HAL_Delay(1);
}

void lcd_init(void) {
    HAL_Delay(50);
    lcd_send_cmd(0x30);
    HAL_Delay(5);
    lcd_send_cmd(0x30);
    HAL_Delay(1);
    lcd_send_cmd(0x30);
    HAL_Delay(10);
    lcd_send_cmd(0x20); // 4-bit mode

    lcd_send_cmd(0x28); // 2 lines, 5x8 font
    lcd_send_cmd(0x08); // display off
    lcd_send_cmd(0x01); // clear
    HAL_Delay(2);
    lcd_send_cmd(0x06); // entry mode
    lcd_send_cmd(0x0C); // display on, cursor off
}

void lcd_send_string(char *str) {
    while (*str) {
        lcd_send_data(*str++);
    }
}

void lcd_send_string_pos(char *str, int row, int col) {
    lcd_put_cur(row, col);
    lcd_send_string(str);
}

void lcd_clear(void) {
    lcd_send_cmd(0x01);
    HAL_Delay(2);
}

void lcd_put_cur(int row, int col) {
    uint8_t pos;
    pos = (row == 0) ? (0x80 + col) : (0xC0 + col);
    lcd_send_cmd(pos);
}
