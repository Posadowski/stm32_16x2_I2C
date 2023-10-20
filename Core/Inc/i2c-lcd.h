#ifndef I2C_LCD_H
#define I2C_LCD_H
#include "stdint.h"
void lcd_init (void);   // initialize lcd

void lcd_send_cmd (char cmd);  // send command to the lcd

void lcd_send_data (char data);  // send data to the lcd

void lcd_send_string (char *str);  // send string to the lcd

void lcd_put_cur(uint8_t row, uint8_t col);  // put cursor at the entered position row (0 or 1), col (0-15);

void lcd_clear (void);

void lcd_display_ds18b20(float temperature);

void lcd_display_ds18b20_count(uint8_t ds18b20_sensor_count);
#endif // I2C_LCD_H
