
/** Put this in the src folder **/

#include "i2c-lcd.h"
#include "main.h"
extern I2C_HandleTypeDef hi2c2;  // change your handler here accordingly

#define SLAVE_ADDRESS_LCD 0x4E // change this according to ur setup

void lcd_send_cmd (char cmd)
{
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c2, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=0
	data_t[1] = data_u|0x09;  //en=0, rs=0
	data_t[2] = data_l|0x0D;  //en=1, rs=0
	data_t[3] = data_l|0x09;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c2, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_clear (void)
{
	lcd_send_cmd (0x80);
	for (int i=0; i<70; i++)
	{
		lcd_send_data (' ');
	}
}

void lcd_put_cur(uint8_t row, uint8_t col)
{
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }

    lcd_send_cmd (col);
}


void lcd_init (void)
{
	// 4 bit initialisation
	HAL_Delay(50);  // wait for >40ms
	lcd_send_cmd (0x30);
	HAL_Delay(5);  // wait for >4.1ms
	lcd_send_cmd (0x30);
	HAL_Delay(1);  // wait for >100us
	lcd_send_cmd (0x30);
	HAL_Delay(10);
	lcd_send_cmd (0x20);  // 4bit mode
	HAL_Delay(10);

  // dislay initialisation
	lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	HAL_Delay(1);
	lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(1);
	lcd_send_cmd (0x01);  // clear display
	HAL_Delay(1);
	HAL_Delay(1);
	lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void lcd_send_string (char *str)
{
    int length = 0;
    //Calculate the length of the string
    while (*str) {
        length++;
        str++;
        // Check if the length exceeds 15 characters
        if (length > 15) {
      	    lcd_clear();
      	    lcd_put_cur(0,0);
        	lcd_send_string("string too long");
        	HAL_Delay(1000);
            return;
        }
    }
    str -= length;  // Restore the pointer to the beginning of the string
	while (*str) {
		lcd_send_data (*str++);
	}
}

void lcd_display_ds18b20(float temperature){
	uint8_t integer_part = (uint8_t)temperature;  // Get the integer part
	uint8_t  decimal_part = (uint8_t)((temperature - integer_part) * 100);  // Get the decimal part (multiplied by 100 to get two digits)
	char ds18b20_data_to_send[16];
	lcd_put_cur(1,0);
	memset(ds18b20_data_to_send, 0, sizeof(ds18b20_data_to_send));
	snprintf(ds18b20_data_to_send, sizeof(ds18b20_data_to_send), "temp: %u.%u", integer_part,decimal_part);
	lcd_send_string(ds18b20_data_to_send);
}

void lcd_display_ds18b20_count(uint8_t ds18b20_sensor_count){
	  lcd_clear();
	  lcd_put_cur(0,0);
	  char sensor_count_string[16];
	  snprintf(sensor_count_string, sizeof(sensor_count_string), "Detected %u",ds18b20_sensor_count);
	  lcd_send_string(sensor_count_string);

	  lcd_put_cur(1,0);
	  lcd_send_string("DS18B20 sensors");
}
