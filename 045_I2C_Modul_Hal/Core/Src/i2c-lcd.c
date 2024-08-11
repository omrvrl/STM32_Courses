
/** Put this in the src folder **/

#include "i2c-lcd.h"
extern I2C_HandleTypeDef hi2c2;  // change your handler here accordingly

#define SLAVE_ADDRESS_LCD 0x4E // change this according to ur setup
uint8_t data_t[4];

void lcd_send_cmd (char cmd)
{
  char data_u, data_l;

	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0				// 0x8c    rs must equal to 0 for command register
	data_t[1] = data_u|0x08;  //en=0, rs=0				// 0x88	   rs must equal to 1 for data register
	data_t[2] = data_l|0x0C;  //en=1, rs=0				// 0xC	   en must equal to 1 to lock data to lcd (send to lcd)
	data_t[3] = data_l|0x08;  //en=0, rs=0

	HAL_I2C_Master_Transmit (&hi2c2, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=1
	data_t[1] = data_u|0x09;  //en=0, rs=1					// en address = 0x4 - rs address = 0x1     !!
	data_t[2] = data_l|0x0D;  //en=1, rs=1
	data_t[3] = data_l|0x09;  //en=0, rs=1
	HAL_I2C_Master_Transmit (&hi2c2, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_init (void)
{
	lcd_send_cmd (0x02);
	lcd_send_cmd (0x28);
	lcd_send_cmd (0x0c);
	lcd_send_cmd (0x80);
}

void lcd_send_string (char *str)
{
	while (*str)
		lcd_send_data (*str++);
}
