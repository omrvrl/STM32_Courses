/*
 * BMP280.h
 *
 *  Created on: Aug 6, 2024
 *      Author: omrvr
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_
#include "stm32f1xx_hal.h"
extern I2C_HandleTypeDef hi2c1;

#define BMP280_Write_SLAVE_ADDRESS 0xEC
#define BMP280_Read_SLAVE_ADDRESS 0xED

#define BMP280_CALL_REGISTER_ADDRESS 136
#define BMP280_CALL_REGISTER_LENGTH 24
#define BMP280_TEMP_REG_ADDRESS 0xFA
#define BMP280_S32_t long signed int
#define BMP280_U32_t long unsigned int
#define BMP280_REG_DIG_T1 0x88
#define BMP280_REG_DIG_T2 0x8A
#define BMP280_REG_DIG_T3 0x8C
#define BMP280_STATUS_ADDR 0xF3
#define BMP280_Config_ADDR 0xF4

void BMP280_Init(void);
void Slave_Scan_Address();
void BMP280_GetCallibrationValue(void);
//void BMP280_GetUncompensatedValue(void);
double bmp280_compensate_T_double(BMP280_S32_t adc_T);
int32_t  Read_Adc_tRaw(void);
uint8_t BMP280_Config(void);

#endif /* INC_BMP280_H_ */
