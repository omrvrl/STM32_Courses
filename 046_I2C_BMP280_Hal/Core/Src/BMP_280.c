/*
 * BMP_280.c
 *
 *  Created on: Aug 8, 2024
 *      Author: omrvr
 */


#include "BMP280.h"
#include "stdio.h"

int8_t a=0;
int i=0;
uint8_t mdata;

//compansated formula parameters
int32_t Temp;
BMP280_S32_t uT, uP;
//BMP280_U32_t var1, var2;
BMP280_S32_t tfine;

 // calibration parameters
unsigned short dig_t1;
signed short dig_t2 ;
signed short dig_t3;
unsigned short dig_P1;
signed short dig_P2;
signed short dig_P3;
signed short dig_P4;
signed short dig_P5;
signed short dig_P6;
signed short dig_P7;
signed short dig_P8;
signed short dig_P9;



void BMP280_Init(){
	Slave_Scan_Address();
	mdata = BMP280_Config();
	Temp = bmp280_compensate_T_double(Read_Adc_tRaw());
	BMP280_GetCallibrationValue();

}

void Slave_Scan_Address(){
	for(i=0;i <= 256; i++){
		if(HAL_I2C_IsDeviceReady(&hi2c1, i, 1, 100) == HAL_OK){
			break;
		}
	}
}

uint8_t BMP280_Config(void){
	uint8_t cdata = 0x03;								// normal mode selection
	HAL_I2C_Mem_Write(&hi2c1, BMP280_Write_SLAVE_ADDRESS, BMP280_Config_ADDR, 1, &cdata, 1, 1000);
	HAL_Delay(5);
	HAL_I2C_Mem_Read(&hi2c1, BMP280_Read_SLAVE_ADDRESS, BMP280_Config_ADDR, 1, &cdata,1, 1000);
	return cdata;
}


int32_t Read_Adc_tRaw(void){
	uint8_t uTdata[6];
	uint8_t sdata;
	while(HAL_I2C_Mem_Read(&hi2c1, BMP280_Read_SLAVE_ADDRESS, BMP280_STATUS_ADDR, 1, &sdata,1, 1000) != HAL_OK);				// wait until conversion is done

	HAL_I2C_Mem_Read(&hi2c1, BMP280_Read_SLAVE_ADDRESS, BMP280_TEMP_REG_ADDRESS, 1, uTdata, 6, 10000);

	uT = (uTdata[0] << 12 | uTdata[1] << 4 | uTdata[2] >> 4);
	uP = (uTdata[3] << 12 | uTdata[4] << 4 | uTdata[5] >> 4);

	return uT;
}

void BMP280_GetCallibrationValue(){
	uint8_t CallBuffer[BMP280_CALL_REGISTER_LENGTH] = {0};


	HAL_I2C_Mem_Read(&hi2c1, BMP280_Read_SLAVE_ADDRESS, BMP280_CALL_REGISTER_ADDRESS, 1, CallBuffer, BMP280_CALL_REGISTER_LENGTH, 1000);

	dig_t1 = (CallBuffer[a] << 8) | (CallBuffer[a+1]); a= a+2;
	dig_t2 = CallBuffer[a] << 8 | CallBuffer[a+1]; a= a+2;
	dig_t3 = CallBuffer[a] << 8 | CallBuffer[a+1]; a= a+2;
	dig_P1 = CallBuffer[a] << 8 | CallBuffer[a+1]; a= a+2;
	dig_P2 = CallBuffer[a] << 8 | CallBuffer[a+1]; a= a+2;
	dig_P3 = CallBuffer[a] << 8 | CallBuffer[a+1]; a= a+2;
	dig_P4 = CallBuffer[a] << 8 | CallBuffer[a+1]; a= a+2;
	dig_P5 = CallBuffer[a] << 8 | CallBuffer[a+1]; a= a+2;
	dig_P6 = CallBuffer[a] << 8 | CallBuffer[a+1]; a= a+2;
	dig_P7 = CallBuffer[a] << 8 | CallBuffer[a+1]; a= a+2;
	dig_P8 = CallBuffer[a] << 8 | CallBuffer[a+1]; a= a+2;
	dig_P9 = CallBuffer[a] << 8 | CallBuffer[a+1]; a= a+2;


	if(dig_t1 == 0x0000 || dig_t1 == 0xFFFF)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	if(dig_t2 == 0x0000 || dig_t2 == 0xFFFF)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	if(dig_t3 == 0x0000 || dig_t3 == 0xFFFF)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	if(dig_P1 == 0x0000 || dig_P1 == 0xFFFF)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	if(dig_P2 == 0x0000 || dig_P2 == 0xFFFF)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	if(dig_P3 == 0x0000 || dig_P3 == 0xFFFF)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	if(dig_P4 == 0x0000 || dig_P4 == 0xFFFF)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	if(dig_P5 == 0x0000 || dig_P5 == 0xFFFF)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	if(dig_P6 == 0x0000 || dig_P6 == 0xFFFF)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	if(dig_P7 == 0x0000 || dig_P7 == 0xFFFF)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	if(dig_P8 == 0x0000 || dig_P8 == 0xFFFF)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	if(dig_P9 == 0x0000 || dig_P9 == 0xFFFF)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);

}
																					// sıcaklık verisi hesaplamaları test edildi.
//int var1_1, var1_2, var1_3;
//
//void BMP280_GetUncompensatedValue(void){
//
//		uint8_t uTdata[3];
//		HAL_I2C_Mem_Read(&hi2c1, BMP280_Read_SLAVE_ADDRESS, BMP280_TEMP_REG_ADDRESS, 1, uTdata, 3, 10000);
//
//		uT = (uTdata[0] << 12 | uTdata[1] << 4 | uTdata[2] >> 4);
//
//		var1_1 = uT/16384;
//		var1_2 = dig_t1/1024;
//		var1_3 = dig_t1;
//		var1 = (var1_1 - var1_2)*var1_3;
//
//		part1 = (((double)uT)/131072 - ((double) dig_t1) / 8192);
//		part2 = dig_t3;
//
//		var2 = ( ((double)part1) * ((double) part1) *  ((double) dig_t3) );
//		tfine =  (var1 + var2);
//		Temp = (tfine / 5120);
//}

int32_t read_word_data(uint8_t reg_addr) {
    int16_t data[2];
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, BMP280_Write_SLAVE_ADDRESS, &reg_addr, 1, HAL_MAX_DELAY);
    if (HAL_OK != status) {
        return 0;
    }
    status = HAL_I2C_Master_Receive(&hi2c1, BMP280_Read_SLAVE_ADDRESS, data, 2, HAL_MAX_DELAY);
    if (HAL_OK != status) {
        return 0;
    }
    return (int32_t)((data[0] << 8) | data[1]);
}

BMP280_S32_t t_fine;
double bmp280_compensate_T_double(BMP280_S32_t adc_T)
{
	double var1, var2, T;
	dig_t1 = read_word_data(BMP280_REG_DIG_T1);
	dig_t2 = read_word_data(BMP280_REG_DIG_T2);
	dig_t3 = read_word_data(BMP280_REG_DIG_T3);

	var1 = (((double)adc_T)/16384.0 - ((double)dig_t1)/1024.0) * ((double)dig_t2);
	var2 = ((((double)adc_T)/131072.0 - ((double)dig_t1)/8192.0) * (((double)adc_T)/131072.0 - ((double) dig_t1)/8192.0)) * ((double)dig_t3);
	t_fine = (BMP280_S32_t)(var1 + var2);
	T = (var1 + var2) / 5120.0;
	return T;
}

