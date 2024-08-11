/*
 * Nokia5110.c
 *
 *  Created on: Aug 9, 2024
 *      Author: omrvr
 */

#include "Nokia5110.h"

uint8_t framebuffer[504];
uint8_t Bi;
uint16_t By;
uint8_t last;

bool Nokia_5110_Init(void){
	Nokia_5110_Reset();

	if(!Nokia_5110_Write(0x21, 0))
		return false;
	if(!Nokia_5110_Write(0x84, 0))
		return false;
	if(!Nokia_5110_Write(0x04, 0))		// TEMPRATURE CO = 0
		return false;
	if(!Nokia_5110_Write(0x13, 0))
		return false;
	if(!Nokia_5110_Write(0x20, 0))		// LCD BASIC COMMAND
		return false;
	if(!Nokia_5110_Write(0x0C, 0))		// DISPLAY_NORMAL
		return false;

	return true;


}


void Nokia_5110_Reset(void){

	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);

}

bool Nokia_5110_Write(uint8_t data, uint8_t mode){


	if(mode == 0){

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);							// DC PIN LOW for command send
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);								// CE PIN RESET (çip seçim pini low oldğunda bu cihaz seçili)
	}
	else if(mode == 1){

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);								// DC PIN HIGH for data send
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);								// CE PIN RESET
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);								// CE PIN HIGH
		return false;
	}

	HAL_SPI_Transmit_DMA(&hspi2, &data , 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);									// CE PIN HIGH

	return true;

}


void Nokia_5110_Clear(uint16_t pixel_index){
	if(pixel_index){
		framebuffer[pixel_index] = 0x00;
		return;
	}

	for(int i=0;i <= 504; i++){
		framebuffer[i] = 0x00;
	}
}


bool Nokia_5110_Update(void){

	// move to x = 0 pixel				imlec kaydırma
	if(!Nokia_5110_Write(0x80,0))
		return false;
	// move to y = 0 pixel
	if(!Nokia_5110_Write(0x40,0))
		return false;

	Nokia_5110_BufferWrite(framebuffer , 504);

	return true;

}

void Nokia_5110_BufferWrite(uint8_t *data, uint16_t length){

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);					// CE PIN LOW FOR SEND DATA
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);						// DC PIN HIGH FOR SEND COMAND
	HAL_SPI_Transmit_DMA(&hspi2, data, length);								// stm32 memory den spi slave cihazına veri gönder.

}

void NOkia5110_SetPixel(uint8_t x, uint8_t y, bool set){


	if (x < 0 || x >= 84 || y < 0 || y >= 84)
		return;

	By = (y/8) * 84 + x;
	Bi = y % 8;

//	if(Bi == 8)
//		Bi = Bi -1;


	if(set){
		framebuffer[By] |= (1 << Bi);
	}
	else{
		framebuffer[By] &= ~(1 << Bi);
	}

}
