/*
 * Nokia5110.h
 *
 *  Created on: Aug 9, 2024
 *      Author: omrvr
 */

#ifndef INC_NOKIA5110_H_
#define INC_NOKIA5110_H_

#include "stm32f1xx_hal.h"
#include "stdbool.h"

extern SPI_HandleTypeDef hspi2;
extern DMA_HandleTypeDef hdma_spi2_tx;


bool Nokia_5110_Init(void);
void Nokia_5110_Reset(void);
void Nokia_5110_Clear(uint16_t frame_index);
bool Nokia_5110_Update(void);
void NOkia5110_SetPixel(uint8_t x, uint8_t y, bool set);
bool Nokia_5110_Write(uint8_t data, uint8_t mode);
void Nokia_5110_BufferWrite(uint8_t *data, uint16_t length);



#endif /* INC_NOKIA5110_H_ */
