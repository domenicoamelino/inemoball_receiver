/**	
 * |----------------------------------------------------------------------
 * | Copyright (C) Tilen Majerle, 2014
 * | 
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |  
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * | 
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |----------------------------------------------------------------------
 */
#include "tm_inemo_spi.h"

void TM_SPI_Init(SPI_TypeDef* SPIx) {
	iNemoSPIInit(SPIx);
}

uint8_t TM_SPI_Send(SPI_TypeDef* SPIx, uint8_t data) {
	//Fill output buffer with data
	SPIx->DR = data;
	//Wait for transmission to complete
	while (!SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE));
	//Wait for received data to complete
	while (!SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE));
	//Wait for SPI to be ready
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY));
	//Return data from buffer
	return SPIx->DR;
}

void TM_SPI_SendMulti(SPI_TypeDef* SPIx, uint8_t *dataOut, uint8_t *dataIn, uint16_t count) {
	uint8_t i;
	for (i = 0; i < count; i++) {
		dataIn[i] = TM_SPI_Send(SPIx, dataOut[i]);
	}
}

void TM_SPI_WriteMulti(SPI_TypeDef* SPIx, uint8_t *dataOut, uint16_t count) {
	uint16_t i;
	for (i = 0; i < count; i++) {
		TM_SPI_Send(SPIx, dataOut[i]);
	}
}

void TM_SPI_ReadMulti(SPI_TypeDef* SPIx, uint8_t *dataIn, uint8_t dummy, uint16_t count) {
	uint16_t i;
	for (i = 0; i < count; i++) {
		dataIn[i] = TM_SPI_Send(SPIx, dummy);
	}
}

