/**
 *	SPI library for STM32F4xx.
 *
 *	@author 	Tilen Majerle
 *	@email		tilen@majerle.eu
 *	@website	http://stm32f4-discovery.com
 *	@link		http://stm32f4-discovery.com/2014/04/library-05-spi-for-stm32f4xx/
 *	@version 	v1.3
 *	@ide		Keil uVision
 *	@license	GNU GPL v3
 *	
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
 *	
 *	Version 1.3
 *	 - 14.09.2014
 *	 - Added additional pins for SPI2
 *
 *	It support all 6 SPIs in master with 2Line Full Duplex mode
 *
 *	All six spis work the same principle by default.
 *	- Master
 *	- 8data bits
 *	- data sampled at first edge
 *	- prescaler 32
 *	- firstbit MSB
 *	- software SS pin configure
 *	- direction is full duplex 2 wire
 *	
 *	Pinout
 *				|PINS PACK 1			|PINS PACK 2			|PINS PACK 3
 *		SPIX	|MOSI	MISO	SCK		|MOSI	MISO	SCK		|MOSI	MISO	SCK
 *				|			
 *		SPI1	|PA7	PA6		PA5		|PB5	PB4		PB3		|
 *		SPI2	|PC3	PC2		PB10	|PB15	PB14	PB13	|PI3	PI2		PI0
 *		SPI3	|PB5	PB4		PB3		|PC12	PC11	PC10	|
 *		SPI4	|PE6	PE5		PE2		|PE14	PE13	PE12	|
 *		SPI5	|PF9	PF8		PF7		|PF11	PH7		PH6		|
 *		SPI6	|PG14	PG12	PG13	|
 *	
 *	Possible changes to each SPI. Set this defines in your defines.h file.
 *	
 *	Change x with 1-6, to match your SPI
 *	
 *	//Default prescaler
 *	#define TM_SPIx_PRESCALER	SPI_BaudRatePrescaler_32
 *	//Specify datasize
 *	#define TM_SPIx_DATASIZE 	SPI_DataSize_8b
 *	//Specify which bit is first
 *	#define TM_SPIx_FIRSTBIT 	SPI_FirstBit_MSB
 *	//Mode, master or slave
 *	#define TM_SPIx_MASTERSLAVE	SPI_Mode_Master
 *	//Specify mode of operation, clock polarity and clock phase
 *	#define TM_SPIx_MODE		TM_SPI_Mode_0
 *	
 */
#ifndef TM_SPI_H
#define TM_SPI_H 130
/**
 * Library dependencies
 * - STM32F4xx
 * - STM32F4xx RCC
 * - STM32F4xx GPIO
 * - STM32F4xx SPI
 * - defines.h
 */
/**
 * Includes
 */
#include "iNEMO_M1_SensorDrivers/Source/inc/iNEMO_SPI_Driver.h"


/**
 * Initialize each SPIx separately
 *
 */
/**
 * Initialize SPIx
 *
 * Parameters:
 * 	- SPI_TypeDef* SPIx:
 * 		SPI 1 - 2
 */
extern void TM_SPI_Init(SPI_TypeDef* SPIx);


extern uint8_t TM_SPI_Send(SPI_TypeDef* SPIx, uint8_t data);

/**
 * Send and receive multiple data bytes over SPI
 *
 * Parameters:
 * 	- SPI_TypeDef* SPIx: Select SPI which will operate with data
 * 	- uint8_t dataOut: pointer to data to be sent out
 *	- uint8_t dataIn: pointer to received data
 *	- uint8_t count: number of bytes to send
 *
 * No returns
 */
extern void TM_SPI_SendMulti(SPI_TypeDef* SPIx, uint8_t *dataOut, uint8_t *dataIn, uint16_t count);

/**
 * Write multiple data via SPI
 *
 * Parameters:
 * 	- SPI_TypeDef* SPIx: Select SPI which will operate with data
 * 	- uint8_t dataOut: pointer to data to be sent out
 *	- uint8_t count: number of bytes to send
 *
 * No returns
 */
extern void TM_SPI_WriteMulti(SPI_TypeDef* SPIx, uint8_t *dataOut, uint16_t count);

/**
 * Send and receive multiple data bytes over SPI
 *
 * Parameters:
 * 	- SPI_TypeDef* SPIx: Select SPI which will operate with data
 *	- uint8_t dataIn: pointer to received data
 *	- uint8_t dummy: dummy byte to be sent to SPI
 *	- uint8_t count: number of bytes to receive
 *
 * No returns
 */
extern void TM_SPI_ReadMulti(SPI_TypeDef* SPIx, uint8_t *dataIn, uint8_t dummy, uint16_t count);

#endif

