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
#include "tm_inemo_nrf24l01.h"
#include "iNEMO_M1_SensorDrivers/Source/inc/iNEMO_SPI_Driver.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "iNemo_Led/iNemoLed.h"
TM_NRF24L01_t TM_NRF24L01_Struct;

//Definizione Variabili
extern uint8_t dataRec [32]; // al momento è da definire nel main
uint8_t ricBuffer[62];
uint8_t rxMutex = 0x00;


uint8_t great_packet_lenght = 0;
uint8_t packet_lenght = 0;

uint8_t TM_NRF24L01_ReceivePacket(uint8_t *data, uint8_t * length)
{
	uint8_t j;

	//Ripulire il buffer ad ogni ricezione. TODO - Allocare questo buffer in fase di inizializzazione del driver.
	for(j = 0; j <= 62; j++)
		ricBuffer[j] = 0;

	rxMutex = 0;
	printf("Blocco il mutex e attendo\n"); //da eliminare.
	while(rxMutex == 0);
	printf("Mutex Sbloccato  %d \n ", rxMutex);

	if(rxMutex == 0x01){
		*length = packet_lenght;
		for(j = 0; j <= packet_lenght; j++)	data[j] = ricBuffer[j];
		printf("Pacchetto Ricevuto \n");
			//printf("Dato Ricevuto %d: %x\n", j, ricBuffer[j]);
			return 1; // Esecuzione Corretta.
	}
	else if(rxMutex == 0x02){
		*length = great_packet_lenght-1;
		for(j = 0; j <= great_packet_lenght; j++)	data[j] = ricBuffer[j];
		printf("Pacchetto Ricevuto \n");
		return 1; //Esecuzione Corretta.
			//printf("Dato Ricevuto %d: %x\n", j, ricBuffer[j]);
	}
	return 0;
}

void TM_NRF24L01_InitPins(void) {
	GPIO_InitTypeDef GPIO_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	
	RCC_APB2PeriphClockCmd(NRF24L01_CSN_RCC | NRF24L01_CE_RCC | NRF24L01_IRQ_RCC, ENABLE);
	
	//Common settings
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	
	//CSN pins
	GPIO_InitStruct.GPIO_Pin = NRF24L01_CSN_PIN;
	GPIO_Init(NRF24L01_CSN_PORT, &GPIO_InitStruct);
	//CE pins
	GPIO_InitStruct.GPIO_Pin = NRF24L01_CE_PIN;
	GPIO_Init(NRF24L01_CE_PORT, &GPIO_InitStruct);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	  GPIO_InitTypeDef GPIO_InitStructure;
	  NVIC_InitTypeDef NVIC_InitStructure;
	  EXTI_InitTypeDef EXTI_InitStructure;

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);

	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource9);

	  EXTI_InitStructure.EXTI_Line = EXTI_Line9;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;

	  EXTI_Init(&EXTI_InitStructure);

	  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
	/*
	//IRQ Pin
	GPIO_InitStruct.GPIO_Pin = NRF24L01_IRQ_PIN;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_Init(NRF24L01_IRQ_PORT,&GPIO_InitStruct);
	//Attach NVIC & EXTI
    	GPIO_EXTILineConfig(NRF24L01_IRQ_PORT_SOURCE,NRF24L01_IRQ_PIN_SOURCE);

    	//Configuring EXTI
    	EXTI_InitStruct.EXTI_Line = NRF24L01_IRQ_EXTI_LINE;
    	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
    	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    	EXTI_Init(&EXTI_InitStruct);

    	//Configuring NVICO
    	NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;
    	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x0F;
    	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x0F;
    	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    	NVIC_Init(&NVIC_InitStruct);

    	*/


	NRF24L01_CE_LOW;
	NRF24L01_CSN_HIGH;
}

uint8_t TM_NRF24L01_Init(uint8_t channel, uint8_t payload_size) {
	//Initialize CE and CSN pins
	TM_NRF24L01_InitPins();
	//Initialize SPI
	TM_SPI_Init(NRF24L01_SPI);
	
	//Max payload is 32bytes
	if (payload_size > 32) {
		payload_size = 32;
	}
	
	TM_NRF24L01_Struct.Channel = channel;
	TM_NRF24L01_Struct.PayloadSize = payload_size;
	TM_NRF24L01_Struct.OutPwr = TM_NRF24L01_OutputPower_0dBm;
	TM_NRF24L01_Struct.DataRate = TM_NRF24L01_DataRate_2M;
	
	//Reset nRF24L01+ to power on registers values
	TM_NRF24L01_SoftwareReset();
	
	//Channel select
	TM_NRF24L01_SetChannel(TM_NRF24L01_Struct.Channel);
	
	//Set pipeline to max possible 32 bytes
	TM_NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P0, TM_NRF24L01_Struct.PayloadSize); // Auto-ACK pipe
	TM_NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P1, TM_NRF24L01_Struct.PayloadSize); // Data payload pipe
	TM_NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P2, TM_NRF24L01_Struct.PayloadSize);
	TM_NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P3, TM_NRF24L01_Struct.PayloadSize);
	TM_NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P4, TM_NRF24L01_Struct.PayloadSize);
	TM_NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P5, TM_NRF24L01_Struct.PayloadSize);
	
	//Set RF settings (2mbps, output power)
	TM_NRF24L01_SetRF(TM_NRF24L01_Struct.DataRate, TM_NRF24L01_Struct.OutPwr);
	
	//Config register
	TM_NRF24L01_WriteRegister(NRF24L01_REG_CONFIG, NRF24L01_CONFIG);
	
	//Enable auto-acknowledgment for all pipes
	TM_NRF24L01_WriteRegister(NRF24L01_REG_EN_AA, 0x3F);
	
	//Enable RX addresses
	TM_NRF24L01_WriteRegister(NRF24L01_REG_EN_RXADDR, 0x3F);

	//Auto retransmit delay: 1000 (4x250) us and Up to 15 retransmit trials
	TM_NRF24L01_WriteRegister(NRF24L01_REG_SETUP_RETR, 0x4F);
	
	//Dynamic length configurations: dynamic length
	TM_NRF24L01_WriteRegister(NRF24L01_REG_DYNPD, 0x3F);
	
	TM_NRF24L01_WriteRegister(NRF24L01_REG_FEATURE,0x04);

	//Clear FIFOs
	NRF24L01_FLUSH_TX;
	NRF24L01_FLUSH_RX;
	
	//Go to RX mode
	TM_NRF24L01_PowerUpRx();
	
	return 1;
}

void TM_NRF24L01_SetMyAddress(uint8_t *adr) {
	NRF24L01_CE_LOW;
	TM_NRF24L01_WriteRegisterMulti(NRF24L01_REG_RX_ADDR_P1, adr, 5);
	NRF24L01_CE_HIGH;
}

void TM_NRF24L01_SetTxAddress(uint8_t *adr) {
	TM_NRF24L01_WriteRegisterMulti(NRF24L01_REG_RX_ADDR_P0, adr, 5);
	TM_NRF24L01_WriteRegisterMulti(NRF24L01_REG_TX_ADDR, adr, 5);
}

void TM_NRF24L01_SetPipe2Address(uint8_t adr) {
	TM_NRF24L01_WriteRegister(NRF24L01_REG_RX_ADDR_P2, adr);
}

void TM_NRF24L01_SetPipe3Address(uint8_t adr) {
	TM_NRF24L01_WriteRegister(NRF24L01_REG_RX_ADDR_P3, adr);
}

void TM_NRF24L01_SetPipe4Address(uint8_t adr) {
	TM_NRF24L01_WriteRegister(NRF24L01_REG_RX_ADDR_P4, adr);
}

void TM_NRF24L01_SetPipe5Address(uint8_t adr) {
	TM_NRF24L01_WriteRegister(NRF24L01_REG_RX_ADDR_P5, adr);
}

void TM_NRF24L01_WriteBit(uint8_t reg, uint8_t bit, BitAction value) {
	uint8_t tmp;
	tmp = TM_NRF24L01_ReadRegister(reg);
	if (value != Bit_RESET) {
		tmp |= 1 << bit;
	} else {
		tmp &= ~(1 << bit);
	}
	TM_NRF24L01_WriteRegister(reg, tmp);
}

uint8_t TM_NRF24L01_ReadBit(uint8_t reg, uint8_t bit) {
	uint8_t tmp;
	tmp = TM_NRF24L01_ReadRegister(reg);
	if (!NRF24L01_CHECK_BIT(tmp, bit)) {
		return 0;
	}
	return 1;
}

uint8_t TM_NRF24L01_ReadRegister(uint8_t reg) {
	uint8_t value;
	NRF24L01_CSN_LOW;
	TM_SPI_Send(NRF24L01_SPI, NRF24L01_READ_REGISTER_MASK(reg));
	value = TM_SPI_Send(NRF24L01_SPI, NRF24L01_NOP_MASK);
	NRF24L01_CSN_HIGH;
	
	return value;
}

void TM_NRF24L01_ReadRegisterMulti(uint8_t reg, uint8_t* data, uint8_t count) {
	NRF24L01_CSN_LOW;
	TM_SPI_Send(NRF24L01_SPI, NRF24L01_READ_REGISTER_MASK(reg));
	TM_SPI_ReadMulti(NRF24L01_SPI, data, NRF24L01_NOP_MASK, count);
	NRF24L01_CSN_HIGH;
}

void TM_NRF24L01_WriteRegister(uint8_t reg, uint8_t value) {
	NRF24L01_CSN_LOW;
	TM_SPI_Send(NRF24L01_SPI, NRF24L01_WRITE_REGISTER_MASK(reg));
	TM_SPI_Send(NRF24L01_SPI, value);
	NRF24L01_CSN_HIGH;
}

void TM_NRF24L01_WriteRegisterMulti(uint8_t reg, uint8_t *data, uint8_t count) {
	NRF24L01_CSN_LOW;
	TM_SPI_Send(NRF24L01_SPI, NRF24L01_WRITE_REGISTER_MASK(reg));
	TM_SPI_WriteMulti(NRF24L01_SPI, data, count);
	NRF24L01_CSN_HIGH;
}

void TM_NRF24L01_PowerUpTx(void) {
	NRF24L01_CLEAR_INTERRUPTS;
	TM_NRF24L01_WriteRegister(NRF24L01_REG_CONFIG, NRF24L01_CONFIG | (0 << NRF24L01_PRIM_RX) | (1 << NRF24L01_PWR_UP));
}

void TM_NRF24L01_PowerUpRx(void) {
	NRF24L01_FLUSH_RX;
	
	NRF24L01_CLEAR_INTERRUPTS;
	
	NRF24L01_CE_LOW;

	TM_NRF24L01_WriteRegister(NRF24L01_REG_CONFIG, NRF24L01_CONFIG | 1 << NRF24L01_PWR_UP | 1 << NRF24L01_PRIM_RX);
	
	NRF24L01_CE_HIGH;
}

void TM_NRF24L01_PowerDown(void) {
	NRF24L01_CE_LOW;
	TM_NRF24L01_WriteBit(NRF24L01_REG_CONFIG, NRF24L01_PWR_UP, Bit_RESET);
}

void TM_NRF24L01_Transmit(uint8_t *data) {
	uint8_t count = TM_NRF24L01_Struct.PayloadSize;

	//Chip enable put to low, disable it
	NRF24L01_CE_LOW;
	
	//Go to power up tx mode
	TM_NRF24L01_PowerUpTx();
	
	//Clear TX FIFO from NRF24L01+
	NRF24L01_FLUSH_TX;
	
	//Send payload to nRF24L01+
	NRF24L01_CSN_LOW;
	//Send write payload command
	TM_SPI_Send(NRF24L01_SPI, NRF24L01_W_TX_PAYLOAD_MASK);
	//Fill payload with data
	TM_SPI_WriteMulti(NRF24L01_SPI, data, count);
	NRF24L01_CSN_HIGH;
	
	//Delay(1000);
	//Send data!
	NRF24L01_CE_HIGH;
}

void TM_NRF24L01_GetData(uint8_t* data) {
	//Pull down chip select
	NRF24L01_CSN_LOW;
	//Send read payload command
	TM_SPI_Send(NRF24L01_SPI, NRF24L01_R_RX_PAYLOAD_MASK);
	//Read payload
	TM_SPI_SendMulti(NRF24L01_SPI, data, data, TM_NRF24L01_Struct.PayloadSize);
	//Pull up chip select
	NRF24L01_CSN_HIGH;
	
	//Reset status register, clear RX_DR interrupt flag
	TM_NRF24L01_WriteRegister(NRF24L01_REG_STATUS, (1 << NRF24L01_RX_DR));
}

uint8_t TM_NRF24L01_DataReady(void) {
	uint8_t status = TM_NRF24L01_GetStatus();
	
	if (NRF24L01_CHECK_BIT(status, NRF24L01_RX_DR)) {
		return 1;
	}
	return !TM_NRF24L01_RxFifoEmpty();
}

uint8_t TM_NRF24L01_RxFifoEmpty(void) {
	uint8_t reg = TM_NRF24L01_ReadRegister(NRF24L01_REG_FIFO_STATUS);
	return NRF24L01_CHECK_BIT(reg, NRF24L01_RX_EMPTY);
}

uint8_t TM_NRF24L01_GetStatus(void) {
	uint8_t status;
	
	NRF24L01_CSN_LOW;
	//First received byte is always status register
	status = TM_SPI_Send(NRF24L01_SPI, NRF24L01_NOP_MASK);
	//Pull up chip select
	NRF24L01_CSN_HIGH;
	
	return status;
}

TM_NRF24L01_Transmit_Status_t TM_NRF24L01_GetTransmissionStatus(void) {
	uint8_t status = TM_NRF24L01_GetStatus();
	if (NRF24L01_CHECK_BIT(status, NRF24L01_TX_DS)) {
		//Successfully sent
		return TM_NRF24L01_Transmit_Status_Ok;
	} else if (NRF24L01_CHECK_BIT(status, NRF24L01_MAX_RT)) {
		//Message lost
		return TM_NRF24L01_Transmit_Status_Lost;
	}
	
	//Still sending
	return TM_NRF24L01_Transmit_Status_Sending;
}

void TM_NRF24L01_SoftwareReset(void) {
	uint8_t data[5];
	
	TM_NRF24L01_WriteRegister(NRF24L01_REG_CONFIG, 		NRF24L01_REG_DEFAULT_VAL_CONFIG);
	TM_NRF24L01_WriteRegister(NRF24L01_REG_EN_AA,		NRF24L01_REG_DEFAULT_VAL_EN_AA);
	TM_NRF24L01_WriteRegister(NRF24L01_REG_EN_RXADDR, 	NRF24L01_REG_DEFAULT_VAL_EN_RXADDR);
	TM_NRF24L01_WriteRegister(NRF24L01_REG_SETUP_AW, 	NRF24L01_REG_DEFAULT_VAL_SETUP_AW);
	TM_NRF24L01_WriteRegister(NRF24L01_REG_SETUP_RETR, 	NRF24L01_REG_DEFAULT_VAL_SETUP_RETR);
	TM_NRF24L01_WriteRegister(NRF24L01_REG_RF_CH, 		NRF24L01_REG_DEFAULT_VAL_RF_CH);
	TM_NRF24L01_WriteRegister(NRF24L01_REG_RF_SETUP, 	NRF24L01_REG_DEFAULT_VAL_RF_SETUP);
	TM_NRF24L01_WriteRegister(NRF24L01_REG_STATUS, 		NRF24L01_REG_DEFAULT_VAL_STATUS);
	TM_NRF24L01_WriteRegister(NRF24L01_REG_OBSERVE_TX, 	NRF24L01_REG_DEFAULT_VAL_OBSERVE_TX);
	TM_NRF24L01_WriteRegister(NRF24L01_REG_RPD, 		NRF24L01_REG_DEFAULT_VAL_RPD);
	
	//P0
	data[0] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_0;
	data[1] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_1;
	data[2] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_2;
	data[3] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_3;
	data[4] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_4;
	TM_NRF24L01_WriteRegisterMulti(NRF24L01_REG_RX_ADDR_P0, data, 5);
	
	//P1
	data[0] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_0;
	data[1] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_1;
	data[2] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_2;
	data[3] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_3;
	data[4] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_4;
	TM_NRF24L01_WriteRegisterMulti(NRF24L01_REG_RX_ADDR_P1, data, 5);
	
	TM_NRF24L01_WriteRegister(NRF24L01_REG_RX_ADDR_P2, 	NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P2);
	TM_NRF24L01_WriteRegister(NRF24L01_REG_RX_ADDR_P3, 	NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P3);
	TM_NRF24L01_WriteRegister(NRF24L01_REG_RX_ADDR_P4, 	NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P4);
	TM_NRF24L01_WriteRegister(NRF24L01_REG_RX_ADDR_P5, 	NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P5);
	
	//TX
	data[0] = NRF24L01_REG_DEFAULT_VAL_TX_ADDR_0;
	data[1] = NRF24L01_REG_DEFAULT_VAL_TX_ADDR_1;
	data[2] = NRF24L01_REG_DEFAULT_VAL_TX_ADDR_2;
	data[3] = NRF24L01_REG_DEFAULT_VAL_TX_ADDR_3;
	data[4] = NRF24L01_REG_DEFAULT_VAL_TX_ADDR_4;
	TM_NRF24L01_WriteRegisterMulti(NRF24L01_REG_TX_ADDR, data, 5);
	
	TM_NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P0, 	NRF24L01_REG_DEFAULT_VAL_RX_PW_P0);
	TM_NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P1, 	NRF24L01_REG_DEFAULT_VAL_RX_PW_P1);
	TM_NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P2, 	NRF24L01_REG_DEFAULT_VAL_RX_PW_P2);
	TM_NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P3, 	NRF24L01_REG_DEFAULT_VAL_RX_PW_P3);
	TM_NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P4, 	NRF24L01_REG_DEFAULT_VAL_RX_PW_P4);
	TM_NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P5, 	NRF24L01_REG_DEFAULT_VAL_RX_PW_P5);
	TM_NRF24L01_WriteRegister(NRF24L01_REG_FIFO_STATUS, NRF24L01_REG_DEFAULT_VAL_FIFO_STATUS);
	TM_NRF24L01_WriteRegister(NRF24L01_REG_DYNPD, 		NRF24L01_REG_DEFAULT_VAL_DYNPD);
	TM_NRF24L01_WriteRegister(NRF24L01_REG_FEATURE, 	NRF24L01_REG_DEFAULT_VAL_FEATURE);
}

uint8_t TM_NRF24L01_GetRetransmissionsCount(void) {
	//Low 4 bits
	return TM_NRF24L01_ReadRegister(NRF24L01_REG_OBSERVE_TX) & 0x0F;
}

void TM_NRF24L01_SetChannel(uint8_t channel) {
	if (channel <= 125 && channel != TM_NRF24L01_Struct.Channel) {
		TM_NRF24L01_Struct.Channel = channel;
		TM_NRF24L01_WriteRegister(NRF24L01_REG_RF_CH, channel);
	}
}

void TM_NRF24L01_SetRF(TM_NRF24L01_DataRate_t DataRate, TM_NRF24L01_OutputPower_t OutPwr) {
	uint8_t tmp = 0;
	TM_NRF24L01_Struct.DataRate = DataRate;
	TM_NRF24L01_Struct.OutPwr = OutPwr;
	
	if (DataRate == TM_NRF24L01_DataRate_2M) {
		tmp |= 1 << NRF24L01_RF_DR_HIGH;
	} else if (DataRate == TM_NRF24L01_DataRate_250k) {
		tmp |= 1 << NRF24L01_RF_DR_LOW;
	}
	//If 1Mbps, all bits set to 0
	
	if (OutPwr == TM_NRF24L01_OutputPower_0dBm) {
		tmp |= 3 << NRF24L01_RF_PWR;
	} else if (OutPwr == TM_NRF24L01_OutputPower_M6dBm) {
		tmp |= 2 << NRF24L01_RF_PWR;
	} else if (OutPwr == TM_NRF24L01_OutputPower_M12dBm) {
		tmp |= 1 << NRF24L01_RF_PWR;
	}
	
	TM_NRF24L01_WriteRegister(NRF24L01_REG_RF_SETUP, tmp);
}

uint8_t TM_NRF24L01_SendPacket(uint8_t *data, uint8_t length){
	TM_NRF24L01_Transmit_Status_t transmissionStatus;
	uint8_t result = 0;
	uint8_t dataTx[32];

	//Un unico invio
	if(length <= 30)
	{
		//uint8_t dataTx[32];
		uint8_t i;

		dataTx[0] = 0x02;
		dataTx[1] = length;

		for (i=0; i<length; i++)
		{
			dataTx[i+2] = data[i];
		}


		//TM_DISCO_LedOn(LED_BLUE);
		//Wait for data to be sent
		TM_NRF24L01_Transmit(dataTx); //TODO - esterna al ciclo
		//TM_NRF24L01_PowerUpRx();
		uint8_t ritrasmissioni = 0;
		do
		{
			//Send it back, automatically goes to TX mode
			//TM_NRF24L01_Transmit(dataTx);
			transmissionStatus = TM_NRF24L01_GetTransmissionStatus();
			//ritrasmissioni = TM_NRF24L01_GetRetransmissionsCount();
			//printf("%x /n",ritrasmissioni);
		}
		while (transmissionStatus == TM_NRF24L01_Transmit_Status_Sending);
		//TM_DISCO_LedOff(LED_BLUE);

		//Go back to RX Mode
		TM_NRF24L01_PowerUpRx();
	}
	else if(length <= 62)
	{
		uint8_t dataHigh[32];
		uint8_t dataLow[32];
		uint8_t i = 0;

		dataHigh[0] = 0x01;
		dataLow[0] = 0x00;
		dataLow[1] = length;

		//TODO - for per riempire e poi invio
		for (i=0; i<=30; i++){
			dataLow[i+2] = data[i];
		}

		for (i=0; i<=(length-30); i++){
			dataHigh[i+1] = data[i + 30];
		}

		//TM_DISCO_LedOn(LED_BLUE);
		//Wait for data to be sent
		//TODO - aggiustare questo
		TM_NRF24L01_Transmit(dataLow);
		do {
			//Send it back, automatically goes to TX mode
			transmissionStatus = TM_NRF24L01_GetTransmissionStatus();
		} while (transmissionStatus == TM_NRF24L01_Transmit_Status_Sending);


		TM_NRF24L01_Transmit(dataHigh);
		do {
			//Send it back, automatically goes to TX mode
			transmissionStatus = TM_NRF24L01_GetTransmissionStatus();
		} while (transmissionStatus == TM_NRF24L01_Transmit_Status_Sending);
		//TM_DISCO_LedOff(LED_BLUE);
		TM_NRF24L01_PowerUpRx();
	}

	if (transmissionStatus == TM_NRF24L01_Transmit_Status_Ok)
		result = 1;
	else if (transmissionStatus == TM_NRF24L01_Transmit_Status_Lost)
		result = 0;

	return result;
}


//TODO - attenzione modifica
/* Handler di gestione della comunicazione con nRF24L01P. Gestisce l'interruzione generatasi dal pin IRQ
 * della nRF connesso ad un pin configurabile della Discovery (nella configurazione)
 * L'handler gestisce tre casi:
 * - RX_INTERRUPT: Il dispositivo ha ricevuto una frame da bufferizzare.
 * - TX_INTERRUPT: Il dispositivo ha ricevuto l'ACK relativo alla frame appena inviata.
 * - MAX_RT: Il dispositivo ha raggiunto il massimo numero di ritentativi di trasmissione di una frame.
 */

void EXTI9_5_IRQHandler(void) {
	printf("Sto per entrare nell' interruzione");
	if (EXTI_GetITStatus(NRF24L01_IRQ_EXTI_LINE) != RESET) {
		EXTI_ClearITPendingBit(NRF24L01_IRQ_EXTI_LINE);
		NRF24L01_CE_LOW;

		printf("Prima cond superata");
		//uint8_t status;
		//status = nRF24L01_get_status(&rf);
		if ((TM_NRF24L01_GetStatus() & RX_DR) != 0) {
			//RX_INTERRUPT
			if (TM_NRF24L01_DataReady()) {

				//TM_DISCO_LedToggle(LED_BLUE);
				//Get data from NRF2L01+
				TM_NRF24L01_GetData(dataRec);

				printf("Dato Ricevuto INTERRUZIONE : %x \n", dataRec[0]);

				uint8_t i;


				if(dataRec[0] == 0x01){
					//sblocco mutex
					for (i=0; i<=(great_packet_lenght-31); i++){
						ricBuffer[i+30] = dataRec[i+1];
					}
					rxMutex = 0x02;
				}
				else if(dataRec[0] == 0x00){
					great_packet_lenght = dataRec[1];
					printf("Lunghezza Pacchetto Grande: %d \n\n\n\r", great_packet_lenght);
					//non sblocco devo attendere il secondo frammento
					for (i=0; i<=30; i++){
						ricBuffer[i] = dataRec[i+2];
						printf("%x \n", ricBuffer[i]);
					}
				}
				else if(dataRec[0] == 0x02){
					//sblocco il mutex essendo un unico invio
					packet_lenght = dataRec[1];
					for (i=0; i<=packet_lenght; i++){
						ricBuffer[i] = dataRec[i+2];
						printf("%x \n", ricBuffer[i]);
					}
					//Indico con il mutex che ho ricevuto una singola frame da 31 byte
					rxMutex = 0x01;
				}
				printf("\n\n");
				TM_NRF24L01_PowerUpRx();
			}

		}
		//TM_NRF24L01_PowerUpRx();
	}
}

