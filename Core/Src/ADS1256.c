/*
 * ADS1256.C
 *
 *  Created on: Feb 20, 2026
 *      Author: broug
 */

#include "ADS1256.h"

void ADS1256_init(ADS1256* dev, SPI_HandleTypeDef* spi, GPIO_TypeDef *csPort, uint16_t csPin, GPIO_TypeDef *rstPort, uint16_t rstPin, uint32_t* rData, float* fdata, uint8_t* softwareRSTflag){

	dev->spiHandle = spi;
	dev->CSpin = csPin;
	dev->CSport = csPort;
	dev->RSTpin = rstPin;
	dev->RSTport = rstPort;
	dev->rawData = rData;
	dev->data = fdata;
	dev->softwareResetFlag =softwareRSTflag;

	HAL_GPIO_WritePin(dev->RSTport, dev->RSTpin, 0);

	HAL_Delay(2);

	HAL_GPIO_WritePin(dev->RSTport, dev->RSTpin, 1);


}

void ADS1256_ReadRegister(ADS1256* dev, uint8_t reg, uint8_t num, uint8_t* pData){

	num -= 1; //reg to be read is desired number - 1

	if((reg > 0x0A) || (num > 10)){
		Error_Handler();
	}

	uint8_t buff[2];

	buff[0] = (reg | 0b00010000);
	buff[1] = num;

	//HAL_SPI_Transmit(dev->spiHandle, buff, 2, HAL_MAX_DELAY);

	HAL_SPI_Transmit(dev->spiHandle, &buff[0], 1, HAL_MAX_DELAY);


	HAL_SPI_Transmit(dev->spiHandle, &buff[1], 1, HAL_MAX_DELAY); //can probably combine these 2 but did this coz I am shit and need to fucking troublehsoot the cunt

	for(volatile uint32_t i = 0; i < 500; i++){
		__NOP();
	}

	HAL_SPI_Receive(dev->spiHandle, pData, (num + 1), HAL_MAX_DELAY);

}

void ADS1256_WriteRegister(ADS1256* dev, uint8_t reg, uint8_t num, uint8_t* pData){

	uint8_t buff[2];

	num -= 1;

	if((reg > 0x0A) || (num > 10)){
		Error_Handler();
	}

	buff[0] = (reg | 0b01010000);
	buff[1] = num;

	HAL_SPI_Transmit(dev->spiHandle, buff, 2, HAL_MAX_DELAY);

	//proably need a delay here

	HAL_SPI_Transmit(dev->spiHandle, pData, (num + 1), HAL_MAX_DELAY);

}

void ADS1256_RegDump(ADS1256* dev, uint8_t* dump){

	ADS1256_CS_EN

//	uint8_t data = 0b10101010;

//	HAL_SPI_Transmit(dev->spiHandle, &data, 1, HAL_MAX_DELAY);//---------TESTING REMOVE THIS -------------------------------------------!!!!
//	for(volatile uint32_t i = 0; i < 50; i++){ //short delay for T6
//		__NOP();
//	}
//	HAL_SPI_Transmit(dev->spiHandle, &data, 1, HAL_MAX_DELAY);

	ADS1256_ReadRegister(dev, 0, 11, dump);

	ADS1256_CS_DIS


}

void ADS1256_SetChannel(ADS1256* dev, uint8_t CHp, uint8_t CHn){

	uint8_t buff = CHp | CHn;

	ADS1256_WriteRegister(dev, R_ADS1256_MUX, 1, &buff);

}

void ADS1256_Read(ADS1256* dev, int32_t* pData){

	ADS1256_CS_EN

	uint8_t buff = C_ADS1256_RDATA;
	uint8_t receive[3];
	int32_t val;

	HAL_SPI_Transmit(dev->spiHandle, &buff, 1, HAL_MAX_DELAY);

	for(volatile uint32_t i = 0; i < 75; i++){ //short delay for T6
		__NOP();
	}

	HAL_SPI_Receive(dev->spiHandle, receive, 3, HAL_MAX_DELAY);

	val = (receive[0] << 16) | (receive[1] << 8) | receive[2];

	if (val & 0x800000){//check to see if there is a 1 in the 24th bit (signed 2s compliment negative)
		val |= 0xFF000000; //fill the left most 8 bits with 1's for a negative number
	}

	*pData = val;

	ADS1256_CS_DIS

}

void ADS1256_Convert(ADS1256* dev, int32_t intVal, float* num){

	*num = (float)intVal * (2.0f * 2.5) / (float)(1 * 8388607);
}

void ADS1256_SoftwareReset(ADS1256* dev){

	if(*dev->softwareResetFlag == 0){
		*dev->softwareResetFlag = 1;
	}

	else if(*dev->softwareResetFlag == 1){
		uint8_t buff;
		buff = 0xFE; //reset command, page 34

		ADS1256_CS_EN

		HAL_SPI_Transmit(dev->spiHandle, &buff, 1, HAL_MAX_DELAY);

		ADS1256_CS_DIS

		*dev->softwareResetFlag = 0;

	}
}

void ADS1256_Reset(ADS1256* dev){
	HAL_GPIO_WritePin(dev->RSTport, dev->RSTpin, 0);

	HAL_Delay(2);

	HAL_GPIO_WritePin(dev->RSTport, dev->RSTpin, 1);
}
