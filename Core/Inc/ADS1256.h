/*
 * ADS1256.h
 *
 *  Created on: Feb 20, 2026
 *      Author: broug
 */

#ifndef INC_ADS1256_H_
#define INC_ADS1256_H_

#include "stm32g4xx_hal.h"
#include "main.h"

/*
 * Register macros
 *
 * Ref register map, ADS1256 datasheet page 30 onwards
 *
 */

#define R_ADS1256_STATUS 0x00
#define R_ADS1256_MUX 0x01
#define R_ADS1256_ADCON 0x02
#define R_ADS1256_DRATE 0x03
#define R_ADS1256_IO 0x04
#define R_ADS1256_OFC0 0x05
#define R_ADS1256_OFC1 0x06
#define R_ADS1256_OFC2 0x07
#define R_ADS1256_FSC0 0x08
#define R_ADS1256_FSC1 0x09
#define R_ADS1256_FSC2 0x0A

/*
 * Command macros
 */

#define C_ADS1256_RDATA 0b00000001

/*
 * MUX Macros
 */

#define P_AIN0 0b00000000
#define P_AIN1 0b00010000
#define P_AIN2 0b00100000
#define P_AIN3 0b00110000
#define P_AIN4 0b01000000
#define P_AIN5 0b01010000
#define P_AIN6 0b01100000
#define P_AIN7 0b01110000
#define P_COM 0b11110000

#define N_AIN0 0b00000000
#define N_AIN1 0b00000001
#define N_AIN2 0b00000010
#define N_AIN3 0b00000011
#define N_AIN4 0b00000100
#define N_AIN5 0b00000101
#define N_AIN6 0b00000110
#define N_AIN7 0b00000111
#define N_COM 0b00001111

//unsure if I should bother doing this part I will see

/*
 * Macro for chip select
 */

#define ADS1256_CS_EN HAL_GPIO_WritePin(dev->CSport, dev->CSpin, 0);
#define ADS1256_CS_DIS HAL_GPIO_WritePin(dev->CSport, dev->CSpin, 1);

typedef struct{

	GPIO_TypeDef* CSport;
	uint16_t CSpin;

	GPIO_TypeDef* RSTport;
	uint16_t RSTpin;

	SPI_HandleTypeDef* spiHandle;

	uint32_t* rawData; //pointer to array raw data
	float* data; //pointer to array float of data if converted

	uint8_t* softwareResetFlag;

}ADS1256;

void ADS1256_init(ADS1256* dev, SPI_HandleTypeDef* spi, GPIO_TypeDef *csPort, uint16_t csPin, GPIO_TypeDef *rstPort, uint16_t rstPin, uint32_t* rData, float* fdata, uint8_t* softwareRSTflag);

void ADS1256_ReadRegister(ADS1256* dev, uint8_t reg, uint8_t num, uint8_t* pData);

void ADS1256_WriteRegister(ADS1256* dev, uint8_t reg, uint8_t num, uint8_t* pData);

void ADS1256_RegDump(ADS1256* dev, uint8_t* dump); // read all the registers and dump them into an array for debug.

void ADS1256_SetChannel(ADS1256* dev, uint8_t CHp, uint8_t CHn);

void ADS1256_Read(ADS1256* dev, int32_t* pData);

void ADS1256_Convert(ADS1256* dev, int32_t intVal, float* num);

void ADS1256_SoftwareReset(ADS1256* dev); //for resetting via software, must be done upon DRDY low

void ADS1256_Reset(ADS1256* dev);


#endif /* INC_ADS1256_H_ */
