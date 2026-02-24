/*
 * 7segmentDisplay_4D.h
 *
 *  Created on: Feb 17, 2026
 *      Author: broug
 *
 *      README
 *      SPI must be LSB first
 *      Daisy chain number 74HC595 -> digit 74HC595
 *
 */

#ifndef INC_7SEGMENTDISPLAY_4D_H_
#define INC_7SEGMENTDISPLAY_4D_H_

#include "stm32g4xx_hal.h"
#include<math.h>
#include "main.h" // moved this from c to h


/*
 * MACROS
 */

//numbers (active HIGH - common cathode)

#define SSD_ZERO 0b11111100
#define SSD_ONE 0b01100000
#define SSD_TWO 0b11011010
#define SSD_THREE 0b11110010
#define SSD_FOUR 0b01100110
#define SSD_FIVE 0b10110110
#define SSD_SIX 0b10111111
#define SSD_SEVEN 0b11100000
#define SSD_EIGHT 0b11111110
#define SSD_NINE 0b11110110

//digit selection (active LOW - common cathode)

#define SSD_D4 0b01111111
#define SSD_D3 0b10111111
#define SSD_D2 0b11011111
#define SSD_D1 0b11101111

//if using common anode, invert selection

//decimal point
#define SSD_NO_DECIMAL 0xff

//segments array

//uint8_t segmentArray[] = {SSD_ZERO, SSD_ONE, SSD_TWO, SSD_THREE, SSD_FOUR, SSD_FIVE, SSD_SIX, SSD_SEVEN, SSD_EIGHT, SSD_NINE}; //this does not work here ?!?!?!?!


typedef struct {

	SPI_HandleTypeDef *spiHandle;
	GPIO_TypeDef *RCLKport;
	uint16_t RCLKpin;

	uint16_t *num;
	float *floatNum;

	uint8_t decPoints;

}sevenSegmentDisplay;

void SSD_Init(sevenSegmentDisplay *dev, SPI_HandleTypeDef *handle, GPIO_TypeDef *Rport, uint16_t Rpin, uint16_t *dat, uint8_t dp); //dat is a pointer to the data that is to be displayed, dp is decimal points (3 max)

void SSD_Write_number(sevenSegmentDisplay *dev, uint8_t num, uint8_t dig, uint8_t dp);

void numToSegment(uint16_t num,  uint8_t *o1,  uint8_t *o2,  uint8_t *o3,  uint8_t *o4);

void largeNumToSegment(uint16_t num,  uint8_t *o1,  uint8_t *o2,  uint8_t *o3,  uint8_t *o4);

void updateScreen(sevenSegmentDisplay *dev, uint8_t dig);

void floatUpdateScreen(sevenSegmentDisplay *dev, float num, uint8_t *flag);

#endif /* INC_7SEGMENTDISPLAY_4D_H_ */
