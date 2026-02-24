/*
 * 7segmentDisplay_4D.c
 *
 *  Created on: Feb 17, 2026
 *      Author: broug
 */


#include "7segmentDisplay_4D.h"

uint8_t segmentArray[] = {SSD_ZERO, SSD_ONE, SSD_TWO, SSD_THREE, SSD_FOUR, SSD_FIVE, SSD_SIX, SSD_SEVEN, SSD_EIGHT, SSD_NINE};

uint8_t counter = 0;


void SSD_Init(sevenSegmentDisplay *dev, SPI_HandleTypeDef *handle, GPIO_TypeDef *Rport, uint16_t Rpin, uint16_t *dat, uint8_t dp){
	dev->spiHandle = handle;
	dev->RCLKport = Rport;
	dev->RCLKpin = Rpin;
	dev->num = dat;
	dev->decPoints = dp;
}

void SSD_Write_number(sevenSegmentDisplay *dev, uint8_t num, uint8_t dig, uint8_t dp){

	HAL_GPIO_WritePin(dev->RCLKport, dev->RCLKpin, 0);

	uint8_t digit; //buffer for digit selection, to be transmitted to the digit shift register

	switch (dig){//decides which digit I want to display, and then inserts the macro for the digit selection shift register into the digit buffer
	case 0:
		digit = SSD_D1;
		break;
	case 1:
		digit = SSD_D2;
		break;
	case 2:
		digit = SSD_D3;
		break;
	case 3:
		digit = SSD_D4;
		break;
	}



	uint8_t pData[2] = {segmentArray[num], digit};// buffer fpr the data to be transmitted. [0] is the number to be displayed, [1] is for the digit selection

	if (dp == 1){ //if decimal place is desired at the digit, enable Q7 of the shift register (connected to decimal point of the display)
		pData[0] |= 0b00000001;
	}
	else if(dp == SSD_NO_DECIMAL){ //if no decimal is required ensure that bit is turned off
		pData[0] &= ~0x01;// ~00000001 = 11111110 - make the last bit 0 to disable decimal point
	}

	HAL_SPI_Transmit(dev->spiHandle, pData, 2, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(dev->RCLKport, dev->RCLKpin, 1);
	HAL_GPIO_WritePin(dev->RCLKport, dev->RCLKpin, 0);
}

void numToSegment(uint16_t num,  uint8_t *o1,  uint8_t *o2,  uint8_t *o3,  uint8_t *o4){
	*o1 = (int)floor((num/1000) % 10);
	*o2 = (int)floor((num/100) % 10);
	*o3 = (int)floor((num/10) % 10);
	*o4 = (int)floor(num % 10);

}

void largeNumToSegment(uint16_t num,  uint8_t *o1,  uint8_t *o2,  uint8_t *o3,  uint8_t *o4){

	num /= 10;

	*o1 = (int)floor((num/1000) % 10);
	*o2 = (int)floor((num/100) % 10);
	*o3 = (int)floor((num/10) % 10);
	*o4 = (int)floor(num % 10);

}

void updateScreen(sevenSegmentDisplay *dev, uint8_t dig){

	uint8_t buff[4];

	numToSegment(*(dev->num), &buff[0], &buff[1], &buff[2], &buff[3]);
	//numToSegment(1489, &skibidiNumber[0], &skibidiNumber[1], &skibidiNumber[2], &skibidiNumber[3]);

	SSD_Write_number(dev, buff[dig], dig, SSD_NO_DECIMAL);

//	for (int i = 0; i < 4; i++){
//		SSD_Write_number(dev, buff[i], i);
//
//	}

}



void floatUpdateScreen(sevenSegmentDisplay *dev, float num, uint8_t *flag){

	if(num == 0){
		*flag = 1;
		return;
	}

	uint8_t buff[4];

	uint32_t intValue = (uint32_t)(num * 1000.0f + 0.5f);// preserve 3 decimal places of the float ( max the display can do anyway), and then add .5 for rounding as the dp gets chopped off when trunacated.

	if(intValue > 100000){
		Error_Handler(); // I may or may not sort out error handling :)
	}

	if(intValue < 10000){

		numToSegment(intValue, &buff[0], &buff[1], &buff[2], &buff[3]);

	}
	else if(intValue <100000){

		largeNumToSegment(intValue, &buff[0], &buff[1], &buff[2], &buff[3]);

	}



	uint8_t dec = SSD_NO_DECIMAL; //default to no decimal place

	if(intValue < 10000 && counter == 0){//unless the value is less than 10.000 and you are at the correct location for the decimal place (first digit)
		dec = 1; //set decimal place to current digit (first)
	}
	else if(((intValue < 100000) && (intValue > 10000)) && counter == 1){//unless the value is in between 10.000 and 100.000
		dec = 1;//set decimal place to current digit (second)
	}


	SSD_Write_number(dev, buff[counter], counter, dec);

	counter ++; //increment counter to keep track of digit

	if(counter == 4){ //if last digit has been reached

			*flag = 1; //flag for telling program the entire digit has been written

			counter = 0; // reset counter
		}


}
