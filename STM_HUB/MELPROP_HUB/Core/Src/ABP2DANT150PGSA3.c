/*
 * ABP2DANT150PGSA3.c
 *
 *  Created on: Nov 17, 2023
 *      Author: p2166
 */
#include "ABP2DANT150PGSA3.h"

int8_t ABP2Write ()
{
	uint8_t cmd1[3] = {0xAA, 0x00, 0x00};
	uint8_t receive1[3] = {0, 0, 0};
	uint8_t cmd2 = 0xF0;
	uint8_t receive2 = 0;
	int8_t x = 1;

	HAL_GPIO_WritePin(GPIOA, 4, 0);
//	HAL_SPI_TransmitReceive(&hspi1, cmd1, receive1, 3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, 4, 1);

	HAL_GPIO_WritePin(GPIOA, 4, 0);
//	HAL_SPI_TransmitReceive(&hspi1, &cmd2, &receive2, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, 4, 1);
	x = (receive2 >> 4) & 1;

	return x;
}

uint32_t ABP2GetRaw ()
{
	uint8_t data[4] = {0, 0, 0, 0};
	uint32_t returnData = 0;
	uint8_t cmd[4] = {0xF0, 0x00, 0x00, 0x00};

	HAL_GPIO_WritePin(GPIOA, 4, 0);
//	HAL_SPI_TransmitReceive(&hspi1, cmd, data, 4, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, 4, 1);
	returnData = ((uint32_t) data[3]) | ((uint32_t) data[2] << 8) | ((uint32_t) data[1] << 16) | ((uint32_t) data[0] << 24);
	return returnData;
}

double ABP2Calc (uint32_t data)
{
	uint32_t press_counts = 0;
	uint32_t outputmax = 15099494; // output at maximum pressure [counts]
	uint32_t outputmin = 1677722; // output at minimum pressure [counts]
	uint32_t pmax = 150; // maximum value of pressure range psi
	uint32_t pmin = 0; // minimum value of pressure range psi

	double pressure = 0; // pressure reading psi

	uint8_t temp[4];

	temp[1] = (uint8_t) (data>>16);
	temp[2] = (uint8_t) (data>>8);
	temp[3] = (uint8_t) data;

	press_counts = temp[3] + (((uint32_t)temp[2])<<8) +  (((uint32_t)temp[1])<<16);

    uint32_t numerator = (press_counts - outputmin) * (pmax - pmin);
    uint32_t denominator = outputmax - outputmin;

    // Perform the division with higher precision (double)
    pressure = ((double)numerator) / denominator + pmin;

	return pressure;
}

