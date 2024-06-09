/*
 * ABP2DANT150PGSA3.h
 *
 *  Created on: Nov 17, 2023
 *      Author: p2166
 */

#ifndef INC_ABP2DANT150PGSA3_H_
#define INC_ABP2DANT150PGSA3_H_

#include "stm32f7xx_hal.h"
#include "spi.h"

int8_t ABP2Write ();
uint32_t ABP2GetRaw ();
double ABP2Calc (uint32_t data);
#endif /* INC_ABP2DANT150PGSA3_H_ */
