/*
 * i2c_scan.c
 *
 *  Created on: Jul 6, 2023
 *      Author: Loc
 */


#include "i2c_scan.h"


uint8_t i2c_scan(I2C_HandleTypeDef * hi2c)
{
	uint8_t data = 0x00;
	for(uint8_t addr = 0x07; addr <= 0x7F; addr++)
	{
		if(HAL_I2C_Master_Transmit(hi2c, (addr << 1)| 0x01, &data, 1, 10) == HAL_OK)
			return addr;
		HAL_Delay(10);
	}
	return 0x00;
}
