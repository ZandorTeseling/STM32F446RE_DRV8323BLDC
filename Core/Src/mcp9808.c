/*
 * mcp9808.c
 *
 *  Created on: 18 Oct 2020
 *      Author: zandor
 */

#include <../Inc/mcp9808.h>

void MCP9808PrintTemp(I2C_HandleTypeDef* pI2C){
	HAL_StatusTypeDef ret;
	uint8_t data_write[12];
	uint8_t data_read[12];
	volatile char TempCelsiusDisplay[] = "+abc.dd C";
	int tempval;
	/* Configure the Temperature sensor device MCP9808:
	- Thermostat mode Interrupt not used
	- Fault tolerance: 0
	*/
	data_write[0] = MCP9808_REG_CONF;
	data_write[1] = 0x00;  // config msb
	data_write[2] = 0x00;  // config lsb
	ret = HAL_I2C_Master_Transmit(pI2C, MCP9808_ADDR, data_write,3,HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
	printf("Config Error:MCP9808 Tx\r\n");
	}

	data_write[0] = MCP9808_REG_TEMP;
	ret = HAL_I2C_Master_Transmit(pI2C,MCP9808_ADDR, data_write, 1, HAL_MAX_DELAY); // no stop
	if ( ret != HAL_OK ) {
	  printf("Notify Error:MCP9808 Tx\r\n");
	}
	ret = HAL_I2C_Master_Receive(pI2C, MCP9808_ADDR, data_read, 2, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
	  printf("Notify Error:MCP9808 Tx\r\n");
	}

	  if((data_read[0] & 0x80)  == 0x80) {
		 printf(" temp >= critical ");
	 }
	 if((data_read[0] & 0x40) == 0x40) {
		 printf("   temp > upper limit ");
	 }
	 if((data_read[0] & 0x20) == 0x20) {
		 printf(" temp < lower limit  ");
	 }
	 if(data_read[0] & 0xE0) {
		 printf("\r\n");
		 data_read[0] = data_read[0] & 0x1F;  // clear flag bits
	 }
	 if((data_read[0] & 0x10) == 0x10) {
		 data_read[0] = data_read[0] & 0x0F;
		 TempCelsiusDisplay[0] = '-';
		 tempval = 256 - (data_read[0] << 4) + (data_read[1] >> 4);
	 } else {
		 TempCelsiusDisplay[0] = '+';
		 tempval = (data_read[0] << 4) + (data_read[1] >> 4);
	 }

	 // fractional part (0.25°C precision)
	 if (data_read[1] & 0x08) {
		 if(data_read[1] & 0x04) {
			 TempCelsiusDisplay[5] = '7';
			 TempCelsiusDisplay[6] = '5';
		 } else {
			 TempCelsiusDisplay[5] = '5';
			 TempCelsiusDisplay[6] = '0';
		 }
	 } else {
		 if(data_read[1] & 0x04) {
			 TempCelsiusDisplay[5] = '2';
			 TempCelsiusDisplay[6] = '5';
		 }else{
			 TempCelsiusDisplay[5] = '0';
			 TempCelsiusDisplay[6] = '0';
		 }
	 }

	 // Integer part
	 TempCelsiusDisplay[1] = (tempval / 100) + 0x30;
	 TempCelsiusDisplay[2] = ((tempval % 100) / 10) + 0x30;
	 TempCelsiusDisplay[3] = ((tempval % 100) % 10) + 0x30;
	 printf("temp = %s\r\n", TempCelsiusDisplay);
}

