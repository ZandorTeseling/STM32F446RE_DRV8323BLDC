/*
 * mcp9808.h
 *
 *  Created on: 18 Oct 2020
 *      Author: zandor
 */

#ifndef INC_MCP9808_H_
#define INC_MCP9808_H_

#include <../Inc/retarget.h>
#include <../Inc/user_configs.h>

/* MCP9808 high accuracy temp sensor from adafruit (no address pins pulled up) */
#define MCP9808_REG_TEMP (0x05) // Temperature Register
#define MCP9808_REG_CONF (0x01) // Configuration Register
#define MCP9808_ADDR     (0x30) // MCP9808 base address 0x18<<1


void MCP9808PrintTemp(I2C_HandleTypeDef* pI2C);

#endif /* INC_MCP9808_H_ */
