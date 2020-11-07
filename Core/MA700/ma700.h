/*
 * ma700.h
 *
 *  Created on: Nov 4, 2020
 *      Author: zandor
 */

#ifndef MA700_MA700_H_
#define MA700_MA700_H_

#include "stm32f4xx_hal.h"
#include <../Inc/structs.h>
#include <../Inc/user_configs.h>
#include <../Inc/retarget.h>
#include <../Inc/motor_config.h>

#define MA700_CPR 0xFFFF
#define MY_PI 3.1415926535897932

void MA700_initSPIInterface(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, SPI_HandleTypeDef* pspi);
void updateEncoder(float a_dt);
void resetEncoder(void);
void printEncoder(void);

#endif /* MA700_MA700_H_ */
