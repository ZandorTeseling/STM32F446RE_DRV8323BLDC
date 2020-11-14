/*
 * hw_config.h
 *
 *  Created on: Nov 14, 2020
 *      Author: zandor
 */

#ifndef INC_HW_CONFIG_H_
#define INC_HW_CONFIG_H_

#define PIN_U PA_10
#define PIN_V PA_9
#define PIN_W PA_8
#define ENABLE_PIN PC_8        // DRV8328 Chip Enable Pin
#define ENABLE_HiZPIN PC_9     // DRV8328 Enable Gate Drive Pin
//#define LED         PC_5        // LED Pin
#define I_SCALE 0.02014160156f  // Amps per A/D Count TODO:CHECK Value
#define V_SCALE 0.012890625f     // Bus volts per A/D Count
#define DTC_MAX 0.94f          // Max phase duty cycle
#define DTC_MIN 0.0f          // Min phase duty cycle
#define PWM_ARR 0x8CA           /// timer autoreload value
#define DTC_COMP .000f          /// deadtime compensation (100 ns / 25 us)



#endif /* INC_HW_CONFIG_H_ */
