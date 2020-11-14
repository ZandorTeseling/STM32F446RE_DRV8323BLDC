/*
 * user_configs.h
 *
 *  Created on: Oct 24, 2020
 *      Author: zandor
 */

#ifndef INC_USER_CONFIG_H_
#define INC_USER_CONFIG_H_

//TODO
//#define E_OFFSET                __float_reg[0]                                  // Encoder electrical offset
//#define M_OFFSET                __float_reg[1]                                  // Encoder mechanical offset
#define I_BW                    500                                  // Current loop bandwidth
#define I_MAX                   5                                  // Torque limit (current limit = torque_limit/(kt*gear ratio))
//#define THETA_MIN               __float_reg[4]                                  // Minimum position setpoint
//#define THETA_MAX               __float_reg[5]                                  // Maximum position setpoint
#define I_FW_MAX                5                                  // Maximum field weakening current
#define R_NOMINAL               0.15                                  // Nominal motor resistance, set during calibration
#define TEMP_MAX                65                                  // Temperature safety lmit
//#define I_MAX_CONT              __float_reg[9]                                  // Continuous max current


#endif /* INC_USER_CONFIGS_H_ */
