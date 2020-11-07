/*
 * motor_config.h
 *
 *  Created on: Nov 4, 2020
 *      Author: zandor
 */

#ifndef INC_MOTOR_CONFIG_H_
#define INC_MOTOR_CONFIG_H_

#define R_PHASE 0.13f           //Ohms
#define L_D 0.00008f            //Henries
#define L_Q 0.00008f            //Henries
#define KT .075f                //N-m per peak phase amp, = WB*NPP*3/2
#define NPP 21                  //Number of pole pairs
#define GR 6.0f                 //Gear ratio
#define KT_OUT 0.45f            //KT*GR
#define WB 0.00287f             //Flux linkage, Webers.
#define R_TH 1.25f              //Kelvin per watt
#define INV_M_TH 0.02825f       //Kelvin per joule
#define T_AMBIENT 25.0f        // ambient temperature during temp calibration

#endif /* INC_MOTOR_CONFIG_H_ */
