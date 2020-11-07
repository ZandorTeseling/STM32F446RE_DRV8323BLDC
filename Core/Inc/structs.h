/*
 * structs.h
 *
 *  Created on: 26 Oct 2020
 *      Author: zandor
 */

#ifndef INC_STRUCTS_H_
#define INC_STRUCTS_H_

#include "stm32f4xx_hal.h"

typedef struct{
	SPI_HandleTypeDef *pSPI_Handle;
	GPIO_TypeDef* PORT_GPIOx;
	uint16_t CS_GPIO_Pin;
	uint16_t SPI_RX_Data[10];
	uint16_t SPI_TX_Data[10];
	uint8_t SPI_TX_Flag;
	uint8_t SPI_RX_Flag;
} SPIStruct;

typedef struct{
	CAN_FilterTypeDef   FilterConfig;
	CAN_TxHeaderTypeDef TxHeader;
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t             TxData[8];
	uint8_t             RxData[8];
	uint32_t            TxMailbox;
	uint8_t 			RxFlag;
} CANStruct;



typedef struct{
    double temperature;                                              // Estimated temperature
    float temp_measured;
    float q_in, q_out;
    float resistance;
    float k;
    float trust;
    float delta_t;
}   ObserverStruct;

typedef struct{
	uint32_t adc1_raw, adc2_raw, adc3_raw;                  // Raw ADC Values
    float i_a, i_b, i_c;                                    // Phase currents
    float v_bus;                                            // DC link voltage
    float theta_mech, theta_elec;                           // Rotor mechanical and electrical angle
    float dtheta_mech, dtheta_elec, dtheta_elec_filt;       // Rotor mechanical and electrical angular velocit
    float i_d, i_q, i_q_filt, i_d_filt;                               // D/Q currents
    float v_d, v_q;                                         // D/Q voltages
    float dtc_u, dtc_v, dtc_w;                              // Terminal duty cycles
    float v_u, v_v, v_w;                                    // Terminal voltages
    float k_d, k_q, ki_d, ki_q, alpha;                      // Current loop gains, current reference filter coefficient
    float d_int, q_int;                                     // Current error integrals
    int adc1_offset, adc2_offset;                           // ADC offsets
    float i_d_ref, i_q_ref, i_d_ref_filt, i_q_ref_filt;     // Current references
    int loop_count;                                         // Degubbing counter
    int timeout;                                            // Watchdog counter
    int mode;
    int ovp_flag;                                           // Over-voltage flag
    float p_des, v_des, kp, kd, t_ff;                       // Desired position, velocity, gians, torque
    float v_ref, fw_int;                                     // output voltage magnitude, field-weakening integral
    float cogging[128];
    int current_sector;
    int otw_flag;                                           // Over-temp warning
    float i_max;
    float inverter_tab[128];
    int oc_flag;
} ControllerStruct;



typedef struct{
	enum rotation_direction{
		Rot_NEG = -1,
		Rot_RESET = 0,
		Rot_POS = 1
	} dir;

	int16_t rotations;
	uint16_t raw_value;
	uint8_t pole_pairs;
	uint8_t first_sample;
	uint32_t cpr;
	float mech_position;
	float mech_velocity;
	float offset;
	float elec_position;
	float elec_velocity;

	float rotation_ratio;
	float old_rotation_ratio;
	float vel_array[40];
} AngularPositionSensorStruct;

#endif /* INC_STRUCTS_H_ */
