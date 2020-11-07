/*
 * ma700.c
 *
 *  Created on: Nov 4, 2020
 *      Author: zandor
 */


#include <../MA700/ma700.h>

volatile AngularPositionSensorStruct s_encoder;
volatile SPIStruct s_ma700xSPI;

void MA700_initSPIInterface(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, SPI_HandleTypeDef* pspi){
	s_ma700xSPI.PORT_GPIOx = GPIOx;
	s_ma700xSPI.CS_GPIO_Pin = GPIO_Pin;
	s_ma700xSPI.pSPI_Handle = pspi;
	s_ma700xSPI.SPI_TX_Flag = 0;
	s_ma700xSPI.SPI_RX_Flag = 0;

	resetEncoder();
}


void resetEncoder(){
	s_encoder.first_sample = 0;
	s_encoder.cpr = MA700_CPR;
	s_encoder.dir = Rot_RESET;
	s_encoder.pole_pairs = NPP;
	s_encoder.rotations = 0;
	s_encoder.mech_position = 0;
	s_encoder.mech_velocity = 0;

	s_encoder.elec_position = 0;
	s_encoder.elec_velocity = 0;

	s_encoder.offset = 0;
	s_encoder.rotation_ratio = 0;
	s_encoder.old_rotation_ratio = 0;
}

//Mechanical position of the encoder is updated.
void updateEncoder(float a_dt){
	uint16_t old_raw = s_encoder.raw_value;
	float old_mech_position = s_encoder.mech_position;

	s_encoder.raw_value 	     = s_ma700xSPI.SPI_RX_Data[0];
	s_encoder.old_rotation_ratio = s_encoder.rotation_ratio;
	s_encoder.rotation_ratio     = s_encoder.raw_value  / (float) MA700_CPR;

	if(s_encoder.first_sample){
		//Check at the midpoint of rotation to see if a full rotation has occured
		if(s_encoder.raw_value - old_raw > MA700_CPR/2 ){
			s_encoder.rotations -= 1;
		}
		else if(s_encoder.raw_value - old_raw < -MA700_CPR/2)
		{
			s_encoder.rotations += 1;
		}
	}
	else{
		s_encoder.first_sample = 1;
	}

	s_encoder.mech_position = 2.0 * MY_PI * (s_encoder.rotations + s_encoder.rotation_ratio);

	//No need to handle angle wrapping when computing delta
	float vel = (s_encoder.mech_position - old_mech_position)/a_dt;
    float sum = vel;
    int n = 40;
    for (int i = 1; i < n; i++){
    	s_encoder.vel_array[n - i] = s_encoder.vel_array[(n-i)-1]; //Shift right.
        sum += s_encoder.vel_array[n-i];
	}
    s_encoder.vel_array[0] = vel;
    s_encoder.mech_velocity  =  sum/((float)n);

}

//To work with no floating point support printf library.
void printEncoder(void){
	float intPosVal = (int)s_encoder.mech_position;
	float decPosVal = (s_encoder.mech_position - (float)intPosVal)*1000.0;
	float intVelVal = (int)s_encoder.mech_velocity;
    float decVelVal = (s_encoder.mech_velocity - (float)intVelVal)*1000.0;

    if(s_encoder.mech_position < 0){
		printf("Ang[int], %d, Ang[rad], -%d.%03d,", s_encoder.raw_value,(int)(-1.0*intPosVal),(int)(-1.0*decPosVal));
	}
	else{
		printf("Ang[int], %d, Ang[rad], %d.%03d, ", s_encoder.raw_value,(int)intPosVal,(int)(decPosVal));
	}
    if(s_encoder.mech_velocity < 0){
    	printf("AngVel[rad/s], -%d.%03d\r\n",(int)(-1.0*intVelVal), (int)(-1.0*decVelVal));
    }
    else{
    	printf("AngVel[rad/s], %d.%03d\r\n",(int)intVelVal, (int)decVelVal);
    }

}
