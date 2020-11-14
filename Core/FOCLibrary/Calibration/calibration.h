#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "../Config/user_config.h"
#include "../Config/motor_config.h"
#include "../Config/current_controller_config.h"

#include "../FOCLibrary/FOC/foc.h"

//#include "PreferenceWriter.h"


#define V_CAL 0.15f;

void order_phases(SPIStruct* sensorSPIStruct, ControllerStruct *controller);// PreferenceWriter *prefs);
void calibrate(SPIStruct* sensorSPIStruct, ControllerStruct *controller);// PreferenceWriter *prefs);
#endif
