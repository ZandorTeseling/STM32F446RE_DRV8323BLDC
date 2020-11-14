#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdio.h>

#include "../Config/user_config.h"
#include "../Config/motor_config.h"
#include "../Config/current_controller_config.h"

#include "../FOCLibrary/FOC/foc.h"

//#include "PreferenceWriter.h"


#define V_CAL 0.15f;

void order_phases(AngularPositionSensorStruct* encoderStruct, ControllerStruct *controller);// PreferenceWriter *prefs);
//void calibrate(AngularPositionSensorStruct* encoderStruct, ControllerStruct *controller);// PreferenceWriter *prefs);
#endif
