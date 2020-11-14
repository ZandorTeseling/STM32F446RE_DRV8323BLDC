/*
 * string_process.h
 *
 *  Created on: Oct 12, 2020
 *      Author: zandor
 */

#ifndef INC_STRING_PROCESS_H_
#define INC_STRING_PROCESS_H_

#include <../Inc/retarget.h>
#include <../Config/user_config.h>

typedef struct{
	unsigned char io_s[CIRCBUFFSIZE+1];
	signed int io_head; //Hold the location of what has been echoed from terminal.
} IOTokenised;

void InitCommandLineProcess();
void CommandLineProcess();

#endif /* INC_STRING_PROCESS_H_ */
