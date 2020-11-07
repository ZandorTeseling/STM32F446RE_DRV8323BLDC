/*
 * string_process.c
 *
 *  Created on: Oct 12, 2020
 *      Author: zandor
 */

#include <stdio.h>
#include <../Inc/string_process.h>

extern CircularBuffer gRX_buffer;
IOTokenised gUART_Token;

void InitCommandLineProcess(){
	gUART_Token.io_head = -1;
}

void CommandLineProcess(){

  if(gRX_buffer.echoed){
	  gRX_buffer.echoed = 0;
	  putchar(gRX_buffer.buffer[gRX_buffer.head]);

	  gUART_Token.io_head++;
	  gUART_Token.io_s[gUART_Token.io_head] = gRX_buffer.buffer[gRX_buffer.head];
  }


  /*Decide if we need to process the string*/
  if(gUART_Token.io_s[gUART_Token.io_head] == 13 || gUART_Token.io_head == CIRCBUFFSIZE){
	  printf("\nCapacity Reached or Enter Pressed: %i\n", gUART_Token.io_head);
	  gUART_Token.io_s[gUART_Token.io_head] = '\0';
	  printf("\n\n\rInput String: %s \n\r", &gUART_Token.io_s);
	  gUART_Token.io_head = -1;
  }


}
