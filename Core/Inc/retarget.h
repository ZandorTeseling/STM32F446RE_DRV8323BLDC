/*
 * retarget.h
 *
 *  Created on: Jul 30, 2020
 *      Author: zandor
 */

#ifndef INC_RETARGET_H_
#define INC_RETARGET_H_

#include "stm32f4xx_hal.h"
#include <sys/stat.h>
#include <stdio.h>
#include <../Config/user_config.h>
//USART 1 RingBuffer Settings
#define CIRCBUFFSIZE 100

typedef struct
{
  unsigned char buffer[CIRCBUFFSIZE];
  volatile signed int head;
  volatile signed int tail;
  volatile unsigned int echoed;
} CircularBuffer;

void RetargetInit(UART_HandleTypeDef *huart);
void My_USART_IQRHandler(UART_HandleTypeDef *huart);
int _myputchar(char* c);
int _isatty(int fd);
int _write(int fd, char* ptr, int len);
int _close(int fd);
int _lseek(int fd, int ptr, int dir);
int _read(int fd, char* ptr, int len);
int _fstat(int fd, struct stat* st);

#endif /* INC_RETARGET_H_ */
