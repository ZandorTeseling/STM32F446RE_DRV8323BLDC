/*
 * retarget.c
 *
 *  Created on: Jul 30, 2020
 *      Author: zandor
 */


// All credit to Carmine Noviello for this code
// https://github.com/cnoviello/mastering-stm32/blob/master/nucleo-f030R8/system/src/retarget/retarget.c

#include <_ansi.h>
#include <_syslist.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/times.h>
#include <limits.h>
#include <signal.h>
#include <../Inc/retarget.h>
#include <stdint.h>
#include <stdio.h>

#if !defined(OS_USE_SEMIHOSTING)

#define STDIN_FILENO  0
#define STDOUT_FILENO 1
#define STDERR_FILENO 2

UART_HandleTypeDef *gHuart;
CircularBuffer gRX_buffer = { { '\0' }, -1, -1, 0};
CircularBuffer gTX_buffer = { { '\0' },  0,  0, 0};

CircularBuffer *_gRX_buffer;
CircularBuffer *_gTX_buffer;

void RetargetInit(UART_HandleTypeDef *huart) {
  gHuart = huart;
  _gRX_buffer = & gRX_buffer;
  _gTX_buffer = & gTX_buffer;
  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  __HAL_UART_ENABLE_IT(gHuart, UART_IT_ERR);

  /* Enable the UART Data Register not empty Interrupt */
  __HAL_UART_ENABLE_IT(gHuart, UART_IT_RXNE);



  /* Disable I/O buffering for STDOUT stream, so that
   * chars are sent out as soon as they are printed. */
  setvbuf(stdout, NULL, _IONBF, 0);
}

void My_USART_IQRHandler(UART_HandleTypeDef *huart)
{
	//need a transmit buffer & receive buffer.
	uint32_t ISRFlags = READ_REG(huart->Instance->SR);
	uint32_t CR1Flags = READ_REG(huart->Instance->CR1);

    /* if DR is not empty and the Rx Int is enabled */
    if (((ISRFlags & USART_SR_RXNE) != RESET) && ((CR1Flags & USART_CR1_RXNEIE) != RESET))
    {
		huart->Instance->SR;              /* Read status register */
        char c = huart->Instance->DR;     /* Read data register */
        gRX_buffer.echoed = 1; //Signal an echo is required.
        gRX_buffer.head++;
        gRX_buffer.head = gRX_buffer.head % CIRCBUFFSIZE;
        gRX_buffer.buffer[gRX_buffer.head] = c;
        return;
    }

	/*If interrupt is caused due to Transmit Data Register Empty */
	if (((ISRFlags & USART_SR_TXE) != RESET) && ((CR1Flags & USART_CR1_TXEIE) != RESET))
	{
		if(gTX_buffer.head + 1 == gTX_buffer.tail){
		  __HAL_UART_DISABLE_IT(huart, UART_IT_TXE); //Disable Interrupt
		}
		else{
		  // There is more data in the output buffer. Send the next byte
		  unsigned char c = gTX_buffer.buffer[gTX_buffer.tail];
		  gTX_buffer.tail++;
		  gTX_buffer.tail = gTX_buffer.tail  % CIRCBUFFSIZE;
		  huart->Instance->SR;
		  huart->Instance->DR = c;
		}
		return;
	}
}


int _isatty(int fd) {
  if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
    return 1;

  errno = EBADF;
  return 0;
}

int _write(int fd, char* ptr, int len) {
  if (fd == STDOUT_FILENO || fd == STDERR_FILENO) {
	  for(int i = 0; i < len; i++){
		  gTX_buffer.head++; //Increment Head
		  gTX_buffer.head = gTX_buffer.head % CIRCBUFFSIZE; //Wrap Head back around if need be.
		  gTX_buffer.buffer[gTX_buffer.head] = ptr[i];
	  }
	  __HAL_UART_ENABLE_IT(gHuart, UART_IT_TXE);
	  return len;
  }
  errno = EBADF;
  return -1;
}

int _close(int fd) {
  if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
    return 0;

  errno = EBADF;
  return -1;
}

int _lseek(int fd, int ptr, int dir) {
  (void) fd;
  (void) ptr;
  (void) dir;

  errno = EBADF;
  return -1;
}

int _read(int fd, char* ptr, int len) {
//  HAL_StatusTypeDef hstatus;

  if (fd == STDIN_FILENO) {
//    hstatus = HAL_UART_Receive(gHuart, (uint8_t *) ptr, 1, HAL_MAX_DELAY);
//    if (hstatus == HAL_OK)
      return 1;
//    else
//      return EIO;
  }
  errno = EBADF;
  return -1;
}

int _fstat(int fd, struct stat* st) {
  if (fd >= STDIN_FILENO && fd <= STDERR_FILENO) {
    st->st_mode = S_IFCHR;
    return 0;
  }

  errno = EBADF;
  return 0;
}

#endif //#if !defined(OS_USE_SEMIHOSTING)


