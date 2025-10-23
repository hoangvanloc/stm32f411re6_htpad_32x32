/*
 * uart_printf.c
 *
 *  Created on: Oct 15, 2025
 *      Author: Asus
 */

#include "uart_printf.h"
#include "usart.h"
#include "main.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#define uart_io    huart2
extern DMA_HandleTypeDef hdma_usart2_rx;
#define dma_usart_rx_io  hdma_usart2_rx
#define UART_BUF_SIZE   5
typedef struct
{
	uint8_t buf_store[UART_BUF_SIZE];
	uint8_t buf_dma[UART_BUF_SIZE];
	uint8_t buf_count_read;
	uint8_t buf_count_total;
}uart_t;

uart_t uart = {0};

void uart_start_receive(void)
{
	// Start UART reception in ReceiveToIdle mode
	HAL_UARTEx_ReceiveToIdle_DMA(&uart_io, uart.buf_dma, UART_BUF_SIZE);
	__HAL_DMA_DISABLE_IT(&dma_usart_rx_io, DMA_IT_HT); // Optional: disable half-transfer interrupt
}

uint8_t serial_available(void)
{
  return uart.buf_count_read < uart.buf_count_total;
}

uint8_t serial_read(void)
{
	uint8_t dat = 0;
	//Read all character in buff -> start new receive buf
  if(uart.buf_count_read < uart.buf_count_total)
  {
	  dat =  uart.buf_store[uart.buf_count_read];
	  uart.buf_count_read ++;
  }
  if(uart.buf_count_total >= UART_BUF_SIZE && uart.buf_count_read == uart.buf_count_total) //Check full buff and read out all data
  {
	  uart.buf_count_read = 0;
	  uart.buf_count_total = 0;
  }
  return dat;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if(huart == &uart_io)
  {
	  int16_t remain_len = UART_BUF_SIZE - uart.buf_count_total;
	  if(remain_len >= Size)
	  {
		  memcpy(&uart.buf_store[uart.buf_count_total], uart.buf_dma, Size);
		  uart.buf_count_total += Size;
	  }else if(remain_len > 0)
	  {
		  memcpy(&uart.buf_store[uart.buf_count_total], uart.buf_dma, Size - remain_len);
		  uart.buf_count_total += (Size - remain_len);
	  }
  }
  // Restart reception
  HAL_UARTEx_ReceiveToIdle_DMA(&uart_io, uart.buf_dma, UART_BUF_SIZE);
}

void _putchar(char character)
{
	HAL_UART_Transmit(&uart_io,(uint8_t *)&character, 1, 10);
}

void __io_putchar(uint8_t ch)
{
	HAL_UART_Transmit(&uart_io, &ch, 1, HAL_MAX_DELAY);
}

int _write(int file, char *ptr, int len)
{
	(void)file;
	 int DataIdx;

	  for (DataIdx = 0; DataIdx < len; DataIdx++)
	  {
	    __io_putchar(*ptr++);
	  }
	 setbuf(stdout, NULL);
	 return len;
}


