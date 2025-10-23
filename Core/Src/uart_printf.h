/*
 * uart_printf.h
 *
 *  Created on: Oct 15, 2025
 *      Author: Asus
 */

#ifndef SRC_UART_PRINTF_H_
#define SRC_UART_PRINTF_H_
#include <stdint.h>
void uart_start_receive(void);
uint8_t serial_available(void);
uint8_t serial_read(void);
#endif /* SRC_UART_PRINTF_H_ */
