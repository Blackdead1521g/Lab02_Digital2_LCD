/* 
 * File:   USART.h
 * Author: Black
 *
 * Created on 27 de julio de 2023, 03:44 PM
 */

#include <stdio.h>
#include <xc.h>
#include <stdint.h>
#include <stdlib.h>
#include <pic16f887.h>

#ifndef USART_H
#define	USART_H

void UART_RX_config (uint16_t baudrate);
void UART_TX_config (uint16_t baudrate);
void UART_write_char (char txt[]);
char UART_read_char ();

#endif	/* USART_H */

