/*
 * Protocol.h
 *
 *  Created on: 27 ม.ค. 2565
 *      Author: SakuranohanaTH
 */

#ifndef INC_PROTOCOL_H_
#define INC_PROTOCOL_H_

extern uint16_t BUFFER_SIZE = 0;
extern uint8_t UART_STLINK_PACKAGE[BUFFER_SIZE] = {0};

void uart_send(uint8_t *buff);
void uart_send_dma(uint8_t *buff);

void uart_read(uint8_t *buff);
void uart_read_dma(uint8_t *buff);

#endif /* INC_PROTOCOL_H_ */
