#ifndef UART_H_
#define UART_H_

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "../motor/motor-controller.h"

/**
 * @name Response codes
 * @brief Return values from `parseCommand` indicating response type.
 */
 #define DEFAULT_RESPONSE 0
#define MA_RESPONSE 1
#define MF_RESPONSE 3

/**
 * @brief Initialize UART peripheral at given baud rate.
 * @param baud Baud rate (e.g., 9600, 115200)
 */
void uartInit(unsigned long baud);

/**
 * @brief Send a single character over UART (blocking until ready).
 * @param c Character to send.
 */
void uartSendChar(char c);

/**
 * @brief Send an unsigned integer as ASCII followed by CR+LF.
 * @param value Value to send (0..65535).
 */
void uartSendInt(uint16_t value);

/**
 * @brief Enable UART RX (receive) hardware.
 */
void uartEnableRX(void);

/**
 * @brief Blocking read of a received UART character.
 * @return Received character from UDR0.
 */
char uartReadChar(void);

/**
 * @brief Parse a received command string and execute actions.
 * @param cmd Null-terminated command string (no CR/LF)
 * @return One of `MA_RESPONSE` or `MF_RESPONSE` indicating response category.
 */
uint8_t parseCommand(char *cmd);

/**
 * @brief Polling task that accumulates incoming characters into a buffer
 * and dispatches complete lines to `parseCommand` when CR or LF is received.
 * * @return One of `MA_RESPONSE` or `MF_RESPONSE` indicating response category.
 */
uint8_t uartCommandTask(void);

#endif /* UART_H_ */