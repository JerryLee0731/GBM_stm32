#ifndef UART_PC_H_
#define UART_PC_H_

#include "mbed.h"

void uart_pc(BufferedSerial& serial_port, char buffer[32]);

#endif // UART_PC_H_