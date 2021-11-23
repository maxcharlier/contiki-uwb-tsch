#ifndef __UART_DEBUG_HANDLER_H__
#define __UART_DEBUG_HANDLER_H__
#include "contiki.h"


#define MAX_SERIAL_LEN    50

extern uint8_t uart_debug_receive_buffer0[MAX_SERIAL_LEN+1];
extern uint8_t uart_debug_receive_buffer1[MAX_SERIAL_LEN+1];

int
uart_receive_byte(unsigned char c);

/********** UART Debug Handler Process *********/
PROCESS_NAME(uart_debug_handler_process);


#endif /* __UART_DEBUG_HANDLER_H__ */