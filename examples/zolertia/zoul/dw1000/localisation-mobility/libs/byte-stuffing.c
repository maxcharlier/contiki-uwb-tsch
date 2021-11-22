#include "contiki.h"
#include "examples/zolertia/zoul/dw1000/localisation-mobility/libs/byte-stuffing.h"

#include "dev/uart.h"
#define write_byte(b) uart_write_byte(DBG_CONF_UART, b)


#define UART_OUTPUT     1
#define UART_DEBUG      0


int 
byte_stuffing_encode(uint8_t *frame, int length, void *destination) {
    /*
     * Max size : All  bytes are escaped (2 * MAX_PAYLOAD_LEN) + start and end flag 
     * Should probably use a malloc.
     */
    uint8_t *start = destination;
    uint8_t *buffer_ptr = destination;

    // Add the flag at the begining of the frame
    *buffer_ptr++ = BS_SFD;
    
    uint8_t *input_bytes = frame;

    while (input_bytes < frame + length) {
        uint8_t b = *input_bytes;
        if (b == BS_SFD || b == BS_ESC || b == BS_EFD) {
            *buffer_ptr++ = BS_ESC;
        }
        *buffer_ptr++ = *input_bytes++;
    }

    // Add the flag at the end of the frame
    *buffer_ptr++ = BS_EFD;
    return buffer_ptr - start;
}

int
byte_stuffing_send_bytes(uint8_t *frame, int length) {
    
    uart_write_byte(UART_OUTPUT, BS_SFD);
    for (int i=0; i<length; i++) {
        uint8_t b = frame[i];
        if (b == BS_SFD || b == BS_ESC || b == BS_EFD) {
            uart_write_byte(UART_OUTPUT, BS_ESC);
        }
        uart_write_byte(UART_OUTPUT, b);
    }
    uart_write_byte(UART_OUTPUT, BS_EFD);
    return 0;
}