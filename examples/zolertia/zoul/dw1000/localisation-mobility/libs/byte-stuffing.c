#include "contiki.h"
#include "examples/zolertia/zoul/dw1000/localisation-mobility/libs/byte-stuffing.h"


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