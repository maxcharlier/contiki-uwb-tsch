#include "contiki.h"
#include "examples/zolertia/zoul/dw1000/localisation-mobility/libs/byte-stuffing.h"


struct stuffed_bytes byte_stuffing_encode(uint8_t *frame, int length) {
    /*
     * Max size : All  bytes are escaped (2 * MAX_PAYLOAD_LEN) + start and end flag 
     * Should probably use a malloc.
     */
    uint8_t buffer[2 * MAX_PAYLOAD_LEN + 2];
    uint8_t *buffer_ptr = buffer;

    // Add the flag at the begining of the frame
    *buffer_ptr++ = BS_SFD;
    
    uint8_t *input_bytes = frame;
    uint8_t *end = frame + length;

    while (input_bytes < end) {
        uint8_t b = *input_bytes;
        if (b == BS_SFD || b == BS_ESC || b == BS_EFD) {
            *buffer_ptr++ = BS_ESC;
        }
        *buffer_ptr++ = *input_bytes++;
    }

    // Add the flag at the end of the frame
    *buffer_ptr = BS_EFD;

    struct stuffed_bytes result = {buffer, buffer_ptr - buffer};
    return result;
    

};