#include "contiki.h"

#define STATE_WAIT_SFD      1
#define STATE_READ_DATA     2
#define STATE_READ_ESC_DATA 3

#define BS_SFD 0xBB
#define BS_EFD 0xEE
#define BS_ESC 0x33

struct stuffed_bytes {
    uint8_t *bytes;
    int length;
};

/**
 *  Note: destination should have at least a size of 2 * length + 2
 * 
 *  Returns the amount of bytes written to destination 
 */
int
byte_stuffing_encode(uint8_t *frame, int length, void *destination);

void
byte_stuffing_decode(uint8_t *frame, int length, void *destination);

