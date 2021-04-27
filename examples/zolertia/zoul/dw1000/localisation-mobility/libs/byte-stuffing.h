#include "contiki.h"

#define MAX_PAYLOAD_LEN   30

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

struct stuffed_bytes byte_stuffing_encode(uint8_t *frame, int length);

uint8_t *byte_stuffing_decode(uint8_t *frame, int length);

