#include "contiki.h"

#define UART_OUTPUT     1
#define UART_DEBUG      0

static struct tsch_slotframe *tsch_slotframe;

void
debug_send_allocation_probe_request();

uip_ipaddr_t *
query_best_anchor();

void
send_allocation_probe_request();

void
receive_uart(uint8_t *pkt, int length);

void
send_to_central_authority(void *data_to_transmit, int length);

void 
act_on_message(uint8_t *msg, int length);

int
uart_receive_byte(unsigned char c);

void
uart_send_bytes(void *data_to_transmit, int length);


void
handle_propagation(struct tsch_neighbor *data);