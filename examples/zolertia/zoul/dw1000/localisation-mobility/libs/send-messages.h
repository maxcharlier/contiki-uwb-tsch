#include "contiki.h"

static struct tsch_slotframe *tsch_slotframe;
static struct ctimer *retry_timer;

void
debug_send_allocation_probe_request();

void
send_allocation_probe_request(uip_ipaddr_t *rpl_child_ip);

void
receive_uart(uint8_t *pkt, int length);

void
send_to_central_authority(void *data_to_transmit, int length);

void 
act_on_message(uint8_t *msg, int length);

int
uart_receive_byte(unsigned char c);