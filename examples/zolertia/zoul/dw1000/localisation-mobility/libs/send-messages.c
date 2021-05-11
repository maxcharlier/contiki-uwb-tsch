#include "contiki.h"

#include "net/rpl/rpl.h"
#include "net/mac/tsch/tsch.h"
#include "net/mac/tsch/tsch-schedule.h"

#include "dev/uart.h"
#define write_byte(b) uart_write_byte(DBG_CONF_UART, b)

#include "examples/zolertia/zoul/dw1000/localisation-mobility/libs/message-formats.h"
#include "examples/zolertia/zoul/dw1000/localisation-mobility/libs/byte-stuffing.h"
#include "examples/zolertia/zoul/dw1000/localisation-mobility/libs/send-messages.h"

static const uip_ipaddr_t null_attached_anchor;
static uip_ipaddr_t current_attached_anchor;

#define IS_LOCATION_SERVER 0

#define MAX_SERIAL_LEN    100

static int state = STATE_WAIT_SFD;
static uint8_t receive_buffer[MAX_SERIAL_LEN];
static uint8_t *receive_ptr = receive_buffer;


#define PRINT_BYTE 1
#undef PRINTF
#if !PRINT_BYTE
  #define PRINTF(...) printf(__VA_ARGS__)
#else
  #define PRINTF(...) do {} while(0)
#endif



void
send_allocation_probe_request(void *ptr)
{
  PRINTF("APP: Leaf-only: %i\n", RPL_LEAF_ONLY);
  
  // Check if a RPL parent is present
  rpl_parent_t *rpl_parent = nbr_table_head(rpl_parents);
  
  //if (!rpl_parent) {
    // No parent to send a probe request to.
    // Wait for RPL to find a parent.
    //PRINTF("APP: No parent, retrying in 1s\n");

    //goto retry;
  //}

  uip_ipaddr_t mobile_ip = uip_ds6_get_global(ADDR_PREFERRED)->ipaddr; // Could also be : uip_ds6_get_link_local()
  uip_ipaddr_t *rpl_parent_ip = rpl_get_parent_ipaddr(rpl_parent);

  if (1 || !memcmp(&current_attached_anchor, &null_attached_anchor, sizeof(uip_ipaddr_t))     // TODO remove true
      || !memcmp(rpl_parent_ip, &current_attached_anchor, sizeof(uip_ipaddr_t))) {
    /*
     *  Either there is a new parent, or the RPL parent changed.
     *  Send a request to receive a new cell, then unsubscribe from the current cell
     */
    PRINTF("APP: RPL Parent changed, requesting a change of geolocation cell\n");
    
    allocation_request rqst = { 
      ALLOCATION_REQUEST,
      255,  // signal power
      mobile_ip,
      *rpl_parent_ip
    };

    send_to_central_authority(&rqst, sizeof(rqst));
  }

retry:
  ctimer_set(&retry_timer, 1*(CLOCK_SECOND), send_allocation_probe_request, &retry_timer);    
}

void
send_to_central_authority(void *data_to_transmit, int length)
{
  uint8_t stuffed_bytes[2 * MAX_SERIAL_LEN + 2];
  int length_to_write = byte_stuffing_encode(data_to_transmit, length, stuffed_bytes);

  uint8_t *current_ptr = stuffed_bytes;
  uint8_t *end = current_ptr + length_to_write;
  while (current_ptr < end) {
    write_byte(*current_ptr);
    current_ptr += 1;
  }
}

int
uart_receive_byte(unsigned char c) {
    uint8_t byte = c;

    uart_write_byte(0, state);

    switch (state){

    case STATE_WAIT_SFD:
      if (byte == BS_SFD) {
        state = STATE_READ_DATA;
      }
      break;
    
    case STATE_READ_DATA:
      if (byte == BS_EFD) {
        act_on_message(receive_buffer, receive_ptr - receive_buffer);

        // Reset buffer for future use
        receive_ptr = receive_buffer;
        state = STATE_WAIT_SFD;
      } else if (byte == BS_ESC) {
        state = STATE_READ_ESC_DATA;
      } else {
        *receive_ptr++ = byte;
      }
      break;
    
    case STATE_READ_ESC_DATA:
        *receive_ptr++ = byte;
        state = STATE_READ_DATA;
      break;
    
    default:
      break;
    }
}

void
receive_uart(uint8_t *pkt, int length) {

  uint8_t *ptr = pkt;
  while (ptr < pkt + length) {
    uint8_t byte = *ptr;

    switch (state){

    case STATE_WAIT_SFD:
      if (byte == BS_SFD) {
        state = STATE_READ_DATA;
      }
      break;
    
    case STATE_READ_DATA:
      if (byte == BS_EFD) {
        act_on_message(receive_buffer, receive_ptr - receive_buffer);

        // Empty the buffer for future use
        receive_ptr = receive_buffer;
      } else if (byte == BS_ESC) {
        state = STATE_READ_ESC_DATA;
      } else {
        *receive_ptr++ = byte;
      }
      break;
    
    case STATE_READ_ESC_DATA:
        *receive_ptr++ = byte;
      break;
    
    default:
      break;
    }

    ptr++;
  }
}

void
act_on_message(uint8_t *msg, int length) {

  uart_write_byte(0, 'Q');
  uart_write_byte(0, *msg);
  
  switch (*msg) {


    case CLEAR_SLOTFRAME: ;

      tsch_schedule_remove_slotframe(tsch_slotframe);
      tsch_slotframe = tsch_schedule_add_slotframe(0, 31);

      clear_ack clearack = {
        CLEAR_ACK,
      };

      send_to_central_authority(&clearack, sizeof(clearack));
      break;


    case ALLOCATION_SLOT: ;
      allocation_slot packet = *(allocation_slot *)(msg);

      uart_write_byte(0, 'r');

#if IS_LOCATION_SERVER == 1

      linkaddr_t *mobile_addr = (linkaddr_t *) uip_ds6_nbr_lladdr_from_ipaddr(&(packet.mobile_addr));
      tsch_schedule_add_link(tsch_slotframe, LINK_OPTION_TX, LINK_TYPE_NORMAL, mobile_addr, packet.timeslot, packet.channel);

#else /* IS_LOCATION_SERVER */
    
      linkaddr_t *anchor_addr = (linkaddr_t *) uip_ds6_nbr_lladdr_from_ipaddr(&(packet.anchor_addr));
      tsch_schedule_add_link(tsch_slotframe, LINK_OPTION_TX, LINK_TYPE_NORMAL, anchor_addr, packet.timeslot, packet.channel);

#endif /* IS_LOCATION_SERVER */

      // Successfully added slot, send ack
      allocation_ack ack = {
        ALLOCATION_ACK,
        packet.mobile_addr,
        packet.anchor_addr,
        packet.timeslot,
        packet.channel
      };

      send_to_central_authority(&ack, sizeof(ack));
      break;

    case DEALLOCATION_SLOT: ;
      deallocation_slot pkt = *(deallocation_slot *)(msg);
      struct tsch_link *to_delete = tsch_schedule_get_link_by_timeslot(tsch_slotframe, pkt.timeslot);
      tsch_schedule_remove_link(tsch_slotframe, to_delete);
      break;


    default: ;

      break;
  }

}