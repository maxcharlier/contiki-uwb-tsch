#include "contiki.h"

#include "net/mac/tsch/tsch.h"
#include "net/mac/tsch/tsch-schedule.h"
#include "net/ipv6/uip-ds6.h"
#include "net/rpl/rpl.h"

#include "dev/uart.h"
#define write_byte(b) uart_write_byte(DBG_CONF_UART, b)

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"
#include "core/net/ip/uip-udp-packet.h"

#include "examples/zolertia/zoul/dw1000/localisation-mobility/libs/message-formats.h"
#include "examples/zolertia/zoul/dw1000/localisation-mobility/libs/byte-stuffing.h"
#include "examples/zolertia/zoul/dw1000/localisation-mobility/libs/send-messages.h"
#include "examples/zolertia/zoul/dw1000/localisation-mobility/libs/schedule.h"


static const uip_ipaddr_t null_attached_anchor;
static uip_ipaddr_t current_attached_anchor;

#define UDP_CLIENT_PORT 8765
#define UDP_SERVER_PORT 5678

#define IS_LOCATION_SERVER 1

#define MAX_SERIAL_LEN    100

static int state = STATE_WAIT_SFD;
static uint8_t receive_buffer[MAX_SERIAL_LEN];
static uint8_t *receive_ptr = receive_buffer;

/*
#define PRINT_BYTE 0
#undef PRINTF
#if !PRINT_BYTE
  #define PRINTF(...) printf(__VA_ARGS__)
#else
  #define PRINTF(...) do {} while(0)
#endif
*/

#define UART_WRITE_STRING(output, text) uart_write_string(output, text, sizeof(text))

//#define SEND_TO_CENTRAL_AUTHORITY(data) send_to_central_authority(&(data), sizeof(data))


uip_ipaddr_t *
query_best_anchor()
{
  // Take the fist parent in the table
  rpl_parent_t *p = nbr_table_head(rpl_parents);
  return rpl_get_parent_ipaddr(p);
}


void
uart_write_string(int output, char text[], int size)
{
  for (int i=0; i<size; i++) {
    uart_write_byte(output, text[i]);
  }
}




allocation_request get_allocation_request() {

  uip_ipaddr_t *rpl_parent_ptr = query_best_anchor();
  uip_ipaddr_t rpl_parent;
  uip_ipaddr_t our_ip = uip_ds6_get_global(ADDR_PREFERRED)->ipaddr; // Could also be : uip_ds6_get_link_local()

  if (!rpl_parent_ptr) {
    // There is no RPL parent yet -> error
    UART_WRITE_STRING(UART_DEBUG, "ERROR: No PRL parent is available\n");
    // For now, only send our own IP as a parent.
    rpl_parent = our_ip; 
  } else {
    rpl_parent = *rpl_parent_ptr;
  }

  allocation_request rqst = { 
      ALLOCATION_REQUEST,
      255,  // signal power
      0,
      rpl_parent,
      our_ip
  };

  UART_WRITE_STRING(UART_DEBUG, "parent, our_ip: \n");
  PRINT6ADDR(&rpl_parent);
  UART_WRITE_STRING(UART_DEBUG, "\n");
  PRINT6ADDR(&our_ip);
  UART_WRITE_STRING(UART_DEBUG, "\n");

  return rqst;

}

void
send_allocation_probe_request()
{
  
  
  //if (!rpl_parent) {
    // No parent to send a probe request to.
    // Wait for RPL to find a parent.
    //PRINTF("APP: No parent, retrying in 1s\n");

    //goto retry;
  //}

  if (0) {
    /*
     *  Either there is a new parent, or the RPL parent changed.
     *  Send a request to receive a new cell, then unsubscribe from the current cell
     * 
     *  || !memcmp(&current_attached_anchor, &null_attached_anchor, sizeof(uip_ipaddr_t))     // TODO remove true
     *  || !memcmp(rpl_parent_ip, &current_attached_anchor, sizeof(uip_ipaddr_t))
     */
    PRINTF("APP: Leaf-only: %i\n", RPL_LEAF_ONLY);
    PRINTF("APP: RPL Parent changed, requesting a change of geolocation cell\n");

    // UART_WRITE_STRING(UART_DEBUG, "mobile_addr ->"); PRINT6ADDR(rpl_parent); UART_WRITE_STRING(UART_DEBUG, "\n");
    allocation_request rqst = get_allocation_request();
    send_to_central_authority(&rqst, sizeof(rqst));

  }

  ctimer_set(&retry_timer, 1*(CLOCK_SECOND), send_allocation_probe_request, &retry_timer);    
}

void
send_to_central_authority(void *data_to_transmit, int length)
{

#if IS_LOCATION_SERVER

  // An anchor has direct UART connection to the central authority.
  
  uart_send_bytes(data_to_transmit, length);

#else /* IS_LOCATION_SERVER */

  // A mobile needs to send packets wirelessly to an (ideally the nearest) anchor,
  // which will forward it to the central autority.

  // Initiate a connection.
  // Remark: Ideally, an anchor should maintain constantly a connection with its nearest anchor.
  uip_ipaddr_t *nearest_anchor_ip = query_best_anchor();
  struct uip_udp_conn *server_conn = udp_new(nearest_anchor_ip, UIP_HTONS(UDP_SERVER_PORT), NULL); 
  
  if (server_conn == NULL) {
    UART_WRITE_STRING(UART_DEBUG, "No UDP connection available, exiting the process!\n");
    return;
  }

  udp_bind(server_conn, UIP_HTONS(UDP_CLIENT_PORT)); 

  // For debugging purposes
  PRINTF("Created a connection with the server ");
  PRINT6ADDR(&server_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n",
	UIP_HTONS(server_conn->lport), UIP_HTONS(server_conn->rport));


  PRINTF("Data Sent to the remote server\n");
  uip_udp_packet_sendto(server_conn, data_to_transmit, length,
                        nearest_anchor_ip, UIP_HTONS(UDP_SERVER_PORT));

#endif

}

void
uart_send_bytes(void *data_to_transmit, int length)
{
  uint8_t stuffed_bytes[2 * MAX_SERIAL_LEN + 2];
  int length_to_write = byte_stuffing_encode(data_to_transmit, length, stuffed_bytes);

  uint8_t *current_ptr = stuffed_bytes;
  uint8_t *end = current_ptr + length_to_write;
  while (current_ptr < end) {
    uart_write_byte(UART_OUTPUT, *current_ptr);
    current_ptr += 1;
  }
}

int
uart_receive_byte(unsigned char c)
{
    uint8_t byte = c;

    uart_write_byte(UART_DEBUG, byte);

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
receive_uart(uint8_t *pkt, int length)
{
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
        
        UART_WRITE_STRING(UART_DEBUG, "ack\n");

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
act_on_message(uint8_t *msg, int length)
{
  uart_write_byte(UART_DEBUG, '0' + state);
  uart_write_byte(UART_DEBUG, '0' + *msg);

  switch (*msg) {

    case CLEAR_SLOTFRAME: ;

      UART_WRITE_STRING(UART_DEBUG, "clear\n");

      // tsch_schedule_remove_all_slotframes();
      // tsch_schedule_remove_slotframe(tsch_slotframe);
      // tsch_slotframe = tsch_schedule_add_slotframe(0, 31);
      // tsch_slotframe = tsch_schedule_create_initial();

      uip_ipaddr_t our_ip = uip_ds6_get_global(ADDR_PREFERRED)->ipaddr;

      clear_ack clearack = {
        CLEAR_ACK,
        our_ip
      };

      send_to_central_authority(&clearack, sizeof(clearack));
      //SEND_TO_CENTRAL_AUTHORITY(clear_ack);

      /*
       * After a Clear slotframe, try to join the network via an ACK if the node is a mobile
       */

#if NODEID != 0x07    // TODO if !IS_LOCATION_SERVER

      allocation_request rqst = get_allocation_request();
      send_to_central_authority(&rqst, sizeof(rqst));
      //SEND_TO_CENTRAL_AUTHORITY(rqst);

#endif /* NODEID != 0X7 */

      break;

    case ALLOCATION_SLOT: ;

      UART_WRITE_STRING(UART_DEBUG,  "add\n");

      allocation_slot packet = *(allocation_slot *)(msg);

      PRINT6ADDR(&packet.mobile_addr);
      UART_WRITE_STRING(UART_DEBUG, "\n");
      PRINT6ADDR(&packet.anchor_addr);
      UART_WRITE_STRING(UART_DEBUG, "\n");


#if IS_LOCATION_SERVER

      linkaddr_t *mobile_addr = (linkaddr_t *) uip_ds6_nbr_lladdr_from_ipaddr(&(packet.mobile_addr));
      tsch_schedule_add_link(tsch_slotframe, LINK_OPTION_TX, LINK_TYPE_NORMAL, mobile_addr, packet.timeslot, packet.channel);

#else /* IS_LOCATION_SERVER */
    
      linkaddr_t *anchor_addr = (linkaddr_t *) uip_ds6_nbr_lladdr_from_ipaddr(&(packet.anchor_addr));
      tsch_schedule_add_link(tsch_slotframe, LINK_OPTION_TX, LINK_TYPE_NORMAL, anchor_addr, packet.timeslot, packet.channel);

#endif /* IS_LOCATION_SERVER */

      // Successfully added slot, send ack
      allocation_ack ack = {
        ALLOCATION_ACK,
        0,    // padding
        packet.timeslot,
        packet.channel,
        packet.mobile_addr,
        packet.anchor_addr
      };

      send_to_central_authority(&ack, sizeof(ack));
      break;

    case DEALLOCATION_SLOT: ;

      UART_WRITE_STRING(UART_DEBUG, "del\n");

      deallocation_slot pkt = *(deallocation_slot *)(msg);
      struct tsch_link *to_delete = tsch_schedule_get_link_by_timeslot(tsch_slotframe, pkt.timeslot);
      tsch_schedule_remove_link(tsch_slotframe, to_delete);
      break;


    default: ;

      UART_WRITE_STRING(UART_DEBUG, "frame ID does not match any known frame types.\n");

      break;
  }

}