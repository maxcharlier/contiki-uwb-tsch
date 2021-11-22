#include "contiki.h"

#include "net/mac/tsch/tsch.h"
#include "net/mac/tsch/tsch-schedule.h"
#include "net/mac/tsch/tsch-queue.h"
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
#include "examples/zolertia/zoul/dw1000/localisation-mobility/libs/schedule-onecell3A1T.h"


// static const uip_ipaddr_t null_attached_anchor;
// static uip_ipaddr_t current_attached_anchor;

#define UDP_CLIENT_PORT 8765
#define UDP_SERVER_PORT 5678



#define MAX_SERIAL_LEN    100

static int state = STATE_WAIT_SFD;
static uint8_t receive_buffer[MAX_SERIAL_LEN];
static uint8_t *receive_ptr = receive_buffer;

#define PRINT_BYTE 0
#undef PRINTF
#if !PRINT_BYTE
  #define PRINTF(...) printf(__VA_ARGS__)
#else
  #define PRINTF(...) do {} while(0)
#endif


void
uart_write_string(int output, char text[], int size)
{
  for (int i=0; i<size; i++) {
    uart_write_byte(output, text[i]);
  }
}


#define UART_WRITE_STRING(output, text) uart_write_string(output, text, sizeof(text))

//#define SEND_TO_CENTRAL_AUTHORITY(data) send_to_central_authority(&(data), sizeof(data))


linkaddr_t
get_linkaddr_from_ipaddr(uip_ip6addr_t *ipaddr)
{
  // linkaddr_copy
  linkaddr_t *from_ip = (linkaddr_t *) uip_ds6_nbr_lladdr_from_ipaddr(ipaddr);
  
  if (!from_ip) {
    // Failed to fetch address
    UART_WRITE_STRING(UART_DEBUG, "Failed to fetch linkaddr_t.\n");

    uip_ds6_nbr_t *nbr = uip_ds6_nbr_lookup(ipaddr);
    if (!nbr) {
      UART_WRITE_STRING(UART_DEBUG, "Failed to get nbr. \n");
    }

    // Hack some other way to get the address
    int last_digits = ipaddr->u8[15];
    linkaddr_t link_address  = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, last_digits } };
    return link_address;
  }
  // PRINTLLADDR(link_address);
  linkaddr_t link_address;
  linkaddr_copy(&link_address, from_ip);
  return link_address;
}


uip_ipaddr_t
get_ipaddr_from_linkaddr(linkaddr_t *linkaddress)
{
  uip_lladdr_t *lladdr = (uip_lladdr_t *) linkaddress; 
  // PRINTLLADDR(lladdr);
  uip_ipaddr_t *from_addr = uip_ds6_nbr_ipaddr_from_lladdr(lladdr);
  // PRINT6ADDR(from_addr);
  uip_ipaddr_t ip_address;
  uip_ipaddr_copy(&ip_address, from_addr);
  return ip_address;
}


#define uip_ip6addr_cmp_ed(addr1, addr2) (memcmp((addr1)+1, (addr2)+1, sizeof(uip_ip6addr_t)-1) == 0)  // GCC Only : using sizeof(void) == 1



allocation_request get_allocation_request() {

  uip_ipaddr_t rpl_parent = query_best_anchor();
  uip_ipaddr_t our_ip = uip_ds6_get_global(ADDR_PREFERRED)->ipaddr; // Could also be : uip_ds6_get_link_local()

  //TODO handle when there is no parent.

  allocation_request rqst = { 
      ALLOCATION_REQUEST,
      255,  // signal power
      0,
      our_ip,
      rpl_parent
  };

  UART_WRITE_STRING(UART_DEBUG, "parent, our_ip: \n");
  PRINT6ADDR(&rpl_parent);
  UART_WRITE_STRING(UART_DEBUG, "\n");
  PRINT6ADDR(&our_ip);
  UART_WRITE_STRING(UART_DEBUG, "\n");

  return rqst;

}

void
rpl_callback_additional_tsch_parent_switch(rpl_parent_t *old, rpl_parent_t *new)
{

  UART_WRITE_STRING(UART_DEBUG, "APP: RPL Parent changed, requesting a change of geolocation cell\n");

#if DEBUG_STARTUP_TIME
  debug_packet dbp = {
    DEBUGGING,
    "parent changed."
  };
  send_to_central_authority(&dbp, sizeof(dbp));
#endif

#if IS_MOBILE

  allocation_request rqst = get_allocation_request();
  send_to_central_authority(&rqst, sizeof(rqst));

#endif

}

void
send_to_mobile(uip_ipaddr_t *mobile_ip, void *data_to_transmit, int length)
{
  uip_ipaddr_t *nearest_anchor_ip = mobile_ip;
  struct uip_udp_conn *mobile_conn = udp_new(nearest_anchor_ip, UIP_HTONS(UDP_SERVER_PORT), NULL); 
  
  if (mobile_conn == NULL) {
    UART_WRITE_STRING(UART_DEBUG, "No UDP connection available, exiting the process!\n");
    return;
  }

  udp_bind(mobile_conn, UIP_HTONS(UDP_CLIENT_PORT)); 

  // For debugging purposes
  UART_WRITE_STRING(UART_DEBUG,"Created a connection with the server ");
  PRINT6ADDR(&mobile_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n",
	UIP_HTONS(mobile_conn->lport), UIP_HTONS(mobile_conn->rport));


  UART_WRITE_STRING(UART_DEBUG,"Data Sent to the remote server\n");
  uip_udp_packet_sendto(mobile_conn, data_to_transmit, length,
                        nearest_anchor_ip, UIP_HTONS(UDP_SERVER_PORT));
}


void send_to_all_mobiles(void *data_to_transmit, int length)
{
  uip_ds6_route_t *route;

  /* Loop over routing entries */
  route = uip_ds6_route_head();
  if (route == NULL) {
      UART_WRITE_STRING(UART_DEBUG, "No routes are available, skipping sending to children.\n");
  }
  while(route != NULL) {
    const uip_ipaddr_t *address = &route->ipaddr;
    const uip_ipaddr_t *nexthop = uip_ds6_route_nexthop(route);

    send_to_mobile(nexthop, data_to_transmit, length);

    route = uip_ds6_route_next(route);
  }
}

void
send_to_central_authority(void *data_to_transmit, int length)
{


#if IS_ANCHOR

  // An anchor has direct UART connection to the central authority.
  if (*((uint8_t *) data_to_transmit) == ALLOCATION_ACK) {
      allocation_ack data = *((allocation_ack *) data_to_transmit);
      PRINTF("sending ALLOCATION_ACK (mobile, anchor):\n");
      PRINT6ADDR(&data.mobile_addr);
      PRINT6ADDR(&data.anchor_addr);
      PRINTF("\n");
  }

  if (*((uint8_t *) data_to_transmit) == ALLOCATION_REQUEST) {
      allocation_request data = *((allocation_request *) data_to_transmit);
      PRINTF("sending ALLOCATION_REQUEST (mobile, anchor):\n");
      PRINT6ADDR(&data.mobile_addr);
      PRINT6ADDR(&data.anchor_addr);
      PRINTF("\n");
  }

  if (*((uint8_t *) data_to_transmit) != CLEAR_SLOTFRAME) {
    // Anchors should ignore CLEAR_SLOTFRAME frames (and not forward them to the central authority).
    uart_send_bytes(data_to_transmit, length);
  }

  if (*((uint8_t *) data_to_transmit) == ALLOCATION_ACK) {
      allocation_ack data = *((allocation_ack *) data_to_transmit);
      PRINTF("after ALLOCATION_ACK (mobile, anchor):\n");
      PRINT6ADDR(&data.mobile_addr);
      PRINT6ADDR(&data.anchor_addr);
      PRINTF("\n");
  }

  if (*((uint8_t *) data_to_transmit) == ALLOCATION_REQUEST) {
      allocation_request data = *((allocation_request *) data_to_transmit);
      PRINTF("after ALLOCATION_REQUEST (mobile, anchor):\n");
      PRINT6ADDR(&data.mobile_addr);
      PRINT6ADDR(&data.anchor_addr);
      PRINTF("\n");
  }
  


#else /* IS_ANCHOR */

  // A mobile needs to send packets wirelessly to an (ideally the nearest) anchor,
  // which will forward it to the central autority.

  // Initiate a connection.
  // Remark: Ideally, an anchor should maintain constantly a connection with its nearest anchor.

  uip_ipaddr_t nearest_anchor_ip = query_best_anchor();
  struct uip_udp_conn *new_conn = udp_new(&nearest_anchor_ip, UIP_HTONS(UDP_SERVER_PORT), NULL);
  
  if (new_conn == NULL) {
    UART_WRITE_STRING(UART_DEBUG, "No UDP connection available, exiting the process! for \n");
    PRINT6ADDR(&nearest_anchor_ip);
    PRINTF("\n");
    rpl_print_neighbor_list();
    return;
  }

  
  anchor_conn = *new_conn; 

  udp_bind(&anchor_conn, UIP_HTONS(UDP_CLIENT_PORT)); 

  // For debugging purposes
  UART_WRITE_STRING(UART_DEBUG,"Created a connection with the server ");
  PRINT6ADDR(&anchor_conn.ripaddr);
  PRINTF(" local/remote port %u/%u\n",
	UIP_HTONS(anchor_conn.lport), UIP_HTONS(anchor_conn.rport));


  UART_WRITE_STRING(UART_DEBUG,"Data Sent to the remote server\n");
  uip_udp_packet_sendto(&anchor_conn, data_to_transmit, length,
                        &nearest_anchor_ip, UIP_HTONS(UDP_SERVER_PORT));

#endif

}

void
uart_send_bytes(void *data_to_transmit, int length)
{
  byte_stuffing_send_bytes(data_to_transmit, length);
  /*
  uint8_t stuffed_bytes[2 * MAX_SERIAL_LEN + 2];
  int length_to_write = byte_stuffing_encode(data_to_transmit, length, stuffed_bytes);

  uint8_t *current_ptr = stuffed_bytes;
  uint8_t *end = current_ptr + length_to_write;
  while (current_ptr < end) {
    uart_write_byte(UART_OUTPUT, *current_ptr);

    current_ptr += 1;
  }
  */
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
      state = STATE_WAIT_SFD;
      // TODO move the buffer here as well ?
          
          /*
          printf("receive_buffer of length %d:\n", receive_ptr - receive_buffer);
          for (int j=0; j<MAX_SERIAL_LEN; j++) {
            printf("%02x", receive_buffer[j]);
          }
          printf("\n");
          */

      act_on_message(receive_buffer, receive_ptr - receive_buffer);
      // Reset buffer for future use
      receive_ptr = receive_buffer;
      memset(receive_buffer, 0xAB , MAX_SERIAL_LEN);
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
  
  return 1;
}

void
act_on_message(uint8_t *msg, int length)
{

  switch (*msg) {

    case CLEAR_SLOTFRAME: ;

      UART_WRITE_STRING(UART_DEBUG, "clear\n");

      tsch_slotframe = tsch_schedule_create_initial();

      uip_ipaddr_t our_ip = uip_ds6_get_global(ADDR_PREFERRED)->ipaddr;

      clear_ack clearack = {
        CLEAR_ACK,
        our_ip
      };

#if IS_ANCHOR
      
      // Anchors forward the clear_ack frames by flooding to all RPL children.
      send_to_all_mobiles(msg, length);    // Forward the received CLEAR_SLOTFRAME frame.

#endif

      send_to_central_authority(&clearack, sizeof(clearack));
      //SEND_TO_CENTRAL_AUTHORITY(clear_ack);

      //uart_write_byte(UART_DEBUG, '0' + IS_MOBILE);

      /*
       * After a Clear slotframe, try to join the network via an ACK if the node is a mobile
       */

#if IS_MOBILE

      allocation_request rqst = get_allocation_request();
      send_to_central_authority(&rqst, sizeof(rqst));
      //SEND_TO_CENTRAL_AUTHORITY(rqst);

#endif /* IS_MOBILE */

      break;

    case ALLOCATION_SLOT: ;

      UART_WRITE_STRING(UART_DEBUG,  "add\n");

      allocation_slot packet = *(allocation_slot *)(msg);

      PRINT6ADDR(&packet.mobile_addr);
      UART_WRITE_STRING(UART_DEBUG, "\n");
      PRINT6ADDR(&packet.anchor_addr);
      UART_WRITE_STRING(UART_DEBUG, "\n");


#if IS_ANCHOR

      //uip_ipaddr_t our_ip_ = uip_ds6_get_global(ADDR_PREFERRED)->ipaddr;

      //if (uip_ip6addr_cmp_ed(&our_ip_, &packet.anchor_addr)) {
        // Only add the link to our schedule if the ALLOCATION_SLOT frame was intended for us.
        linkaddr_t mobile_addr = get_linkaddr_from_ipaddr(&packet.mobile_addr);
        UART_WRITE_STRING(UART_DEBUG, "Address of mobile: ");
        PRINTLLADDR(&mobile_addr);
        UART_WRITE_STRING(UART_DEBUG, "\n");
        tsch_schedule_add_link(tsch_slotframe, LINK_OPTION_TX, LINK_TYPE_PROP, &mobile_addr, packet.timeslot, packet.channel);
      //}

      // The frame also has to be forwarded to the anchor
      send_to_mobile(&packet.mobile_addr, msg, length);    // Forward the received ALLOCATION_SLOT frame.

#else /* IS_ANCHOR */
    
      linkaddr_t anchor_addr = get_linkaddr_from_ipaddr(&packet.anchor_addr);
      tsch_schedule_add_link(tsch_slotframe, LINK_OPTION_RX, LINK_TYPE_PROP, &anchor_addr, packet.timeslot, packet.channel);

#endif /* IS_ANCHOR */

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

      // Successfully deleted slot, send ack
      deallocation_ack d_ack = {
        DEALLOCATION_ACK,
        0,    // padding
        pkt.timeslot,
        pkt.channel,
        pkt.mobile_addr,
        pkt.anchor_addr
      };

      send_to_central_authority(&d_ack, sizeof(d_ack));

      break;


    default: ;

      UART_WRITE_STRING(UART_DEBUG, "frame ID does not match any known frame types.\n");

      break;
  }

}

void
handle_propagation(struct tsch_neighbor *data)
{
  // Instead of printing the prop time, send it to the CA
  
  // printf("New prop time %ld %u %lu %u\n", 
  //   data->last_prop_time.prop_time, 
  //   data->last_prop_time.asn.ms1b, /* most significant 1 byte */
  //   data->last_prop_time.asn.ls4b, /* least significant 4 bytes */
  //   data->last_prop_time.tsch_channel); 

  // UART_WRITE_STRING(UART_DEBUG, "handle_propagation called.\n");
  

  // Only anchors will initiate two-way ranging (see schedule)
  uip_ipaddr_t our_ip = uip_ds6_get_global(ADDR_PREFERRED)->ipaddr;

  uip_ipaddr_t mobile_ip = get_ipaddr_from_linkaddr(&data->last_prop_time.neighbor_addr);

  propagation_time prop_time = {
    PROPAGATION_TIME,
    0,  // padding1
    0,  // padding2
    data->last_prop_time.tsch_channel,
    mobile_ip,   // Change to mobile IP
    our_ip,
    data->last_prop_time.prop_time,
    data->last_prop_time.asn
  };

  send_to_central_authority(&prop_time, sizeof(propagation_time));

}