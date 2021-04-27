#include "contiki.h"
#include "lib/random.h"
#include "sys/ctimer.h"
#include "net/ip/uip.h"
#include "net/ip/uipopt.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-udp-packet.h"
#include "net/rpl/rpl.h"
#include <stdio.h>
#include <string.h>

#include "net/ipv6/sicslowpan.h" // get the last channel

#include "net/ipv6/uip-ds6-route.h"
#include "net/ipv6/uip-ds6-nbr.h"
#include "net/mac/tsch/tsch.h"
#include "net/netstack.h"

/* contain the tsch_current_asn */
#include "net/mac/tsch/tsch-private.h"
#include "net/mac/tsch/tsch-asn.h"


/* containt def of tsch_schedule_get_slotframe_duration */
#include "net/mac/tsch/tsch-schedule.h" 

#include "examples/zolertia/zoul/dw1000/localisation-mobility/libs/message-formats.h"
#include "examples/zolertia/zoul/dw1000/localisation-mobility/libs/byte-stuffing.h"

#include "dev/uart.h"
#include "dev/serial-line.h"

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

#define write_byte(b) uart_write_byte(DBG_CONF_UART, b)

#define PRINT_BYTE 1


#undef PRINTF
#if !PRINT_BYTE
  #define PRINTF(...) printf(__VA_ARGS__)
#else
  #define PRINTF(...) do {} while(0)
#endif


#define ROOT_ID  0X01

#undef RPL_LEAF_ONLY
#define RPL_LEAF_ONLY 1

#define UDP_PORT 5678
#define MAX_PAYLOAD_LEN   30
#define MAX_SERIAL_LEN    100


#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#define BUF_LEN 6

// <<<<<<< End timer config

static struct uip_udp_conn *client_conn;
static struct ctimer periodic_timer1, periodic_timer2;

static const uip_ipaddr_t ip_addr_node1 = { { 0xfe, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x01 } };
static const uip_ipaddr_t ip_addr_node2 = { { 0xfe, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x02 } };
static const uip_ipaddr_t ip_addr_node3 = { { 0xfe, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x03 } };

static const uip_ipaddr_t * local_neighborg_addr[] = { 
  &ip_addr_node1, 
  &ip_addr_node2, 
  &ip_addr_node3
};

static const int len_local_neighborg=3;
static const int number_of_transmission_per_timer=1;


/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP Client");
AUTOSTART_PROCESSES(&udp_client_process);
/*---------------------------------------------------------------------------*/
static int seq_id=0;
static int sending_index=0;

static void
tcpip_handler(void)
{
  char *appdata;

  if(uip_newdata()) {
    int64_t value = 0;
    appdata = (char *)uip_appdata;
    appdata[uip_datalen()] = 0;
    #if PRINT_BYTE
      /* print S: _NODEADDR_status_num_tx
      */
      write_byte((uint8_t) '-');
      write_byte((uint8_t) 'R');
      write_byte((uint8_t) ':');
      write_byte(UIP_IP_BUF->srcipaddr.u8[15]);
      write_byte(UIP_IP_BUF->srcipaddr.u8[14]);

      for(int i = 0; i < MIN(BUF_LEN,uip_datalen()) ; i++){
        write_byte((uint8_t) appdata[i]);    
      }

      memcpy(&value, &tsch_current_asn, 5);
      for(int i = 0; i < 5 ; i++){
        write_byte((uint8_t) ((uint8_t*)&value)[i]);    
      }
      write_byte((uint8_t) sicslowpan_get_last_channel());
      write_byte((uint8_t) '\n');
    #else /* PRINT_BYTE */  

      printf("R:%02x%02x:%d:", UIP_IP_BUF->srcipaddr.u8[14], UIP_IP_BUF->srcipaddr.u8[15], appdata[0]);
      /* asn */
      memcpy(&value, &appdata[1], 5);
      printf("%llu:",  value);

      value = 0;
      memcpy(&value, &tsch_current_asn, 5);
      printf("%llu",  value);
      printf("%d",  sicslowpan_get_last_channel());
      printf("\n");

      // printf("S: 0X%02X%02X stat %d tx %d \n", 
      //   dest->u8[0], dest->u8[1], status, num_tx);
    #endif /* PRINT_BYTE */
  }
}

static void
print_local_addresses(void);
/*---------------------------------------------------------------------------*/
static void
send_packet(void *ptr)
{
	char buf[MAX_PAYLOAD_LEN];
  // print_local_addresses();
  // printf("send_packet()\n");
  

  /* first we check if we have neighbor (if it's the case we have join TSCH) */
  if(nbr_table_head(ds6_neighbors) != NULL){
    for(int i = 0; i < number_of_transmission_per_timer; i++) {
      /* check to not send message to our addr */
      if((sending_index + i + 1) != NODEID){
        
        uip_ipaddr_t *destination_address = local_neighborg_addr[(sending_index + i)%len_local_neighborg];

        /* send the message */
        PRINTF("DATA send to %d 'Hello %d'\n", destination_address->u8[sizeof(destination_address->u8) - 1], seq_id);
        sprintf(buf, "Hello %d from the client", seq_id);

        uip_udp_packet_sendto(client_conn, buf, strlen(buf), destination_address , UIP_HTONS(UDP_PORT));
      }
      if((sending_index + i)%len_local_neighborg == 0){
        seq_id ++;
      }
    }

    sending_index += number_of_transmission_per_timer;
  }

  rpl_print_neighbor_etx_list();

  ctimer_set(&periodic_timer1, 1*(CLOCK_SECOND * tsch_schedule_get_slotframe_duration())/RTIMER_SECOND, send_packet, &periodic_timer1);
}

/*---------------------------------------------------------------------------*/
static void
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


/*---------------------------------------------------------------------------*/
static void
send_allocation_probe_request(void *ptr)
{
  PRINTF("APP: Leaf-only: %i\n", RPL_LEAF_ONLY);


          // Temporary :
          uip_ipaddr_t mobile_ip = uip_ds6_get_global(ADDR_PREFERRED)->ipaddr;
          allocation_request rqst = { 
            ALLOCATION_REQUEST,
            255,  // signal power
            mobile_ip,
            mobile_ip
          };
          send_to_central_authority(&rqst, sizeof(rqst));
          goto retry;
  
  // Check if a RPL parent is present
  rpl_parent_t *rpl_parent = nbr_table_head(rpl_parents);
  
  if (!rpl_parent) {
    // No parent to send a probe request to.
    // Wait for RPL to find a parent.
    PRINTF("APP: No parent, retrying in 1s\n");

    goto retry;
  }

  uip_ipaddr_t mobile_ip = uip_ds6_get_global(ADDR_PREFERRED)->ipaddr; // Could also be : uip_ds6_get_link_local()
  uip_ipaddr_t *rpl_parent_ip = rpl_get_parent_ipaddr(rpl_parent);

  allocation_request rqst = { 
    ALLOCATION_REQUEST,
    255,  // signal power
    mobile_ip,
    *rpl_parent_ip
  };

  PRINTF("APP: Sending data through serial.\n");
  send_to_central_authority(&rqst, sizeof(rqst));

retry:
  ctimer_set(&periodic_timer1, 1*(CLOCK_SECOND), send_allocation_probe_request, &periodic_timer1);    
}
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTF("Server IPv6 addresses: ");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(state == ADDR_TENTATIVE || state == ADDR_PREFERRED) {
      PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      printf("\n");
      for (int j = 0; j< sizeof(uip_ipaddr_t); j++){
        printf("0x%02x ", uip_ds6_if.addr_list[i].ipaddr.u8[j]);
      }
      printf("\n");
    }
  }
}
static void
set_global_address(void);
/*---------------------------------------------------------------------------*/
static void
print_info(void *ptr){
  // print_local_addresses();
  tsch_schedule_print();

  ctimer_restart(&periodic_timer2);
}
/*---------------------------------------------------------------------------*/
static void
set_global_address(void)
{
  uip_ipaddr_t ipaddr;

  #if NODEID == ROOT_ID
  struct uip_ds6_addr *root_if;

  uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_MANUAL);
  root_if = uip_ds6_addr_lookup(&ipaddr);
  if(root_if != NULL) {
    rpl_dag_t *dag;
    dag = rpl_set_root(RPL_DEFAULT_INSTANCE,(uip_ip6addr_t *)&ipaddr);
    uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
    rpl_set_prefix(dag, &ipaddr, 64);
    PRINTF("created a new RPL dag\n");
  } else {
    PRINTF("failed to create a new RPL DAG\n");
  }
  #else
  uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);
  #endif /* NODEID */

}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{
  PROCESS_BEGIN();
  PROCESS_PAUSE();

  set_global_address();

  print_local_addresses();

  // printf("Node ID::%02x%02x\n", uip_lladdr.addr[6],uip_lladdr.addr[7]);
  // PRINTF("UDP client process started nbr:%d routes:%d\n",
  //        NBR_TABLE_CONF_MAX_NEIGHBORS, UIP_CONF_MAX_ROUTES);


  NETSTACK_MAC.on();

  /* new connection with remote host */
  client_conn = udp_new(NULL, UIP_HTONS(UDP_PORT), NULL); 
  if(client_conn == NULL) {
    PRINTF("No UDP connection available, exiting the process!\n");
    PROCESS_EXIT();
  }
  udp_bind(client_conn, UIP_HTONS(UDP_PORT)); 

  /* interval is a slotframe duration. 
  We convert the tsch_schedule_get_slotframe_duration in Rtimer to Ctimer
  */
  ctimer_set(&periodic_timer1, 15 * CLOCK_SECOND, send_allocation_probe_request, &periodic_timer1);
  // ctimer_set(&periodic_timer1, 10 * CLOCK_SECOND, send_packet, &periodic_timer1);


  // ctimer_set(&periodic_timer2, (CLOCK_SECOND * 10), print_info, &periodic_timer2);

  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      tcpip_handler();
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
