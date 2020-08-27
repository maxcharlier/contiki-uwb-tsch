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
#include "examples/zolertia/zoul/dw1000/testbed/schedule-thomas/schedule-thomas.h"

#include "dev/uart.h"
#include "dev/serial-line.h"

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

#define write_byte(b) uart_write_byte(DBG_CONF_UART, b)

#define PRINT_BYTE 0

#undef PRINTF
#if !PRINT_BYTE
  #define PRINTF(...) printf(__VA_ARGS__)
#else /* !PRINT_BYTE */
  #define PRINTF(...) do {} while(0)
#endif /* PRINT_BYTE */

#define ROOT_ID  0X01

#define UDP_PORT 5678
#define MAX_PAYLOAD_LEN   30


#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#define BUF_LEN 6

// <<<<<<< End timer config

static struct uip_udp_conn *client_conn;

static const uip_ipaddr_t ip_addr_node1 = { { 0xfe, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x01 } };
static const uip_ipaddr_t ip_addr_node2 = { { 0xfe, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x02 } };
static const uip_ipaddr_t ip_addr_node3 = { { 0xfe, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x03 } };
static const uip_ipaddr_t ip_addr_node4 = { { 0xfe, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x04 } };
static const uip_ipaddr_t ip_addr_node5 = { { 0xfe, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x05 } };
static const uip_ipaddr_t ip_addr_node6 = { { 0xfe, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x06 } };
static const uip_ipaddr_t ip_addr_node7 = { { 0xfe, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x07 } };
static const uip_ipaddr_t ip_addr_node8 = { { 0xfe, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x08 } };
static const uip_ipaddr_t ip_addr_node9 = { { 0xfe, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x09 } };
static const uip_ipaddr_t ip_addr_node10 = { { 0xfe, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x0A } };
static const uip_ipaddr_t ip_addr_node11 = { { 0xfe, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x0B } };
static const uip_ipaddr_t ip_addr_node12 = { { 0xfe, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x0C } };
static const uip_ipaddr_t ip_addr_node13 = { { 0xfe, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x0D } };
static const uip_ipaddr_t ip_addr_node14 = { { 0xfe, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x0E } };
static const uip_ipaddr_t ip_addr_node15 = { { 0xfe, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x0F } };
static const uip_ipaddr_t ip_addr_node16 = { { 0xfe, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x10 } };


static const uip_ipaddr_t ip_addr_nodeb1 = { { 0xfe, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0X11 } };
static const uip_ipaddr_t ip_addr_nodeb2 = { { 0xfe, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0X12 } };

static unsigned char last_prop_buf[MAX_PAYLOAD_LEN]; /* this buffer will contain the last propagation time measured */
static int current_index = 0; // used to store to total about of bytes in last_prop_buf
/*---------------------------------------------------------------------------*/
PROCESS(test_ranging, "Localization based on TASA");
PROCESS(TSCH_PROP_PROCESS, "TSCH localization process");
AUTOSTART_PROCESSES(&test_ranging);
/*---------------------------------------------------------------------------*/


/**
 * This function will be used by the sink of the network when after receiving a message containing propoagation time between a mobile node and an anchor. 
 */
static void
tcpip_handler(void)
{
  /* TODO see udp-ping/unicast-full-mesh.c for example */
  char *appdata;
  if(uip_newdata()) {
    // if new data is available
    appdata = (char *) uip_appdata;
    appdata[uip_datalen()] = 0;

    int32_t propagation_time = 0;
    memcpy(&propagation_time, &appdata[2], sizeof(int32_t));

    int64_t asn = 0;
    memcpy(&asn, &appdata[6], 5);


    // TODO for now, always print
    
    printf("R: 0X%02X%02X  -> 0X%02X%02X %ld %llu %u\n",
            UIP_IP_BUF->srcipaddr.u8[14], // src addr 
            UIP_IP_BUF->srcipaddr.u8[15], // src addr
            appdata[0],         // dst addr
            appdata[1],         // dst addr
            propagation_time,
            asn,
            appdata[11]);       // channel

  }
}

/*---------------------------------------------------------------------------*/
/**
* This function will be used to transmit a message to the sink, the message will contain propagation time information (also the source and the destination of the propagation time).
*/
static void
send_packet()
{

  /* first we check if we have neighbor (if it's the case we have joined TSCH) */
  if(nbr_table_head(ds6_neighbors) != NULL) {
    // Send data with the distination to the root
    
    // always send data to the root
    uip_udp_packet_sendto(client_conn, last_prop_buf, current_index, &ip_addr_node1, UIP_HTONS(UDP_PORT));
  }
  
  
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
/*---------------------------------------------------------------------------*/
static void
print_info(void *ptr){
  // print_local_addresses();
  tsch_schedule_print();
}
/*---------------------------------------------------------------------------*/
/**
* Used to define the IP addr, the IP will be made 
*/
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
/* replace data in last_prop_buf with the last propagation time measured */
static void
create_prop_buffer(struct tsch_neighbor * data)
{
  current_index = 0;
  /* The ranging is made between this node (check local addr) and the destination node. 
  The destination node have a linkaddr_t on 8 bytes, we get the two last one that represent the node ID */
  last_prop_buf[current_index] = data->addr.u8[6]; // address destination voisin
  current_index++;
  last_prop_buf[current_index] = data->addr.u8[7];
  current_index++;
  memcpy(&last_prop_buf[current_index], &(data->last_prop_time.prop_time), 4); // temps de propagation
  current_index += 4;
  memcpy(&last_prop_buf[current_index], &(data->last_prop_time.asn), 5); //asn
  current_index += 5;
  memcpy(&last_prop_buf[current_index], &(data->last_prop_time.tsch_channel), 1); //canal
  current_index += 1;
}

/*---------------------------------------------------------------------------*/
static void
print_buffer()
{

  #if PRINT_BYTE
    /* print R: _NODEADDR_PACKETBUF_LEN_
      for each prop time:
      _ANCHOR_ID T_PROP_ T_MESUREAMENT CHANNEL
    */
    printf("-R:");
    write_byte(last_prop_buf[1]);
    write_byte(last_prop_buf[0]);
    write_byte(current_index-1);
    for(int i = 2; i < current_index; i++){
      write_byte((uint8_t) last_prop_buf[i]);    
    }

    write_byte((uint8_t) '\n');

  #else /* PRINT_BYTE */  
    printf("R: 0X%02X%02X", last_prop_buf[0], last_prop_buf[1]);
      int64_t value = 0;
      int i = 2;

      /* prop time */
      memcpy(&value, &last_prop_buf[2], 4);
      i += 4;
      printf(" %lld",  value);
      value = 0;

      /* asn */
      memcpy(&value, &last_prop_buf[i], 5);
      i += 5;
      printf(" %llu",  value);

      /* channel */
      printf(" %u",  last_prop_buf[i]);

    printf("\n");
  #endif /* PRINT_BYTE */
}
/*---------------------------------------------------------------------------*/
/* Protothread for localisation slot operation, called by update_neighbor_prop_time() 
 * function. "data" is a struct tsch_neighbor pointer.

 After a localisation we will create a message and send it to the sink.*/
PROCESS_THREAD(TSCH_PROP_PROCESS, ev, data)
{
  PROCESS_BEGIN();

  printf("tsch_loc_operation start\n");

  while(1) {
    PROCESS_YIELD();
    /* receive a new propagation time measurement */
    if(ev == PROCESS_EVENT_MSG){
      
      // printf("Node 0X%02X prop time %ld %lu %u\n", 
      //   ((struct tsch_neighbor *) data)->addr.u8[sizeof(linkaddr_t)-1],
      //   ((struct tsch_neighbor *) data)->last_prop_time.prop_time, 
      //   ((struct tsch_neighbor *) data)->last_prop_time.last_mesureament,
      //   ((struct tsch_neighbor *) data)->last_prop_time.tsch_channel);

      create_prop_buffer((struct tsch_neighbor *) data);
      print_buffer();

      send_packet();
    }
    if(ev == serial_line_event_message && data != NULL) {
      char *str;
      str = data;
      if(str[0] == 'h') {
        printf("Available commands:\n");
        printf("'r' display the schedule\n");
        printf("'a' to display the ASN\n");
        printf("'l' to display if localization timeslots are enable\n");
        printf("'e' enable the localization timeslot\n");
        printf("'d' disable the localization timeslot\n");
        
      }
      if(str[0] == 'r') {
        printf("tsch_schedule_print node id 0X%02X\n", linkaddr_node_addr.u8[1]);
        tsch_schedule_print();
        
      }
      if(str[0] == 'a') {
        /* this value come from the file net/mac/tsch/tsch-asn.h */
        int64_t value;
        memcpy(&value, &tsch_current_asn, 5);
        printf("tsch current ASN %llu\n", value);
      }
      if(str[0] == 'l') {
        if(tsch_is_localization_enable()){
          printf("Localization timeslots are enable\n");
        }
        else {
          printf("Localization timeslots are disabled\n");
        }
      }

      if(str[0] == 'e') {
        printf("Enable localization timeslots\n");
        tsch_set_localization(1);
      }
      if(str[0] == 'd') {
        printf("Disable localization timeslots\n");
        tsch_set_localization(0);
      }
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(test_ranging, ev, data)
{
  PROCESS_BEGIN();
  PROCESS_PAUSE();

  set_global_address();

  print_local_addresses();

  // printf("Node ID::%02x%02x\n", uip_lladdr.addr[6],uip_lladdr.addr[7]);
  // PRINTF("UDP client process started nbr:%d routes:%d\n",
  //        NBR_TABLE_CONF_MAX_NEIGHBORS, UIP_CONF_MAX_ROUTES);

  tsch_schedule_create_testbed_localization_for_2_mobiles();

  tsch_schedule_print();


  NETSTACK_MAC.on();

  /* new connection with remote host */
  client_conn = udp_new(NULL, UIP_HTONS(UDP_PORT), NULL); 
  if(client_conn == NULL) {
    PRINTF("No UDP connection available, exiting the process!\n");
    PROCESS_EXIT();
  }
  udp_bind(client_conn, UIP_HTONS(UDP_PORT)); 

  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      tcpip_handler();
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
