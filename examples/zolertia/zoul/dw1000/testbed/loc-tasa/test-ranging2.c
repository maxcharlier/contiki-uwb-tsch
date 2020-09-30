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


#define NODEID_0 0xA1
#define NODEID_1 0xA2
#define ROOT_ID  NODEID_0

#define UDP_PORT 5678
#define MAX_PAYLOAD_LEN   30


#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#define BUF_LEN 6

// <<<<<<< End timer config

static struct uip_udp_conn *client_conn;


/* fd00::fdff:ffff:ffff:1 is the gloabal addr and fe80::fdff:ffff:ffff:1 is a local addr */
static  uip_ipaddr_t ip_addr_root = { { 0xfd, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, ROOT_ID } };
// static  uip_ipaddr_t ip_addr_root = { { 0xfe, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, ROOT_ID } };
// static  uip_ipaddr_t ip_addr_root = { { 0x0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, ROOT_ID } };

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
    printf("Send to :");
    PRINT6ADDR(&ip_addr_root);
    printf("\n");
    
    // always send data to the root
    uip_udp_packet_sendto(client_conn, last_prop_buf, current_index, &ip_addr_root, UIP_HTONS(UDP_PORT));
  }
  else{
    printf("Try to send to :");
    PRINT6ADDR(&ip_addr_root);
    printf("\n");
    printf("Error send_packet no neighbor\n");

    uip_ds6_route_lookup(&ip_addr_root);
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
  PRINTF("Server IPv6 addresses: ");
  PRINT6ADDR(&ip_addr_root);
  printf("\n");
  for (int j = 0; j< sizeof(uip_ipaddr_t); j++){
    printf("0x%02x ", ip_addr_root.u8[j]);
  }
  printf("\n");
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

    PRINTF("Server IPv6 addresses: ");
    PRINT6ADDR(&ipaddr);
    printf("\n");
    for (int j = 0; j< sizeof(uip_ipaddr_t); j++){
      printf("0x%02x ", ipaddr.u8[j]);
    }
    printf("\n");
  } else {
    PRINTF("failed to create a new RPL DAG\n");
  }
  #else
  uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);
  #endif /* NODEID */



  /* Mode 1 - 64 bits inline */
  // uip_ip6addr(&ip_addr_root, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, ROOT_ID);
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
      value = 0;
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
        printf("'r' display the schedule and routing\n");
        printf("'a' to display the ASN\n");
        printf("'l' to display if localization timeslots are enable\n");
        printf("'e' enable the localization timeslot\n");
        printf("'d' disable the localization timeslot\n");
        printf("'i' print local addr\n");
        
      }
      if(str[0] == 'r') {
        printf("tsch_schedule_print node id 0X%02X\n", linkaddr_node_addr.u8[1]);
        tsch_schedule_print();

        uip_ds6_route_lookup(&ip_addr_root);
        
      }
      if(str[0] == 'a') {
        /* this value come from the file net/mac/tsch/tsch-asn.h */
        int64_t value = 0;
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
      if(str[0] == 'i') {
        printf("Print local addr\n");
        print_local_addresses();
      }
    }
  }

  PROCESS_END();
}
void tsch_schedule_fullmesh_data_2nodes2(void)
{
  struct tsch_slotframe *sf_custom;

  /* First, empty current schedule */
  tsch_schedule_remove_all_slotframes();

  /* Build schedule.
   * We pick a slotframe length of TSCH_SCHEDULE_DEFAULT_LENGTH */
  sf_custom = tsch_schedule_add_slotframe(0, 11);

  
      static linkaddr_t node_1_address;
      linkaddr_copy(&node_1_address, &linkaddr_node_addr);
      node_1_address.u8[7] = NODEID_0;
      static linkaddr_t node_2_address;
      linkaddr_copy(&node_2_address, &linkaddr_node_addr);
      node_2_address.u8[7] = NODEID_1;

  const struct {
    struct tsch_slotframe *slotframe;
    uint8_t                link_options;
    enum link_type         link_type;
    const linkaddr_t      *address;
    uint16_t               timeslot;
    uint16_t               channel_offset;
  } timeslots[] = {
    { sf_custom, LINK_OPTION_TX | LINK_OPTION_RX | LINK_OPTION_SHARED | LINK_OPTION_TIME_KEEPING, LINK_TYPE_ADVERTISING, &tsch_broadcast_address, 0, 0 },
#if NODEID == NODEID_0
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_PROP, &node_2_address, 2, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 4, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 6, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 8, 0 },
#elif NODEID == NODEID_1
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_PROP, &node_1_address, 2, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 4, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 6, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 8, 0 },
#endif /* NODEID */
    { 0 }
  }, *l;

  for(l = timeslots ; l->slotframe ; l++)
    tsch_schedule_add_link(l->slotframe, l->link_options, l->link_type, l->address, l->timeslot, l->channel_offset);
}
void tsch_schedule_fullmesh_data_2nodes3(void)
{
  struct tsch_slotframe *sf_custom;

  /* First, empty current schedule */
  tsch_schedule_remove_all_slotframes();

  /* Build schedule.
   * We pick a slotframe length of TSCH_SCHEDULE_DEFAULT_LENGTH */
  sf_custom = tsch_schedule_add_slotframe(0, 16);

  
      static linkaddr_t node_1_address;
      linkaddr_copy(&node_1_address, &linkaddr_node_addr);
      node_1_address.u8[7] = NODEID_0;
      static linkaddr_t node_2_address;
      linkaddr_copy(&node_2_address, &linkaddr_node_addr);
      node_2_address.u8[7] = NODEID_1;

  const struct {
    struct tsch_slotframe *slotframe;
    uint8_t                link_options;
    enum link_type         link_type;
    const linkaddr_t      *address;
    uint16_t               timeslot;
    uint16_t               channel_offset;
  } timeslots[] = {
    { sf_custom, LINK_OPTION_TX | LINK_OPTION_RX | LINK_OPTION_SHARED | LINK_OPTION_TIME_KEEPING, LINK_TYPE_ADVERTISING, &tsch_broadcast_address, 0, 0 },
#if NODEID == NODEID_0
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_PROP, &node_2_address, 3, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 6, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 9, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 12, 0 },
#elif NODEID == NODEID_1
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_PROP, &node_1_address, 3, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 6, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 9, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 12, 0 },
#endif /* NODEID */
    { 0 }
  }, *l;

  for(l = timeslots ; l->slotframe ; l++)
    tsch_schedule_add_link(l->slotframe, l->link_options, l->link_type, l->address, l->timeslot, l->channel_offset);
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

  tsch_schedule_fullmesh_data_2nodes2();

  tsch_schedule_print();


  NETSTACK_MAC.on();

  /* new connection with remote host */
  client_conn = udp_new(NULL, UIP_HTONS(UDP_PORT), NULL); 
  if(client_conn == NULL) {
    PRINTF("No UDP connection available, exiting the process!\n");
    PROCESS_EXIT();
  }
  udp_bind(client_conn, UIP_HTONS(UDP_PORT)); 

  PRINTF("Created a server connection with remote address ");
  PRINT6ADDR(&client_conn->ripaddr);

  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      tcpip_handler();
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
