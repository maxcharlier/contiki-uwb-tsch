#include "contiki.h"
#include "lib/random.h"
#include "sys/ctimer.h"
#include "net/ip/uip.h"
#include "net/ip/uipopt.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-udp-packet.h"
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

#include "net/rpl/rpl.h"


#include "examples/zolertia/zoul/dw1000/localisation-mobility/libs/message-formats.h"
#include "examples/zolertia/zoul/dw1000/localisation-mobility/libs/byte-stuffing.h"
#include "examples/zolertia/zoul/dw1000/localisation-mobility/libs/send-messages.h"
#include "examples/zolertia/zoul/dw1000/localisation-mobility/libs/schedule.h"

#include "dev/uart.h"
#include "dev/serial-line.h"

#include "cpu/cc2538/lpm.h"

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

#define write_byte(b) uart_write_byte(DBG_CONF_UART, b)

#define PRINT_BYTE 0


#undef PRINTF
#if !PRINT_BYTE
  #define PRINTF(...) printf(__VA_ARGS__)
#else
  #define PRINTF(...) do {} while(0)
#endif



#define UDP_CLIENT_PORT	8765
#define UDP_SERVER_PORT	5678

#define MAX_PAYLOAD_LEN   30



#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#define BUF_LEN 6


static struct uip_udp_conn *client_conn;
static struct uip_udp_conn *server_conn;




/*---------------------------------------------------------------------------*/




/*---------------------------------------------------------------------------*/
PROCESS(TSCH_PROP_PROCESS, "TSCH propagation time process");
PROCESS(udp_client_process, "UDP Client");
AUTOSTART_PROCESSES(&udp_client_process);
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* Protothread for localisation slot operation, called by update_neighbor_prop_time() 
 * function. "data" is a struct tsch_neighbor pointer.

 After a localisation we will create a message and send it to the sink.*/
PROCESS_THREAD(TSCH_PROP_PROCESS, ev, data)
{
  PROCESS_BEGIN();

  // PROCESS_PAUSE();

  while(1) {
    PROCESS_YIELD();
    // PROCESS_WAIT_EVENT();
    if(ev == PROCESS_EVENT_MSG){
      handle_propagation(data);
    }
  }

  PROCESS_END();
}

void debug_precompiler() {
  uart_write_byte(UART_DEBUG, '0' + IS_MOBILE); 
  printf("\n");
}

static void
iterate_children(void)
{
  uip_ds6_route_t *route;
  printf("Start listing childen:\n");

  /* Loop over routing entries */
  route = uip_ds6_route_head();
  while(route != NULL) {
    const uip_ipaddr_t *address = &route->ipaddr;
    const uip_ipaddr_t *nexthop = uip_ds6_route_nexthop(route);

    PRINT6ADDR(address);
    PRINT6ADDR(nexthop);
    printf("\n");
    
    if(uip_ipaddr_cmp(&address, &nexthop)) {
       /* direct child: do somehting */
       
    }

    route = uip_ds6_route_next(route);
  }
  printf("End listing childen.\n");
}

int
debug_uart_receive_byte(unsigned char c) {
  switch (c) {
    case 's':   tsch_schedule_print();                                                                      break;
    case 'S':   tsch_slotframe = tsch_schedule_create_initial();                                            break;
    case 'p':   PRINT6ADDR(query_best_anchor());                                                            break;
    case 'n':   rpl_print_neighbor_list();                                                                  break;
    case 'e':   tsch_set_prop_measurement(1);                                                               break;
    case 'd':   tsch_set_prop_measurement(0);                                                               break;
    case 'D':   uart_write_byte(UART_DEBUG, '0' + NODEID);                                                  break;
    case 'i':   uip_ipaddr_t our_ip = uip_ds6_get_global(ADDR_PREFERRED)->ipaddr; PRINT6ADDR(&our_ip);      break;
    case 'x':   uart_write_byte(UART_DEBUG, '0' + sizeof(message_type));                                    break;
    case 'y':   uip_ipaddr_t *parent = query_best_anchor(); PRINT6ADDR(parent);                             break;
    case 'c':   iterate_children();                                                                         break;
    case 'b':   debug_precompiler();
    default :   uart_write_byte(UART_DEBUG, c);                                                             break;
  }
  return 1;
}


static void
tcpip_handler(void)
{
  if(uip_newdata()) {

#if IS_ANCHOR
    
    printf("Packet received.\n");
    send_to_central_authority(uip_appdata, uip_datalen()); 

#else /* IS_ANCHOR */

    // This is a mobile node, messages coming from a UDP packet are just forwarded
    // through the nearest anchor.
    act_on_message(uip_appdata, uip_datalen());

#endif /* IS_ANCHOR */

  }  
}

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

  lpm_set_max_pm(LPM_PM0);    // Keep the UART from going to sleep

  set_global_address();

  // Define the schedule
  tsch_slotframe = tsch_schedule_create_initial();


  uart_set_input(0, debug_uart_receive_byte);
  uart_set_input(1, uart_receive_byte);


  NETSTACK_MAC.on();



  /* Enable receiving UDP packets */
  server_conn = udp_new(NULL, UIP_HTONS(UDP_CLIENT_PORT), NULL);
  if(server_conn == NULL) {
    PRINTF("No UDP connection available, exiting the process!\n");
    PROCESS_EXIT();
  }
  udp_bind(server_conn, UIP_HTONS(UDP_SERVER_PORT));

  PRINTF("Created a server connection with remote address ");
  PRINT6ADDR(&server_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n", UIP_HTONS(server_conn->lport),
         UIP_HTONS(server_conn->rport));
  /* Finished enabling receiving UDP packets */

  // ctimer_set(&retry_timer, 15 * CLOCK_SECOND, send_allocation_probe_request, &retry_timer);

  while(1) {
    PROCESS_YIELD();
    
    if(ev == tcpip_event) {
      tcpip_handler();
    }

  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
