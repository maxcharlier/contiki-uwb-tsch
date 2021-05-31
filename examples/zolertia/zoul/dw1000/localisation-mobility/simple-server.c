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


#include "examples/zolertia/zoul/dw1000/localisation-mobility-reborn/libs/message-formats.h"
#include "examples/zolertia/zoul/dw1000/localisation-mobility-reborn/libs/send-messages.h"


#include "dev/uart.h"

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

#define write_byte(b) uart_write_byte(DBG_CONF_UART, b)

#define PRINT_BYTE 0

#undef IS_LOCATION_SERVER
#define IS_LOCATION_SERVER 1


#undef PRINTF
#if !PRINT_BYTE
  #define PRINTF(...) printf(__VA_ARGS__)
#else
  #define PRINTF(...) do {} while(0)
#endif


#define ROOT_ID  0X05

#define UDP_PORT 5678
#define MAX_PAYLOAD_LEN   30


#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#define BUF_LEN 6

// <<<<<<< End timer config

static struct uip_udp_conn *client_conn;


/*---------------------------------------------------------------------------*/
PROCESS(udp_server_process, "UDP Server");
AUTOSTART_PROCESSES(&udp_server_process);
/*---------------------------------------------------------------------------*/

int
debug_uart_receive_byte(unsigned char c) {
  uart_write_byte(0, c);
  uart_write_byte(1, c);
  return 1;
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
  //uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  //uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);
  #endif /* NODEID */

}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_server_process, ev, data)
{
  PROCESS_BEGIN();
  PROCESS_PAUSE();

  set_global_address();

  // print_local_addresses();

  // struct tsch_slotframe *tsch_slotframe = tsch_schedule_add_slotframe(0, 31);

  uart_set_input(0, debug_uart_receive_byte);
  uart_set_input(1, debug_uart_receive_byte);

  NETSTACK_MAC.on();


  ctimer_set(&retry_timer, 5 * CLOCK_SECOND, send_allocation_probe_request, &retry_timer);

  while(1) {
    PROCESS_YIELD();

    if(ev == tcpip_event) {
      // tcpip_handler();
    }

  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
