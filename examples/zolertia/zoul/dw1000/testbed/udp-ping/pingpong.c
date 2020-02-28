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

#include "net/ipv6/uip-ds6-route.h"
#include "net/ipv6/uip-ds6-nbr.h"
#include "net/mac/tsch/tsch.h"
#include "net/netstack.h"


/* containt def of tsch_schedule_get_slotframe_duration */
#include "net/mac/tsch/tsch-schedule.h" 

#include "examples/zolertia/zoul/dw1000/testbed/shedule-testbed.h"

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


// <<<<<<< End timer config

static struct uip_udp_conn *client_conn;
static uip_ipaddr_t dest_ipaddr;
static struct ctimer periodic_timer1, periodic_timer2;

/*---------------------------------------------------------------------------*/
PROCESS(udp_ping_process, "Ping Pong");
AUTOSTART_PROCESSES(&udp_ping_process);
/*---------------------------------------------------------------------------*/
static int seq_id=0;

static void
tcpip_handler(void)
{
  char *appdata;

  if(uip_newdata()) {
    appdata = (char *)uip_appdata;
    appdata[uip_datalen()] = 0;
    printf("R:%02x%02x:%d:%s\n", UIP_IP_BUF->srcipaddr.u8[14], UIP_IP_BUF->srcipaddr.u8[15], UIP_TTL - UIP_IP_BUF->ttl + 1,appdata);
  }
}
/*---------------------------------------------------------------------------*/
static void
send_packet(void *ptr)
{
	char buf[MAX_PAYLOAD_LEN];
  uip_ds6_nbr_t *nbr;
  int num;

  seq_id++;
  num = 0;
  printf("send_packet()\n");
  for(nbr = nbr_table_head(ds6_neighbors);
      nbr != NULL;
      nbr = nbr_table_next(ds6_neighbors, nbr)) {

    sprintf(buf, "%d:%d", seq_id, num);

    printf("S:%02x%02x:%d:%d\n", nbr->ipaddr.u8[14],nbr->ipaddr.u8[15], seq_id, num);

    uip_udp_packet_sendto(client_conn, buf, strlen(buf), &(nbr->ipaddr), UIP_HTONS(UDP_PORT));
    num++;
  }

  ctimer_restart(&periodic_timer1);
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
      PRINTF("\n");
    }
  }
}
static void
set_global_address(void);
/*---------------------------------------------------------------------------*/
static void
print_info(void *ptr){
  print_local_addresses();
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
PROCESS_THREAD(udp_ping_process, ev, data)
{
  PROCESS_BEGIN();
  PROCESS_PAUSE();

  set_global_address();

  print_local_addresses();


  uip_ip6addr(&dest_ipaddr, 0xff02, 0, 0, 0, 0, 0, 0, 0x001a);

  // printf("Node ID::%02x%02x\n", uip_lladdr.addr[6],uip_lladdr.addr[7]);
  // PRINTF("UDP client process started nbr:%d routes:%d\n",
  //        NBR_TABLE_CONF_MAX_NEIGHBORS, UIP_CONF_MAX_ROUTES);

  tsch_schedule_fullmesh_data();
  // tsch_schedule_fullmesh_data_2nodes();
  tsch_schedule_print();


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
  ctimer_set(&periodic_timer1, 4*(CLOCK_SECOND * tsch_schedule_get_slotframe_duration())/RTIMER_SECOND, send_packet, &periodic_timer1);


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
