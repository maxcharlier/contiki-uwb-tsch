/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

#include "contiki.h"
#include "lib/random.h"
#include "sys/ctimer.h"
#include "net/ip/uip.h"
#include "net/ip/uipopt.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-udp-packet.h"
#include "net/rpl/rpl.h"
#ifdef WITH_COMPOWER
#include "powertrace.h"
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dev/serial-line.h"
#include "net/ipv6/uip-ds6-route.h"


#include "net/mac/tsch/tsch-queue.h" /* containt def of tsch_neighbor */
/* containt def of tsch_schedule_get_slotframe_duration */
#include "net/mac/tsch/tsch-schedule.h" 


/* from core/net/ip/simple-udp.c */
#define UIP_IP_BUF   ((struct uip_udpip_hdr *)&uip_buf[UIP_LLH_LEN])


#define UDP_CLIENT_PORT 8765
#define UDP_SERVER_PORT 5678

#define UDP_SERVER_ADDR SINK_ID

#define DEBUG DEBUG_FULL
#include "net/ip/uip-debug.h"

#ifndef PERIOD
#define PERIOD 15
#endif

#define START_INTERVAL		(15 * CLOCK_SECOND)
#define SEND_INTERVAL		(PERIOD * CLOCK_SECOND)
#define SEND_TIME		(random_rand() % (SEND_INTERVAL))
#define MAX_PAYLOAD_LEN		30

static struct uip_udp_conn *client_conn;
static uip_ipaddr_t server_ipaddr;
static struct tsch_prop_time anchors_prop[4];
static rtimer_clock_t last_transmition = 0;


/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client process");
PROCESS(TSCH_PROP_PROCESS, "TSCH localization process");
AUTOSTART_PROCESSES(&udp_client_process);
/*---------------------------------------------------------------------------*/

static void
tcpip_handler(void)
{
  char *str;

  if(uip_newdata()) {
    str = uip_appdata;
    str[uip_datalen()] = '\0';

    printf("DATA recv from %d \n",
           UIP_IP_BUF->srcipaddr.u8[sizeof(UIP_IP_BUF->srcipaddr.u8) - 1]);
    uint8_t current_index= 0;
    int32_t prop_time;
    for (int i = 0; i < uip_datalen() / 5; i++){
      printf("Node id 0X%02X", (uint8_t) str[current_index]);
      current_index++;
      memcpy(&prop_time, &str[current_index], 4);
      current_index += 4;
      printf(" %ld\n", prop_time);
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
send_packet(void)
{
  char buf[MAX_PAYLOAD_LEN];
  uint8_t current_index= 0;
  for (int i=0; i <4; i++){
    if(anchors_prop[i].last_mesureament > last_transmition){
      buf[current_index] = 0XA1+i;
      current_index++;
      memcpy(&buf[current_index], &(anchors_prop[i].prop_time), 4);
      current_index += 4;
    }

  }
  if(current_index > 0 ){
    printf("Server address 0X%02X packet_len %d \n", server_ipaddr.u8[15], current_index);
    uip_udp_packet_sendto(client_conn, buf, current_index,
                          &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
    printf("send packet\n");
    last_transmition = RTIMER_NOW();
  }
}
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTF("Client IPv6 addresses: ");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      PRINTF("\n");
      /* hack to make address "final" */
      if (state == ADDR_TENTATIVE) {
	uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
set_global_address(void)
{
  uip_ipaddr_t ipaddr;
  uip_lladdr_t server_lladdr;

  struct uip_ds6_addr *root_if;

  uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);



  if(ipaddr.u8[15]== ROOT_ID){
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
  }
  else{
    printf("This node is not a root node\n");
  }

  // uip_ip6addr(&server_ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0x00ff, 0xfe00, 0XA5);


  /* we set manualy the Server addr based on the current mac addr of the node.
  Then we replace the last byte of the mac addr by the server node id */
  uip_ip6addr(&server_ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
  memcpy(&server_lladdr, &uip_lladdr, sizeof(uip_lladdr_t));
  ((uint8_t *) &server_lladdr)[sizeof(uip_lladdr_t)-1] = UDP_SERVER_ADDR;
  uip_ds6_set_addr_iid(&server_ipaddr, &server_lladdr);

  printf("Server addr: ");
  uip_debug_ipaddr_print(&server_ipaddr);
  printf("\n");
  
}

/*---------------------------------------------------------------------------*/
/* Protothread for slot operation, called by update_neighbor_prop_time() 
 * function. "data" is a struct tsch_neighbor pointer.*/
PROCESS_THREAD(TSCH_PROP_PROCESS, ev, data)
{
  PROCESS_BEGIN();

  // PROCESS_PAUSE();

  printf("tsch_loc_operation start\n");

  while(1) {
    PROCESS_WAIT_EVENT();
    /* receive a new propagation time measurement */
    if(ev == PROCESS_EVENT_MSG){
      // printf("Node 0X%02X prop time %ld %lu\n", 
      //   ((struct tsch_neighbor *) data)->addr.u8[7],
      //   ((struct tsch_neighbor *) data)->last_prop_time.prop_time, 
      //   ((struct tsch_neighbor *) data)->last_prop_time.last_mesureament);

      printf("Node 0X%02X anchors_prop index %d\n", 
        ((struct tsch_neighbor *) data)->addr.u8[7],
        ((struct tsch_neighbor *) data)->addr.u8[7]-(0XA1));


      struct tsch_prop_time n_prop_time;
      n_prop_time.prop_time = ((struct tsch_neighbor *) data)->last_prop_time.prop_time;
      n_prop_time.last_mesureament = ((struct tsch_neighbor *) data)->last_prop_time.last_mesureament;
      /* replace older measurement */
      anchors_prop[((struct tsch_neighbor *) data)->addr.u8[7]-(0XA1)] = n_prop_time;

      /* check if we need to send an updated */
      // if(((struct tsch_neighbor *) data)->last_prop_time.last_mesureament > 
      //   last_mesureament + tsch_schedule_get_slotframe_duration()){
      //   send_packet();
      // }
      /* check if we need to send an updated */
      if( RTIMER_NOW() > (last_transmition + (5 * RTIMER_SECOND))){
        send_packet();
      }
    }
    if(ev == serial_line_event_message && data != NULL) {
      char *str;
      str = data;
      if(str[0] == 'r') {
        printf("tsch_schedule_print TSCH_PROP_PROCESS \n");
        tsch_schedule_print();
        
      }
    }
  }

  PROCESS_END();
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{

  PROCESS_BEGIN();

  PROCESS_PAUSE();

  set_global_address();

  PRINTF("UDP client process started nbr:%d routes:%d\n",
         NBR_TABLE_CONF_MAX_NEIGHBORS, UIP_CONF_MAX_ROUTES);

  print_local_addresses();

  struct tsch_prop_time n_prop_time;
  n_prop_time.prop_time = 0;
  n_prop_time.last_mesureament = 0;
      
  for (int i =0; i < 4; i++){
    anchors_prop[i]=n_prop_time;
  }
  /* new connection with remote host */
  client_conn = udp_new(NULL, UIP_HTONS(UDP_SERVER_PORT), NULL); 
  if(client_conn == NULL) {
    PRINTF("No UDP connection available, exiting the process!\n");
    PROCESS_EXIT();
  }
  udp_bind(client_conn, UIP_HTONS(UDP_CLIENT_PORT)); 

  PRINTF("Created a connection with the server ");
  PRINT6ADDR(&client_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n",
  UIP_HTONS(client_conn->lport), UIP_HTONS(client_conn->rport));

#if WITH_COMPOWER
  powertrace_sniff(POWERTRACE_ON);
#endif

  NETSTACK_MAC.on();
  
  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      tcpip_handler();
    }

    if(ev == serial_line_event_message && data != NULL) {
      char *str;
      str = data;
      if(str[0] == 'r') {
        printf("tsch_schedule_print udp_client_process \n");
        tsch_schedule_print();
        
      }
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
