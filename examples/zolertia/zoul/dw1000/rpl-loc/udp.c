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
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/ip/uip.h"
#include "net/rpl/rpl.h"

#include "net/netstack.h"
#include "dev/button-sensor.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "dev/uart.h"
#include "dev/serial-line.h"
#include "net/mac/tsch/tsch-schedule.h"
#include "net/mac/tsch/tsch-queue.h" /* containt def of tsch_neighbor */

#define SERVER_ADDR   0X0D

#define DEBUG DEBUG_FULL
#include "net/ip/uip-debug.h"


#define MAX_PAYLOAD_LEN   70

#define write_byte(b) uart_write_byte(DBG_CONF_UART, b)

#define PRINT_BYTE 1
#if !PRINT_BYTE
  #undef PRINTF
  #define PRINTF(...) printf(__VA_ARGS__)
#else /* !PRINT_BYTE */
  #undef PRINTF
  #define PRINTF(...) do {} while(0)
#endif /* PRINT_BYTE */



const linkaddr_t sink_addr =    { { 0X00, 0XD0 } };
static unsigned char buf[MAX_PAYLOAD_LEN];
static uint8_t current_index = 0;


/*---------------------------------------------------------------------------*/
PROCESS(TSCH_PROP_PROCESS, "TSCH localization process");
PROCESS(udp_process, "UDP process");
AUTOSTART_PROCESSES(&udp_process);
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/* Localization part */
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
    write_byte(buf[1]);
    write_byte(buf[0]);
    write_byte(current_index-1);
    write_byte(linkaddr_node_addr.u8[1]);
    for(int i = 2; i < current_index; i++){
      write_byte((uint8_t) buf[i]);    
    }

    write_byte((uint8_t) '\n');

  #else /* PRINT_BYTE */  
    printf("R: 0X%02X%02X", buf[0], buf[1]);
      int32_t value;
      int i = 2;

      /* prop time */
      memcpy(&value, &buf[2], 4);
      i += 4;
      printf(" %ld",  value);

      /* asn */
      memcpy(&value, &buf[i], 5);
      i += 5;
      printf(" %llu",  value);

      /* channel */
      printf(" %u",  buf[i]);

    printf("\n");
  #endif /* PRINT_BYTE */
}
/*---------------------------------------------------------------------------*/
static void
create_buffer(struct tsch_neighbor * data)
{
  current_index = 0;
  buf[current_index] = data->addr.u8[0];
  current_index++;
  buf[current_index] = data->addr.u8[1];
  current_index++;
  memcpy(&buf[current_index], &(data->last_prop_time.prop_time), 4);
  current_index += 4;
  memcpy(&buf[current_index], &(data->last_prop_time.asn), 5);
  current_index += 5;
  memcpy(&buf[current_index], &(data->last_prop_time.tsch_channel), 1);
  current_index += 1;
}


/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/* RPL - UDP part */
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  printf("Client IPv6 addresses: ");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      printf("\n");
      /* hack to make address "final" */
      if (state == ADDR_TENTATIVE) {
	uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
      }
    }
  }
}/*---------------------------------------------------------------------------*/
/* Protothread for slot operation, called by update_neighbor_prop_time() 
 * function. "data" is a struct tsch_neighbor pointer.
 Need to declare TSCH_LOC_THREAD in the project-conf file*/
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
      //   ((struct tsch_neighbor *) data)->last_prop_time.last_measurement,
      //   ((struct tsch_neighbor *) data)->last_prop_time.tsch_channel);

      create_buffer((struct tsch_neighbor *) data);
      print_buffer();
    }
    if(ev == serial_line_event_message && data != NULL) {
      char *str;
      str = data;
      if(str[0] == 'd') {
        printf("tsch_schedule_print node id 0X%02X\n", linkaddr_node_addr.u8[1]);
        tsch_schedule_print();
        
      }
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_process, ev, data)
{
  uip_ipaddr_t ipaddr;
  struct uip_ds6_addr *root_if;

  PROCESS_BEGIN();

  PROCESS_PAUSE();

  /* Ensure that the NODEID macro is correctly passed by the Makefile.
   That's not the case for each target platform. */
#ifndef NODEID
#  error "Node ID not defined. Perhaps you need to tweak target Makefile."
#endif /* NODEID */

/* The choice of server address determines its 6LoWPAN header compression.
 * Obviously the choice made here must also be selected in udp-client.c.
 *
 * For correct Wireshark decoding using a sniffer, add the /64 prefix to the
 * 6LowPAN protocol preferences,
 * e.g. set Context 0 to fd00::. At present Wireshark copies Context/128 and
 * then overwrites it.
 * (Setting Context 0 to fd00::1111:2222:3333:4444 will report a 16 bit
 * compressed address of fd00::1111:22ff:fe33:xxxx)
 * Note Wireshark's IPCMV6 checksum verification depends on the correct
 * uncompressed addresses.
 */
 /* Mode 3 - derived from link local (MAC) address */
  uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

  if(ipaddr.u8[15]== SERVER_ADDR){
    root_if = uip_ds6_addr_lookup(&ipaddr);
    if(root_if != NULL) {
      rpl_dag_t *dag;
      dag = rpl_set_root(RPL_DEFAULT_INSTANCE,(uip_ip6addr_t *)&ipaddr);
      uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
      rpl_set_prefix(dag, &ipaddr, 64);
      printf("created a new RPL dag\n");
    } else {
      printf("failed to create a new RPL DAG\n");
    }
  }
  else{
    printf("This node is not a root node\n");
  }


  printf("UDP process started nbr:%d routes:%d\n",
         NBR_TABLE_CONF_MAX_NEIGHBORS, UIP_CONF_MAX_ROUTES);

  print_local_addresses();


  NETSTACK_MAC.on();
  
  while(1) {
    PROCESS_YIELD();
    if(ev == serial_line_event_message && data != NULL) {
      char *str;
      str = data;
      if(str[0] == 'r') {
        printf("tsch_schedule_print udp_server_process \n");
        tsch_schedule_print();
      }
    }
  }

  PROCESS_END();
}
