/*
 * Copyright (c) 2016, Inria.
 * All rights reserved.
 *
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

/**
 * \file
 *         An example of Rime/TSCH
 * \author
 *         Simon Duquennoy <simon.duquennoy@inria.fr>
 *
 */

#include <stdio.h>
#include "contiki-conf.h"
#include "net/netstack.h"
#include "net/rime/rime.h"
#include "net/mac/tsch/tsch.h"

#include "dev/uart.h"

#include "dev/serial-line.h"


#include "net/mac/tsch/tsch-queue.h" /* containt def of tsch_neighbor */
/* containt def of tsch_schedule_get_slotframe_duration */
#include "net/mac/tsch/tsch-schedule.h" 

#define HELLO_PORT    146
#define RANGING_PORT  147
#define MAX_PAYLOAD_LEN   70

#define write_byte(b) uart_write_byte(DBG_CONF_UART, b)

#define PRINT_BYTE 1
#if !PRINT_BYTE
  #define PRINTF(...) printf(__VA_ARGS__)
#else /* !PRINT_BYTE */
  #define PRINTF(...) do {} while(0)
#endif /* PRINT_BYTE */



const linkaddr_t coordinator_addr =    { { 0X00, 0XD0 } };
const linkaddr_t sink_addr =    { { 0X00, 0XD0 } };

static struct tsch_prop_time anchors_prop[5];
static rtimer_clock_t last_transmition = 0;

/*---------------------------------------------------------------------------*/
PROCESS(unicast_test_process, "Rime Node");
PROCESS(TSCH_PROP_PROCESS, "TSCH localization process");
AUTOSTART_PROCESSES(&unicast_test_process);

/*---------------------------------------------------------------------------*/
/* Hello connection */
static void
recv_uc(struct unicast_conn *c, const linkaddr_t *from)
{
  PRINTF("App: unicast message received from %u.%u\n",
   from->u8[0], from->u8[1]);
}
static void
sent_uc(struct unicast_conn *ptr, int status, int num_tx)
{
  PRINTF("App: unicast message sent, status %u, num_tx %u\n",
   status, num_tx);
}

/*---------------------------------------------------------------------------*/
/* ranging connection */
static void
sent_ranging(struct unicast_conn *ptr, int status, int num_tx)
{
 PRINTF("Ranging: unicast message sent, status %u, num_tx %u\n",
   status, num_tx);
}
/*---------------------------------------------------------------------------*/
static void
recv_ranging(struct unicast_conn *c, const linkaddr_t *from)
{

  #if PRINT_BYTE
    /* print R: _NODEADDR_PACKETBUF_LEN_
      for each prop time:
      _ANCHORID_TPROP_
    */
    printf("-R:");
    write_byte(from->u8[1]);
    write_byte(from->u8[0]);
    unsigned char *str;
    if(packetbuf_datalen() > 0) {
      str = (unsigned char *)packetbuf_dataptr();
      write_byte(packetbuf_datalen());
      for(int i = 0; i < packetbuf_datalen(); i++){
        write_byte((uint8_t) str[i]);    
      }
    }

    write_byte((uint8_t) '\n');

  #else /* PRINT_BYTE */  
    printf("R: 0X%02X%02X",from->u8[0], from->u8[1]);
    unsigned char *str;
    if(packetbuf_datalen() > 0) {
      uint8_t current_index= 0;
      int32_t prop_time;

      str = (unsigned char *)packetbuf_dataptr();
      str[packetbuf_datalen()] = '\0';

      for(int i = 0; i < packetbuf_datalen() / 11; i++){
        printf(" %c", (uint8_t) str[current_index]);
        current_index++;
        memcpy(&prop_time, &str[current_index], 4);
        current_index += 4;
        printf(" %ld",  prop_time);
        memcpy(&prop_time, &str[current_index], 6);
        current_index += 6;
        printf(" %ld",  prop_time);
        /* channel */
        printf(" %u",  str[current_index]);
        current_index += 1;
      }
      write_byte(packetbuf_datalen());
      for(int i = 0; i < packetbuf_datalen(); i++){
        write_byte((uint8_t) str[i]);    
      }
    }
    printf("\n");
  #endif /* PRINT_BYTE */
}
static void
initialize_anchro_prop(void){
  struct tsch_prop_time n_prop_time;
  n_prop_time.prop_time = 0;
      
  for (int i =0; i < 4; i++){
    anchors_prop[i]=n_prop_time;
  }
}
/*---------------------------------------------------------------------------*/

static const struct unicast_callbacks hello_callbacks = { recv_uc, sent_uc };
static struct unicast_conn hello_connection;

static const struct unicast_callbacks ranging_callbacks = { recv_ranging, sent_ranging };
static struct unicast_conn ranging_connection;
/*---------------------------------------------------------------------------*/
static void
send_packet(void)
{
  char buf[MAX_PAYLOAD_LEN];
  uint8_t current_index= 0;
  for (int i=0; i <5; i++){
    // if(anchors_prop[i].asn > 0){
      buf[current_index] = 0XD0+i;
      current_index++;
      memcpy(&buf[current_index], &(anchors_prop[i].prop_time), 4);
      current_index += 4;
      memcpy(&buf[current_index], &(anchors_prop[i].asn), 5);
      current_index += 5;
      memcpy(&buf[current_index], &(anchors_prop[i].tsch_channel), 1);
      current_index += 1;
    // }

  }
  if(current_index > 0 && !linkaddr_cmp(&sink_addr, &linkaddr_node_addr)){
    packetbuf_copyfrom(buf, current_index);
    packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 1);

    printf("App: sending unicast message to 0X%02X.%02X\n", sink_addr.u8[0], sink_addr.u8[1]);
    unicast_send(&ranging_connection, &sink_addr);


    last_transmition = RTIMER_NOW();
  }
}
/*---------------------------------------------------------------------------*/
/* Protothread for slot operation, called by update_neighbor_prop_time() 
 * function. "data" is a struct tsch_neighbor pointer.*/
PROCESS_THREAD(TSCH_PROP_PROCESS, ev, data)
{
  PROCESS_BEGIN();

  printf("tsch_loc_operation start\n");

  while(1) {
    PROCESS_YIELD();
    /* receive a new propagation time measurement */
    if(ev == PROCESS_EVENT_MSG){
      printf("Node 0X%02X prop time %ld %u %lu %u\n", 
        ((struct tsch_neighbor *) data)->addr.u8[sizeof(linkaddr_t)-1],
        ((struct tsch_neighbor *) data)->last_prop_time.prop_time, 
        ((struct tsch_neighbor *) data)->last_prop_time.asn.ms1b, /* most significant 1 byte */
        ((struct tsch_neighbor *) data)->last_prop_time.asn.ls4b, /* least significant 4 bytes */
        ((struct tsch_neighbor *) data)->last_prop_time.tsch_channel);

      // printf("Node 0X%02X anchors_prop index %d\n", 
      //   ((struct tsch_neighbor *) data)->addr.u8[sizeof(linkaddr_t)-1],
      //   ((struct tsch_neighbor *) data)->addr.u8[sizeof(linkaddr_t)-1]-(0XA1));


      struct tsch_prop_time n_prop_time;
      n_prop_time.prop_time = ((struct tsch_neighbor *) data)->last_prop_time.prop_time;
      n_prop_time.asn = ((struct tsch_neighbor *) data)->last_prop_time.asn;
      n_prop_time.tsch_channel = ((struct tsch_neighbor *) data)->last_prop_time.tsch_channel;
      /* replace older measurement */
      uint8_t i = MAX(0, ((struct tsch_neighbor *) data)->addr.u8[sizeof(linkaddr_t)-1]-(0XD0));
      anchors_prop[i] = n_prop_time;

      /* check if we need to send an updated */
      // if( RTIMER_NOW() > (last_transmition + (5 * RTIMER_SECOND))){
      if( RTIMER_NOW() > (last_transmition + tsch_schedule_get_slotframe_duration())){
        send_packet();
      }
    }
    if(ev == serial_line_event_message && data != NULL) {
      char *str;
      str = data;
      if(str[0] == 'r') {
        PRINTF("tsch_schedule_print node id 0X%02X\n", linkaddr_node_addr.u8[1]);
        tsch_schedule_print();
        
      }
    }
  }

  PROCESS_END();
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(unicast_test_process, ev, data)
{
  PROCESS_BEGIN();

  tsch_set_coordinator(linkaddr_cmp(&coordinator_addr, &linkaddr_node_addr));

  initialize_anchro_prop();

  NETSTACK_MAC.on();

  unicast_open(&hello_connection, HELLO_PORT, &hello_callbacks);
  unicast_open(&ranging_connection, RANGING_PORT, &ranging_callbacks);

  while(1) {
    static struct etimer et;

    etimer_set(&et, CLOCK_SECOND*10);

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    packetbuf_copyfrom("Hello", 5);
    // packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 1);


    if(!linkaddr_cmp(&sink_addr, &linkaddr_node_addr)) {
      printf("App: sending unicast message to %u.%u\n", sink_addr.u8[0], sink_addr.u8[1]);
      unicast_send(&hello_connection, &sink_addr);
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
