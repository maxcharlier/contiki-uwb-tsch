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
 *         Send a broacast message in each slotframe.
 * \author
 *         Maximilien Charlier
 *
 */

#include <stdio.h>
#include "contiki.h"
#include "contiki-conf.h"
#include "net/netstack.h"
#include "net/rime/rime.h"
#include "net/mac/tsch/tsch.h"

/* containt def of tsch_schedule_get_slotframe_duration */
#include "net/mac/tsch/tsch-schedule.h" 

const linkaddr_t coordinator_addr =    { { 0X00, 0XD0 } };
const linkaddr_t destination_addr =    { { 0x00, 0XD0 } };




static struct rtimer timer_send;

/*---------------------------------------------------------------------------*/
PROCESS(global_pdr_process, "Global PDR");
AUTOSTART_PROCESSES(&global_pdr_process);

void tsch_create_schedule(void);
/*---------------------------------------------------------------------------*/
static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
  printf("broadcast message received from 0X%02X%02X: '%s'\n",
         from->u8[0], from->u8[1], (char *)packetbuf_dataptr());
}
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;
/*---------------------------------------------------------------------------*/
void
trigger_broadcast(struct rtimer *timer, void *ptr)
{
  packetbuf_copyfrom("Hello", 6);

  broadcast_send(&broadcast);
  printf("broadcast message sent\n");
  // printf("Rtimer now %lu %lu\n", RTIMER_NOW(), tsch_schedule_get_slotframe_duration());

  /* Re-arm rtimer */
  rtimer_set(&timer_send, RTIMER_NOW() + 2*tsch_schedule_get_slotframe_duration(), 
    0, trigger_broadcast, NULL);
}



/*---------------------------------------------------------------------------*/
static void
recv_uc(struct unicast_conn *c, const linkaddr_t *from)
{
  printf("App: unicast message received from %u.%u\n",
   from->u8[0], from->u8[1]);
}
/*---------------------------------------------------------------------------*/
static void
sent_uc(struct unicast_conn *ptr, int status, int num_tx)
{
  printf("App: unicast message sent, status %u, num_tx %u\n",
   status, num_tx);
}

static const struct unicast_callbacks unicast_callbacks = { recv_uc, sent_uc };
static struct unicast_conn uc;


void
trigger_unicast(struct rtimer *timer, void *ptr)
{

  printf("pouet %lu\n", RTIMER_NOW());
  printf("seconds %lu\n", (tsch_schedule_get_slotframe_duration()*2)/RTIMER_SECOND);
    packetbuf_copyfrom("Hello", 5);
    // packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 1);


    if(!linkaddr_cmp(&destination_addr, &linkaddr_node_addr)) {
      printf("App: sending unicast message to %u.%u\n", destination_addr.u8[0], destination_addr.u8[1]);
      unicast_send(&uc, &destination_addr);
    }

  // printf("Rtimer now %lu %lu\n", RTIMER_NOW(), tsch_schedule_get_slotframe_duration());

  /* Re-arm rtimer */
  rtimer_set(&timer_send, RTIMER_NOW() + 2*tsch_schedule_get_slotframe_duration(), 
    0, trigger_unicast, NULL);
  printf("ok %lu\n", RTIMER_NOW());
}



/*---------------------------------------------------------------------------*/
PROCESS_THREAD(global_pdr_process, ev, data)
{  
    static struct etimer et;

  PROCESS_EXITHANDLER(broadcast_close(&broadcast);)

  PROCESS_BEGIN();

  tsch_create_schedule();

  tsch_set_coordinator(linkaddr_cmp(&coordinator_addr, &linkaddr_node_addr));

  NETSTACK_MAC.on();

  broadcast_open(&broadcast, 129, &broadcast_call);
    unicast_open(&uc, 146, &unicast_callbacks);


    /* Delay 50 seconds */
    etimer_set(&et, CLOCK_SECOND * 150);
  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    packetbuf_copyfrom("Hello", 6);
    broadcast_send(&broadcast);
    printf("broadcast message sent\n");

    /* Delay 1 slotframe */
    etimer_set(&et, (CLOCK_SECOND * tsch_schedule_get_slotframe_duration())/RTIMER_SECOND);

  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

/* Ensure that the NODEID macro is correctly passed by the Makefile.
   That's not the case for each target platform. */
#ifndef NODEID
#  error "Node ID not defined. Perhaps you need to tweak target Makefile."
#endif /* NODEID */

void tsch_create_schedule(void)
{
  struct tsch_slotframe *sf_custom;

  /* First, empty current schedule */
  tsch_schedule_remove_all_slotframes();

  /* Build schedule.
   * We pick a slotframe length of TSCH_SCHEDULE_DEFAULT_LENGTH */
  sf_custom = tsch_schedule_add_slotframe(0, TSCH_SCHEDULE_DEFAULT_LENGTH);

  linkaddr_t node_1_address = { { 0xFF, 0xFF } };
  linkaddr_t node_2_address = { { 0xFF, 0xFF } };
  linkaddr_t node_3_address = { { 0xFF, 0xFF } };
  linkaddr_t node_4_address = { { 0xFF, 0xFF } };
  linkaddr_t node_5_address = { { 0xFF, 0xFF } };
  linkaddr_t node_6_address = { { 0xFF, 0xFF } };
  linkaddr_t node_7_address = { { 0xFF, 0xFF } };
  linkaddr_t node_8_address = { { 0xFF, 0xFF } };
  linkaddr_t node_9_address = { { 0xFF, 0xFF } };
  linkaddr_t node_10_address = { { 0xFF, 0xFF } };
  linkaddr_t node_11_address = { { 0xFF, 0xFF } };
  linkaddr_t node_12_address = { { 0xFF, 0xFF } };


  linkaddr_copy(&node_1_address, &linkaddr_node_addr);
  node_1_address.u8[(sizeof(linkaddr_t)-1)] = 0xD0;
  linkaddr_copy(&node_2_address, &linkaddr_node_addr);
  node_2_address.u8[(sizeof(linkaddr_t)-1)] = 0x01;
  linkaddr_copy(&node_3_address, &linkaddr_node_addr);
  node_3_address.u8[(sizeof(linkaddr_t)-1)] = 0x02;
  linkaddr_copy(&node_4_address, &linkaddr_node_addr);
  node_4_address.u8[(sizeof(linkaddr_t)-1)] = 0x03;
  linkaddr_copy(&node_5_address, &linkaddr_node_addr);
  node_5_address.u8[(sizeof(linkaddr_t)-1)] = 0x04;
  linkaddr_copy(&node_6_address, &linkaddr_node_addr);
  node_6_address.u8[(sizeof(linkaddr_t)-1)] = 0x05;
  linkaddr_copy(&node_7_address, &linkaddr_node_addr);
  node_7_address.u8[(sizeof(linkaddr_t)-1)] = 0x06;
  linkaddr_copy(&node_8_address, &linkaddr_node_addr);
  node_8_address.u8[(sizeof(linkaddr_t)-1)] = 0x07;
  linkaddr_copy(&node_9_address, &linkaddr_node_addr);
  node_9_address.u8[(sizeof(linkaddr_t)-1)] = 0x08;
  linkaddr_copy(&node_10_address, &linkaddr_node_addr);
  node_10_address.u8[(sizeof(linkaddr_t)-1)] = 0x09;
  linkaddr_copy(&node_11_address, &linkaddr_node_addr);
  node_11_address.u8[(sizeof(linkaddr_t)-1)] = 0x0A;
  linkaddr_copy(&node_12_address, &linkaddr_node_addr);
  node_12_address.u8[(sizeof(linkaddr_t)-1)] = 0x0B;
  

  const struct {
    struct tsch_slotframe *slotframe;
    uint8_t                link_options;
    enum link_type         link_type;
    const linkaddr_t      *address;
    uint16_t               timeslot;
    uint16_t               channel_offset;
  } timeslots[] = {
    { sf_custom, LINK_OPTION_TX | LINK_OPTION_RX | LINK_OPTION_SHARED | LINK_OPTION_TIME_KEEPING, LINK_TYPE_ADVERTISING, &tsch_broadcast_address, 60, 0 },
#if NODEID == 0xD0
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 4, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 8, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 12, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 16, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 20, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 24, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 28, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 32, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 36, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 40, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 44, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 48, 0 },
#elif NODEID == 0x01
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 4, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 8, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 12, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 16, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 20, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 24, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 28, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 32, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 36, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 40, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 44, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 48, 0 },
#elif NODEID == 0x02
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 4, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 8, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 12, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 16, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 20, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 24, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 28, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 32, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 36, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 40, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 44, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 48, 0 },
#elif NODEID == 0x03
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 4, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 8, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 12, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 16, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 20, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 24, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 28, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 32, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 36, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 40, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 44, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 48, 0 },
#elif NODEID == 0x04
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 4, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 8, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 12, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 16, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 20, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 24, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 28, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 32, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 36, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 40, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 44, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 48, 0 },
#elif NODEID == 0x05
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 4, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 8, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 12, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 16, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 20, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 24, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 28, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 32, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 36, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 40, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 44, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 48, 0 },
#elif NODEID == 0x06
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 4, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 8, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 12, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 16, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 20, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 24, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 28, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 32, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 36, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 40, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 44, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 48, 0 },
#elif NODEID == 0x07
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 4, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 8, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 12, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 16, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 20, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 24, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 28, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 32, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 36, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 40, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 44, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 48, 0 },
#elif NODEID == 0x08
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 4, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 8, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 12, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 16, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 20, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 24, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 28, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 32, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 36, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 40, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 44, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 48, 0 },
#elif NODEID == 0x09
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 4, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 8, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 12, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 16, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 20, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 24, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 28, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 32, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 36, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 40, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 44, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 48, 0 },
#elif NODEID == 0x0A
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 4, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 8, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 12, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 16, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 20, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 24, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 28, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 32, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 36, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 40, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 44, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 48, 0 },
#elif NODEID == 0x0B
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 4, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 8, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 12, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 16, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 20, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 24, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 28, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 32, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 36, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 40, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 44, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 48, 0 },
#else
#  error "Unhandled NODEID for static schedule."
#endif /* NODEID */
    { 0 }
  }, *l;

  for(l = timeslots ; l->slotframe ; l++)
    tsch_schedule_add_link(l->slotframe, l->link_options, l->link_type, l->address, l->timeslot, l->channel_offset);
}