/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
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
 *         Best-effort single-hop unicast example
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"
#include "net/rime/rime.h"
#include "random.h"

#include "dev/watchdog.h"
#include "dev/button-sensor.h"

#include "dev/leds.h"

#include <stdio.h>

#include "sys/timer.h"

#define ANCHOR_ID 0xA1

// uint8_t cir[DW_LEN_ACC_MEM]; 

#include "net/netstack.h"
#include "dev/serial-line.h"
#include "dw1000-arch.h"
#include "dw1000-const.h"
#include "dw1000.h"
#include "dw1000-driver.h"
#include "dw1000-util.h"

#include "net/rime/chameleon-bitopt.h"

#include "dev/uart.h"
#define DBG_CONF_UART               0
#define write_byte(b) uart_write_byte(DBG_CONF_UART, b)

#define PRINT_BYTE 1

#undef PRINTF
#if !PRINT_BYTE
  #define PRINTF(...) printf(__VA_ARGS__)
#else /* !PRINT_BYTE */
  #define PRINTF(...) do {} while(0)
#endif /* PRINT_BYTE */

void dw_read_CIR(uint8_t * read_buf);

static struct ctimer timer_transceiver_reset;
/**
 * Perfomes a Soft Reset of the transceiver every 5 seconds 
 * to restore a working state.
 * */
void
transceiver_soft_reset(){
    ctimer_reset(&timer_transceiver_reset);
    NETSTACK_RADIO.off();
    NETSTACK_RADIO.init();
    NETSTACK_RADIO.on();
}


/*---------------------------------------------------------------------------*/
PROCESS(anchor_process, "Anchor Process");
AUTOSTART_PROCESSES(&anchor_process);
/*---------------------------------------------------------------------------*/
/**
 * Schedule a delayed reception after a fixed delai in us and an offset of 248 ns according to the anchor id 
 * */ 
// void 
// dw1000_schedule_tx_radio(uint64_t delay);
/*---------------------------------------------------------------------------*/

/**
 * Send an automatic response after receiving a message from the initator.
 * 
 * First, places the transceiver in ILDE to allow delayed TX.
 * Then, choice the response delay according to the anchor ID,
 * Prepare and send the delayed Response
 * */
void send_delayed_response(struct broadcast_conn *conn){

  NETSTACK_RADIO.off();
  //Anchors receive a message and send a response after a fixed delay.
  // 1500 us + 128 ns * offset => one unit is 8 ns
  // 9 lower bit are ignored by the transceiver, unit of the 10nd is 8 ns
  uint64_t delay_radio =  US_TO_RADIO(1500) + (((linkaddr_node_addr.u8[1] - ANCHOR_ID)*16) << DW_TIMESTAMP_CLOCK_OFFSET);

  // preparation duration is about 185 us
  //prepare the response
  packetbuf_copyfrom("Resp", 5);

  /* disable ACK request to avoid modify the CIR on the receiver side */
  packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 0);
  // packetbuf_set_attr(PACKETBUF_ATTR_MAX_MAC_TRANSMISSIONS, 1);

  packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &linkaddr_node_addr);
  //call made in chameleon.c
  chameleon_bitopt.output(&conn->c.channel); //chameleon-bitop.c
  packetbuf_set_attr(PACKETBUF_ATTR_CHANNEL, *(&conn->c.channel.channelno));
  //make data consecutive in the packet buffer
  packetbuf_compact();

  packetbuf_set_attr(PACKETBUF_ATTR_FRAME_TYPE, FRAME802154_DATAFRAME);



  // create the header 
  NETSTACK_FRAMER.create();
  uint8_t *frame = packetbuf_hdrptr();
  uint8_t packet_len =packetbuf_totlen();

  
  // compatible with 850 and 6800 kbps bitare
  dw1000_schedule_tx_chorus(delay_radio);
  // print_frame(packet_len, frame);

  NETSTACK_RADIO.prepare(frame, packet_len);

  
  uint8_t value = NETSTACK_RADIO.transmit(packet_len);

    NETSTACK_RADIO.on();
  printf("message received\n");
  printf("Delay %lld in ns : %ld\n", delay_radio, RADIO_TO_NS(delay_radio));

  if(value ==  RADIO_TX_OK)
    printf("Transmit OK\n");
  else
    printf("Transmit Error %d\n", value);
  
  watchdog_periodic(); /* avoid watchdog timer to be reach */

}
static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
  send_delayed_response(c);
}
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(anchor_process, ev, data)
{
  PROCESS_EXITHANDLER(broadcast_close(&broadcast);)

  PROCESS_BEGIN();
  ctimer_set(&timer_transceiver_reset, 20 * CLOCK_SECOND, transceiver_soft_reset, NULL);

  broadcast_open(&broadcast, 129, &broadcast_call);

  while(1) {
    PROCESS_WAIT_EVENT();
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
