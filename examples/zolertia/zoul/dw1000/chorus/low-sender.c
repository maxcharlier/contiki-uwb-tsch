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

#define INITIATOR_ID 0x01

// uint8_t cir[DW_LEN_ACC_MEM]; 

#include "net/netstack.h"
#include "dev/serial-line.h"
#include "dw1000-arch.h"
#include "dw1000-const.h"
#include "dw1000.h"
#include "dw1000-util.h"
#include "dw1000-driver.h"
#include "net/rime/chameleon-bitopt.h"
#include "dev/uart.h"
#define DBG_CONF_UART               0
#define write_byte(b) uart_write_byte(DBG_CONF_UART, b)

#define PRINT_BYTE 1
#define RIME_CHANNEL 129
#undef PRINTF
#if !PRINT_BYTE
  #define PRINTF(...) printf(__VA_ARGS__)
#else /* !PRINT_BYTE */
  #define PRINTF(...) do {} while(0)
#endif /* PRINT_BYTE */

void dw_read_CIR(uint8_t * read_buf);
/*---------------------------------------------------------------------------*/
PROCESS(initiator_process, "Initiator Process");
AUTOSTART_PROCESSES(&initiator_process);
/*---------------------------------------------------------------------------*/
static uint8_t cir[DW_LEN_ACC_MEM]; 
/*---------------------------------------------------------------------------*/
static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{

  // printf("unicast message received from %d.%d\n",
  //  from->u8[0], from->u8[1]);
  if(from->u8[1] !=  INITIATOR_ID){
    write_byte((uint8_t) '-');
    write_byte((uint8_t) 'R');
    write_byte((uint8_t) ':');
    write_byte((uint8_t) from->u8[0]);
    write_byte((uint8_t) from->u8[1]);
    write_byte((uint8_t) ':');
    uint16_t fp_index = dw_get_fp_index();
    write_byte((uint8_t) (fp_index >> 6) & 0XFF);
    write_byte((uint8_t) (fp_index >> (8+6)) & 0xFF );
    write_byte((uint8_t) (fp_index & 0x3F));
    // printf("fp_index %d %d\n", dw_is_lde_done(), fp_index>>6);
    write_byte((uint8_t) ':');


    /* Clear the memory (to check if data is write in the table later) */
    for( uint16_t i =0; i < DW_LEN_ACC_MEM; i++){
      cir[i] = 0;
    }  

    /* read the ACC memory without using DMA */
    dw_read_CIR(cir);

    NETSTACK_RADIO.off();

    /* send to serial the contain of the ACC memory */
    for( uint16_t i =0; i < DW_LEN_ACC_MEM; i++){
      write_byte((uint8_t) cir[i]);
      watchdog_periodic(); /* avoid watchdog timer to be reach */
    }  

    write_byte((uint8_t) '\n');
    NETSTACK_RADIO.on();
  }
  else{
    printf("broadcast message received from %d.%d: '%s'\n",
         from->u8[0], from->u8[1], (char *)packetbuf_dataptr());
  }

  watchdog_periodic(); /* avoid watchdog timer to be reach */
}
void send_message(struct broadcast_conn *conn){
  uint64_t sys_time_init = 0ULL, sys_time_end = 0ULL;

  dw_read_reg(DW_REG_SYS_TIME, DW_LEN_SYS_TIME, (uint8_t *)&sys_time_init);
  //prepare the response
  packetbuf_copyfrom("Hello", 5);

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

  NETSTACK_RADIO.prepare(frame, packet_len);

  // packetbuf_holds_broadcast();
  
  dw_read_reg(DW_REG_SYS_TIME, DW_LEN_SYS_TIME, (uint8_t *)&sys_time_end);
  printf("send_message duration %lu %llu %llu\n", RADIO_TO_US(sys_time_end-sys_time_init), sys_time_end, sys_time_init);
  print_frame(packet_len, frame);

  uint8_t value = NETSTACK_RADIO.transmit(packet_len);
  printf("message received\n");

  if(value ==  RADIO_TX_OK)
    printf("Transmit OK\n");
  else
    printf("Transmit Error %d\n", value);
  
  watchdog_periodic(); /* avoid watchdog timer to be reach */

}
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

static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(initiator_process, ev, data)
{
  static struct etimer et;

  PROCESS_EXITHANDLER(broadcast_close(&broadcast);)

  PROCESS_BEGIN();

  ctimer_set(&timer_transceiver_reset, 15 * CLOCK_SECOND, transceiver_soft_reset, NULL);

  broadcast_open(&broadcast, RIME_CHANNEL, &broadcast_call);

  while(1) {
    linkaddr_t addr;
    /* Delay 2-4 seconds */
    etimer_set(&et, CLOCK_SECOND * 4 + random_rand() % (CLOCK_SECOND * 4));

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));


    addr.u8[0] = 0;
    addr.u8[1] = INITIATOR_ID;

    if(linkaddr_cmp(&addr, &linkaddr_node_addr)) {
      send_message(&broadcast);
      //   packetbuf_copyfrom("Hello", 5);

      // broadcast_send(&broadcast);
      printf("broadcast message sent\n");
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
