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

#define RIME_CHANNEL 129

// uint8_t cir[DW_LEN_ACC_MEM]; 

#include "net/netstack.h"
#include "dev/serial-line.h"
#include "dw1000-arch.h"
#include "dw1000-const.h"
#include "dw1000.h"
#include "dw1000-driver.h"


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
/*---------------------------------------------------------------------------*/
PROCESS(initiator_process, "Initiator Process");
AUTOSTART_PROCESSES(&initiator_process);
/*---------------------------------------------------------------------------*/
static uint8_t cir[DW_LEN_ACC_MEM]; 
static rtimer_clock_t last_receive = 0;
/*---------------------------------------------------------------------------*/
/** 
 * Use the same configuration as Chorus :
 * In all experiments, we set the DW1000 to use
 * channel 7 with center frequency f c = 6489.6 GHz 
 * and bandwidth 1081.6 MHz. We use the shortest preamble 
 * length of 64 symbols with preamble code 17, 
 * the highest PRF = 64 MHz, and the highest
 * 6.8 Mbps data rate. 
 * Finally, we set the response delay δ TX = 280 μs
 * 
 * In out case due to contiki implementation, the response delay is about 5000 us */
void set_chorus_radio_configuration(void){
  /* we place the radio in IDLE before any change to the configuration */

  NETSTACK_RADIO.off();
  NETSTACK_RADIO.init();

  // be default we use the preamble code 17 
  dw1000_driver_config(DW_CHANNEL_7, DW_DATA_RATE_6800_KBPS, DW_PREAMBLE_LENGTH_64, DW_PRF_64_MHZ);

  NETSTACK_RADIO.on();
}

static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{

  // printf("unicast message received from %d.%d\n",
  //  from->u8[0], from->u8[1]);
  if(from->u8[1] !=  INITIATOR_ID){

    last_receive = RTIMER_NOW();

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


    uint16_t lde_threshold = dx_get_lde_threshold();
    write_byte((uint8_t) lde_threshold & 0xFF );
    write_byte((uint8_t) (lde_threshold >> 8) & 0XFF);

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
    // printf("broadcast message received from %d.%d: '%s'\n",
    //      from->u8[0], from->u8[1], (char *)packetbuf_dataptr());

    write_byte((uint8_t) '-');
    write_byte((uint8_t) 'R');
    write_byte((uint8_t) 'E');
    write_byte((uint8_t) 'Q');
    write_byte((uint8_t) ':');
    write_byte((uint8_t) from->u8[0]);
    write_byte((uint8_t) from->u8[1]);
    write_byte((uint8_t) '\n');
  }

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

    //reset the transceiver only if we don't receive a request for more the 2 seconde.
    if(RTIMER_NOW() > (last_receive + 2 * CLOCK_SECOND))
      set_chorus_radio_configuration();
}

static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(initiator_process, ev, data)
{
  static struct etimer et;

  static linkaddr_t addr;
  addr.u8[0] = 0;
  addr.u8[1] = INITIATOR_ID;

  PROCESS_EXITHANDLER(broadcast_close(&broadcast);)

  PROCESS_BEGIN();
  // reset the initiator every 15 seconds, mobile each 2 seconds if no messages received.
  if(linkaddr_cmp(&addr, &linkaddr_node_addr))
    ctimer_set(&timer_transceiver_reset, 15 * CLOCK_SECOND, transceiver_soft_reset, NULL);
  else
    ctimer_set(&timer_transceiver_reset, CLOCK_SECOND, transceiver_soft_reset, NULL);

  last_receive = RTIMER_NOW();

  broadcast_open(&broadcast, 129, &broadcast_call);
  set_chorus_radio_configuration();
  while(1) {
    /* Delay 2-4 seconds */
    // etimer_set(&et, CLOCK_SECOND * 4 + random_rand() % (CLOCK_SECOND * 4));
    etimer_set(&et, CLOCK_SECOND * 2 + random_rand() % (CLOCK_SECOND * 2));

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));


    packetbuf_copyfrom("Req.", 4);

    /* disable ACK request to avoid modify the CIR on the receiver side */
    packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 0);
    packetbuf_set_attr(PACKETBUF_ATTR_MAX_MAC_TRANSMISSIONS, 1);

    if(linkaddr_cmp(&addr, &linkaddr_node_addr)) {
      broadcast_send(&broadcast);
      printf("broadcast message sent\n");
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
