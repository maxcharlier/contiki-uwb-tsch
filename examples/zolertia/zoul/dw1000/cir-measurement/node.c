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
#include <stdio.h>
#include "dev/watchdog.h"

#include "sys/timer.h"

#define DESTINATION_ID 0x01

// uint8_t cir[DW_LEN_ACC_MEM]; 

#include "net/netstack.h"
#include "dev/serial-line.h"
#include "dw1000-arch.h"
#include "dw1000-const.h"
#include "dw1000.h"

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
PROCESS(example_unicast_process, "Example unicast");
AUTOSTART_PROCESSES(&example_unicast_process);
/*---------------------------------------------------------------------------*/

static uint8_t cir[DW_LEN_ACC_MEM]; 
static void
recv_uc(struct unicast_conn *c, const linkaddr_t *from)
{

  // printf("unicast message received from %d.%d\n",
  //  from->u8[0], from->u8[1]);

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

  // dw_read_reg(DW_REG_ACC_MEM, DW_LEN_ACC_MEM, cir);

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


  watchdog_periodic(); /* avoid watchdog timer to be reach */
}
/*---------------------------------------------------------------------------*/
static void
sent_uc(struct unicast_conn *c, int status, int num_tx)
{

  NETSTACK_RADIO.off();
  const linkaddr_t *dest = packetbuf_addr(PACKETBUF_ADDR_RECEIVER);
  if(linkaddr_cmp(dest, &linkaddr_null)) {
    return;
  }
  printf("unicast message sent to %d.%d: status %d num_tx %d\n",
    dest->u8[0], dest->u8[1], status, num_tx);

  NETSTACK_RADIO.on();
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
/*---------------------------------------------------------------------------*/
static const struct unicast_callbacks unicast_callbacks = {recv_uc, sent_uc};
static struct unicast_conn uc;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_unicast_process, ev, data)
{
  PROCESS_EXITHANDLER(unicast_close(&uc);)
    
  PROCESS_BEGIN();

  unicast_open(&uc, 146, &unicast_callbacks);
  
  ctimer_set(&timer_transceiver_reset, 5 * CLOCK_SECOND, transceiver_soft_reset, NULL);

  /* Enable Accumulator memory used to store the CIR after the transmittion or the recpeiton of a message */
  NETSTACK_RADIO.set_value(RADIO_ACCUMULATOR_MEMORY, RADIO_POWER_MODE_ON);
  
  while(1) {

    static struct etimer et;
    linkaddr_t addr;
    
    etimer_set(&et, CLOCK_SECOND);
    
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    packetbuf_copyfrom("Hello", 5);
    addr.u8[0] = 0;
    addr.u8[1] = DESTINATION_ID;

    /* disable ACK request to avoid modify the CIR on the receiver side */
    packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 0);
    packetbuf_set_attr(PACKETBUF_ATTR_MAX_MAC_TRANSMISSIONS, 1);


    if(!linkaddr_cmp(&addr, &linkaddr_node_addr)) {
      unicast_send(&uc, &addr);
    }

  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
