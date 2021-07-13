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


/*---------------------------------------------------------------------------*/
PROCESS(example_unicast_process, "Example unicast");
AUTOSTART_PROCESSES(&example_unicast_process);
/*---------------------------------------------------------------------------*/
static void
recv_uc(struct unicast_conn *c, const linkaddr_t *from)
{
uint8_t cir[DW_LEN_ACC_MEM]; 
  NETSTACK_RADIO.off();
  // printf("unicast message received from %d.%d\n",
  //  from->u8[0], from->u8[1]);

  write_byte((uint8_t) '-');
  write_byte((uint8_t) 'R');
  write_byte((uint8_t) ':');
  write_byte((uint8_t) from->u8[0]);
  write_byte((uint8_t) from->u8[1]);
  // printf("%d",from->u8[0]);
  // printf("%d",from->u8[1]);
  write_byte((uint8_t) ':');

  dw_read_reg(DW_REG_ACC_MEM, DW_LEN_ACC_MEM, cir);

  /* send to serial the contain of the ACC memory */
  for( uint16_t i =1; i < DW_LEN_ACC_MEM; i++){
    write_byte((uint8_t) cir[i]);
    // write_byte((uint8_t) i);
    watchdog_periodic(); /* avoid watchdog timer to be reach */
  }

  write_byte((uint8_t) '\n');
  NETSTACK_RADIO.on();
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

/*---------------------------------------------------------------------------*/
static const struct unicast_callbacks unicast_callbacks = {recv_uc, sent_uc};
static struct unicast_conn uc;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_unicast_process, ev, data)
{
  PROCESS_EXITHANDLER(unicast_close(&uc);)
    
  PROCESS_BEGIN();

  unicast_open(&uc, 146, &unicast_callbacks);

  while(1) {
    static struct etimer et;
    linkaddr_t addr;
    
    etimer_set(&et, CLOCK_SECOND);
    
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    packetbuf_copyfrom("Hello", 5);
    addr.u8[0] = 0;
    addr.u8[1] = DESTINATION_ID;

    /* disable ACk request to avoid modify the CIR on the receiver side */
    packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 0);
    packetbuf_set_attr(PACKETBUF_ATTR_MAX_MAC_TRANSMISSIONS, 1);
    if(!linkaddr_cmp(&addr, &linkaddr_node_addr)) {
      unicast_send(&uc, &addr);
    }

  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
