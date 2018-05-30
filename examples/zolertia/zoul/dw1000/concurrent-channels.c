/*
 * Copyright (c) 2017, UMONS University.
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
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * \file
 *         Master file to test the DWM1000 fonctionnality.
 *
 * \author
 *         Charlier Maximilien  <maximilien.charlier@umons.ac.be>
 */

#include "contiki.h"
#include "net/rime/rime.h"
#include "dev/serial-line.h"
#include <stdio.h>
#include <stdlib.h>
#include "net/netstack.h"
#include "dw1000-driver.h"
#include "dw1000-util.h"
#include "dw1000.h"

// #define DEBUG 1

/*---------------------------------------------------------------------------*/
PROCESS(initiator_process, "Initiator process");
PROCESS(receive_process, "Receive manager");

AUTOSTART_PROCESSES(&initiator_process);
/*---------------------------------------------------------------------------*/

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while(0)
#endif

#define RIME_CHANNEL 151
#define RIME_TYPE    "unicast"
static void recv_callback(struct unicast_conn *c, const linkaddr_t *from);
static struct unicast_conn uc;
static const struct unicast_callbacks uc_cb = { recv_callback };


/* store the last receivedd messag*/
static char payload[30];
static uint8_t payload_len;
static uint8_t seq_num = 0;

#define COMMAND_MESSAGE     0
#define CONCURRENT_MESSAGE  1
const uint16_t nodes_set[] = {0X1, 0X2, 0X3, 0X4};

static uint8_t message_received = 0; /* indicate if a message was received */

static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
  printf("broadcast message received from %d.%d: '%s'\n",
         from->u8[0], from->u8[1], (char *)packetbuf_dataptr());

  payload_len = packetbuf_datalen();
  packetbuf_copyto(payload);

  process_poll(&receive_process);

  message_received = 1;
}
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;



/* used when a message is received */
static void recv_callback(struct unicast_conn *c, const linkaddr_t *from)
{
  if(packetbuf_datalen() >= 2){ 
    payload_len = packetbuf_datalen();
    packetbuf_copyto(payload);

    PRINTF("Node receive message from 0x%02X%02X  seq num %u\n", 
                      from->u8[1], 
                      from->u8[0],
                      payload[1]);
  }
  else{
    PRINTF("Receive with unsupported size (%d) form %02X%02X\n", 
      packetbuf_datalen(), from->u8[1], from->u8[0]);
  }
}


PROCESS_THREAD(frame_master_process, ev, data)
{
  PROCESS_EXITHANDLER(unicast_close(&uc);)
  PROCESS_EXITHANDLER(broadcast_close(&broadcast);)

  PROCESS_BEGIN();

  printf("Node addr 0x%02X%02X\n", 
                      linkaddr_node_addr.u8[1], 
                      linkaddr_node_addr.u8[0]);
  printf("Set 1 : 0x%04X, 0x%04X\n", nodes_set[0], nodes_set[1]);
  printf("Set 2 : 0x%04X, 0x%04X\n", nodes_set[2], nodes_set[3]);
  printf("     TSCH CHANNEL Set1 and Set2\n");

  unicast_open(&uc, RIME_CHANNEL, &uc_cb);
  broadcast_open(&broadcast, 129, &broadcast_call);


  process_start(&receive_process, NULL);

  for(;;) {
    PROCESS_WAIT_EVENT();
    /* master part */
    if(ev == serial_line_event_message) {
      /* we convert the input string data to tow int using strlol see 
      https://www.tutorialspoint.com/c_standard_library/c_function_strtol.htm */
      char * str;
      uint8_t channel_set1 = strtol(data, &str, 10);
      uint8_t channel_set2 = strtol(str, &str, 10);

      PRINTF("Received line: %s\n", (char *)data);

      char request[13]; // 1 for mode, 2 for source, 2 for dest
      request[0] = COMMAND_MESSAGE;
      /* Copy the twin node channel*/
      for(int i = 0; i < 4; i++){

        memcpy(&request[1+(i*3)], &nodes_set[i], 2);
        if(i < 2)
          request[1+2+(i*3)] = channel_set1;
        else
          request[1+2+(i*3)] = channel_set2;
      }
      packetbuf_copyfrom(request, 13);
      broadcast_send(&broadcast);
    }
  }
  PROCESS_END();
}

/* Use to manage a response */
PROCESS_THREAD(receive_process, ev, data)
{

  PROCESS_BEGIN();

  while (1) {
    PROCESS_WAIT_EVENT();
    if(ev == PROCESS_EVENT_POLL){
      uint16_t node_addr = linkaddr_node_addr.u8[0] | linkaddr_node_addr.u8[1] << 8;

      for(int i = 0; i < 4; i++{
        if(payload[1+(i*3)] == node_addr[i] && 0xFF && payload[2+(i*3)] == (node_addr[i] << 8) && 0xFF){
          uint8_t channel = payload[3+(i*3)];
          NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, (radio_value_t) channel);

          if(i % 2 == 0){
            uint16_t dest_addr = node_addr[i+1];
            /* this is a sender */
            char request[2]; 
            /* store the mode */
            request[0] = CONCURRENT_MESSAGE;
            /* ranging type */
            request[1] = seq_num;

            packetbuf_copyfrom(request, 2);

            /* request an ACK */
            packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 0); /* no */

            unicast_send(&uc, &dest_addr);

            seq_num++;
          }
        }
      }
    }
  }
  
  PROCESS_END();
}