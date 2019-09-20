/*
 * Copyright (c) 2018, UMONS University.
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
 *         File to get the some values for TSCH with the DW1000 transceiver.
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
#include "dw1000.h"
#include "dw1000-driver.h"


// #define DEBUG 1

/*---------------------------------------------------------------------------*/
PROCESS(tsch_delay_process, "TSCH Delay");

AUTOSTART_PROCESSES(&tsch_delay_process);
/*---------------------------------------------------------------------------*/

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while(0)
#endif

                                                      \
uint64_t start, end;  
#define DURATION(input)                                                       \
  start = dw_read_reg_64(DW_REG_SYS_TIME, DW_LEN_SYS_TIME);                   \
  input;                                                                      \
  end = dw_read_reg_64(DW_REG_SYS_TIME, DW_LEN_SYS_TIME);                     \
  printf("DURATION RTIMER %lu\n", RADIO_TO_RTIMER(end-start));                \
  printf("DURATION US %lu\n", RTIMERTICKS_TO_US(RADIO_TO_RTIMER(end-start))); \
  printf("DURATION US %lu\n", RADIO_TO_US(end-start)); \
  printf("DURATION DW1000 %llu\n", end-start);                                \

#define RIME_CHANNEL 151
#define RIME_TYPE    "unicast"
static void recv_callback(struct unicast_conn *c, const linkaddr_t *from);
static struct unicast_conn uc;
static const struct unicast_callbacks uc_cb = { recv_callback };


/* used when a message is received */
static void recv_callback(struct unicast_conn *c, const linkaddr_t *from)
{
  if(packetbuf_datalen() >= 1){ 
    PRINTF("Node receive message from 0x%02X%02X\n", 
                      from->u8[1], 
                      from->u8[0]);

  }
  else{
    PRINTF("Receive with unsupported size (%d) form %02X%02X\n", 
      packetbuf_datalen(), from->u8[1], from->u8[0]);
  }
}


PROCESS_THREAD(tsch_delay_process, ev, data)
{
  PROCESS_EXITHANDLER(unicast_close(&uc);)
  PROCESS_BEGIN();

  printf("Node addr 0x%02X%02X\n", 
                      linkaddr_node_addr.u8[1], 
                      linkaddr_node_addr.u8[0]);
  printf("     1 Send a message to get the RADIO_DELAY_BEFORE_TX value \n");
  printf("     2 Reinit the receiver, to get the RADIO_DELAY_BEFORE_RX value\n");
  printf("     3 modify the channel value:    0 [channel value]\n");
  printf("     4 Set the radio to on\n");
  printf("     5 Set the radio to off\n");
  printf("     6 Read 127 bytes\n");
  printf("     7 Write 69 bytes\n");
  printf("You need to set the MACRO RADIO_DELAY_MEASUREMENT to 1.\n");
  printf("The time computation have an overhead.\n");

  printf("DURATION_OVERHEAD:\n");
  DURATION(do{} while(0));

  unicast_open(&uc, RIME_CHANNEL, &uc_cb);

  for(;;) {
    PROCESS_WAIT_EVENT();
    /* master part */
    if(ev == serial_line_event_message) {
      /* we convert the input string data to tow int using strlol see 
      https://www.tutorialspoint.com/c_standard_library/c_function_strtol.htm */
      char * str;
      uint8_t mode = strtol(data, &str, 16);

      PRINTF("Received line: %s\n", (char *)data);
      if(mode == 0x01){ /* ranging request */
        uint16_t dest = 0XFFFF;
        printf("Forward a 'hello world' (dest : 0x%.4X)\n", dest);
        /* we are the source of the ranging request */
        linkaddr_t addr;
        addr.u8[0]= dest & 0xFF;
        addr.u8[1]= (dest >> 8) & 0xFF;
        
        packetbuf_copyfrom("Hello World", 11);
        unicast_send(&uc, &addr);
      }
      else if(mode == 0x2){
        printf("Re init RX\n"); 
        NETSTACK_RADIO.off();
        NETSTACK_RADIO.on();
      }
      else if(mode == 0x03 ){ 
        uint16_t channel = 0;
        NETSTACK_RADIO.get_value(RADIO_PARAM_CHANNEL, (radio_value_t*) &channel);
        printf("previous channel: 0x%.4X %u\n", channel, (unsigned int) channel);

        channel = strtol(str, &str, 16);
        printf("new channel: 0x%.4X %u\n", channel, (unsigned int) channel);

        DURATION(NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, (radio_value_t) channel));
      }
      else if(mode == 0x04){ 
        printf("radio on\n");
      
        DURATION(NETSTACK_RADIO.on());
      }
      else if(mode == 0x05){ 
        printf("radio off\n");
        
        DURATION(NETSTACK_RADIO.off());
      }
      else if(mode == 0x06){
        uint8_t buffer[127];
        printf("Read 127 bytes\n");
        
        // Buffer empty can not read a message
        // DURATION(NETSTACK_RADIO.read((void*) &buffer[0], 127));
        DURATION(dw_read_reg(DW_REG_RX_BUFFER, 127, (uint8_t *)&buffer[0]););
        printf("Write 127 bytes\n");
        
        DURATION(NETSTACK_RADIO.prepare((void*) &buffer[0], 127));
      }
      else if(mode == 0x07){
        uint8_t buffer[69];
        printf("Read 69 bytes\n");
        
        // DURATION(NETSTACK_RADIO.read((void*) &buffer[0], 69));
        DURATION(dw_read_reg(DW_REG_RX_BUFFER, 69, (uint8_t *)&buffer[0]););

        printf("Write 69 bytes\n");
        
        DURATION(NETSTACK_RADIO.prepare((void*) &buffer[0], 69));
      }
    }
  }
  PROCESS_END();
}