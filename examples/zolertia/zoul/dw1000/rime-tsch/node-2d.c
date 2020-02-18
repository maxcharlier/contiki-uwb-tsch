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



const linkaddr_t sink_addr =    { { 0X00, 0XD0 } };
static unsigned char buf[MAX_PAYLOAD_LEN];
static uint8_t current_index = 0;

/*---------------------------------------------------------------------------*/
PROCESS(unicast_test_process, "Rime Node");
PROCESS(TSCH_PROP_PROCESS, "TSCH localization process");
AUTOSTART_PROCESSES(&unicast_test_process);

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
      
      // printf("Node 0X%02X prop time %ld %lu %u\n", 
      //   ((struct tsch_neighbor *) data)->addr.u8[sizeof(linkaddr_t)-1],
      //   ((struct tsch_neighbor *) data)->last_prop_time.prop_time, 
      //   ((struct tsch_neighbor *) data)->last_prop_time.last_mesureament,
      //   ((struct tsch_neighbor *) data)->last_prop_time.tsch_channel);

      create_buffer((struct tsch_neighbor *) data);
      print_buffer();
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

  tsch_set_coordinator(linkaddr_cmp(&sink_addr, &linkaddr_node_addr));

  NETSTACK_MAC.on();

  while(1) {
    static struct etimer et;

    etimer_set(&et, CLOCK_SECOND*10);

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
