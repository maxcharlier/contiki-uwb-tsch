/*
 * Copyright (c) 2014, SICS Swedish ICT.
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
 *         ITSCH MAC localization manager.
 * \author
 *         Maximilien Charlier <maximilien.charlier@umons.ac.be>
 */


#include "contiki.h"
#include "net/mac/tsch/tsch-packet.h"
#include "net/mac/tsch/tsch-prop.h"
#include "net/mac/tsch/tsch-queue.h"
#include "net/mac/tsch/tsch-schedule.h"
#include "dev/radio.h"


#include <stdio.h>
#include <string.h>


#ifndef TSCH_LOC_THREAD
  PROCESS(TSCH_PROP_PROCESS, "TSCH propagation time process");

  /*---------------------------------------------------------------------------*/
  /* Protothread for slot operation, called by update_neighbor_prop_time() 
   * function. "data" is a struct tsch_neighbor pointer.*/
  PROCESS_THREAD(TSCH_PROP_PROCESS, ev, data)
  {
    PROCESS_BEGIN();

    // PROCESS_PAUSE();

    printf("tsch_loc_operation start\n");

    while(1) {
      PROCESS_WAIT_EVENT();
      // printf("Got event number %d\n", ev);
      if(ev == PROCESS_EVENT_MSG){
        printf("New prop time %ld %lu %u\n", 
          ((struct tsch_neighbor *) data)->last_prop_time.prop_time, 
          ((struct tsch_neighbor *) data)->last_prop_time.last_mesureament,
          ((struct tsch_neighbor *) data)->last_prop_time.tsch_channel);
      }
    }

    PROCESS_END();
  }
#endif /* TSCH_LOC_THREAD */

/*---------------------------------------------------------------------------*/
/**
 * Tell if a slot is active or not 
 * The slot is ative if we have a packet to send or 
 * If the slot is a receive slot or
 * If the slot is a localisation slot in RX or
 * A localisation slot in TX with a unicast link. */
int
is_active_timeslot(struct tsch_packet *p, struct tsch_neighbor *n, 
              struct tsch_link *link)
{
  return p != NULL || 
       (link->link_options & LINK_OPTION_RX) ||
       (link->link_options & LINK_OPTION_TX && link->link_type == LINK_TYPE_LOC);
}


/*---------------------------------------------------------------------------*/
/* Update the propagation time between the node and his neighbor */
void
update_neighbor_prop_time(struct tsch_neighbor *n, int32_t prop_time, 
                          rtimer_clock_t last_mesureament, uint8_t tsch_channel)
{
  struct tsch_prop_time n_prop_time;
  n_prop_time.prop_time = prop_time;
  n_prop_time.last_mesureament = last_mesureament;
  n_prop_time.tsch_channel = tsch_channel;
  n->last_prop_time = n_prop_time;

  /* printf("TSCH-prop %ld %lu\n", 
          prop_time, 
          last_mesureament); */

  /* Send the PROCESS_EVENT_MSG event asynchronously to 
  "tsch_loc_operation", with a pointer to the tsch_neighbor. */
  process_post(&TSCH_PROP_PROCESS,
                PROCESS_EVENT_MSG, (void *) n);
}

