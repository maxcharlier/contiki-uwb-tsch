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

#ifndef __TSCH_LOC_H__
#define __TSCH_LOC_H__

/********** Includes **********/

#include "contiki.h"
#include "net/mac/tsch/tsch-asn.h"
#include "net/mac/tsch/tsch-packet.h"
#include "net/mac/tsch/tsch-queue.h"
#include "net/mac/tsch/tsch-schedule.h"

/********** Propagation time Process *********/
PROCESS_NAME(TSCH_PROP_PROCESS);

/********** Type *********/

/* tsch_prop_time is defined in tsch-queue.h to avoid loop in declaration. */

/********** Functions *********/

void update_neighbor_prop_time(struct tsch_neighbor *n, int32_t prop_time, struct tsch_asn_t * asn, uint8_t tsch_channel);

int32_t compute_prop_time(int32_t initiator_roundtrip, 
      int32_t initiator_reply, int32_t replier_roundtrip, 
      int32_t replier_reply);

#endif /* __TSCH_LOC_H__ */
