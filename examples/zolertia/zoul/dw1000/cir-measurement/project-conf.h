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
 *         Project config file
 * \author
 *         Simon Duquennoy <simon.duquennoy@inria.fr>
 *
 */

#ifndef __PROJECT_CONF_H__
#define __PROJECT_CONF_H__

/* Netstack layers */
/* Network setup for non-IPv6 (rime). */
#define NETSTACK_CONF_NETWORK        rime_driver
#define NETSTACK_CONF_MAC            nullmac_driver
#define NETSTACK_CONF_RDC            nullrdc_driver
#define NETSTACK_CONF_FRAMER         framer_802154
#define NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE 8

#define COLLECT_CONF_ANNOUNCEMENTS           1
#define RIME_CONF_NO_POLITE_ANNOUCEMENTS     0


#define COLLECT_NBR_TABLE_CONF_MAX_NEIGHBORS 32

#ifndef QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM                    8
#endif

/* Transceiver configuration */
#define DW1000_CHANNEL              3
#define DW1000_DATA_RATE            DW_DATA_RATE_6800_KBPS
/*#define DW1000_PREAMBLE             DW_PREAMBLE_LENGTH_256 */
#define DW1000_PRF                  DW_PRF_64_MHZ

/* enable CIR memory accumulator */
#define ENABLE_ACCUMULATOR_CIR      1

#define DW1000_TSCH                 0

/* Specify a minimum packet size for 6lowpan compression to be
   enabled. This is needed for ContikiMAC, which needs packets to be
   larger than a specified size, if no ContikiMAC header should be
   used. */
#define SICSLOWPAN_CONF_COMPRESSION_THRESHOLD 63

#define DW1000_CONF_AUTOACK                   1
#define NETSTACK_RDC_CHANNEL_CHECK_RATE       8


#define PACKETBUF_CONF_ATTRS_INLINE       1

/* Declare the usage of a specific tread for the localisation */
#define TSCH_LOC_THREAD 1

/* IEEE802.15.4 PANID */
#undef IEEE802154_CONF_PANID
#define IEEE802154_CONF_PANID 0xabcd

#if CONTIKI_TARGET_COOJA
#define COOJA_CONF_SIMULATE_TURNAROUND 0
#endif /* CONTIKI_TARGET_COOJA */


#endif /* __PROJECT_CONF_H__ */
