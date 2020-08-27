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


#undef LPM_CONF_ENABLE
#define LPM_CONF_ENABLE 0

// #define TSCH_DEBUG_SCHEDULE 1
// #define TSCH_DEBUG_QUEUE 1
// #define DEBUG_STARTUP 1
// #define DEBUG_GPIO_TSCH 1


/* Netstack layers */
#undef NETSTACK_CONF_MAC
#define NETSTACK_CONF_MAC     tschmac_driver
#undef NETSTACK_CONF_RDC
#define NETSTACK_CONF_RDC     nordc_driver
#undef NETSTACK_CONF_FRAMER
#define NETSTACK_CONF_FRAMER  framer_802154

/* IEEE802.15.4 frame version */
#undef FRAME802154_CONF_VERSION
#define FRAME802154_CONF_VERSION FRAME802154_IEEE802154E_2012

/* TSCH and RPL callbacks */
#define RPL_CALLBACK_PARENT_SWITCH tsch_rpl_callback_parent_switch
#define RPL_CALLBACK_NEW_DIO_INTERVAL tsch_rpl_callback_new_dio_interval
#define TSCH_CALLBACK_JOINING_NETWORK tsch_rpl_callback_joining_network
#define TSCH_CALLBACK_LEAVING_NETWORK tsch_rpl_callback_leaving_network

/* Needed for CC2538 platforms only */
/* For TSCH we have to use the more accurate crystal oscillator
 * by default the RC oscillator is activated */
#undef SYS_CTRL_CONF_OSC32K_USE_XTAL
#define SYS_CTRL_CONF_OSC32K_USE_XTAL 1

/* TSCH logging. 0: disabled. 1: basic log. 2: with delayed
 * log messages from interrupt */
#undef TSCH_LOG_CONF_LEVEL
#define TSCH_LOG_CONF_LEVEL 0

/* We use a specific thread for the localisation */
#define TSCH_LOC_THREAD 1


/* QUEUEBUF_CONF_NUM specifies the number of queue buffers. 
We increase the number to 16 because we have up to 16 nodes */
// #define QUEUEBUF_CONF_NUM  16

/* Only send on time a unicast message */
#define TSCH_CONF_MAC_MAX_FRAME_RETRIES  0

/* Do not start TSCH at init, wait for NETSTACK_MAC.on() */
#undef TSCH_CONF_AUTOSTART
#define TSCH_CONF_AUTOSTART 0

/* Disable the 6TiSCH minimal schedule */
#define TSCH_SCHEDULE_CONF_WITH_6TISCH_MINIMAL 0


#undef TSCH_LOG_CONF_ID_FROM_LINKADDR
#define TSCH_LOG_CONF_ID_FROM_LINKADDR(addr) ((addr) ? (addr)->u8[LINKADDR_SIZE - 2] : 0)


#if CONTIKI_TARGET_CC2538DK || CONTIKI_TARGET_ZOUL || \
  CONTIKI_TARGET_OPENMOTE_CC2538
#define TSCH_CONF_HW_FRAME_FILTERING    0
#endif /* CONTIKI_TARGET_CC2538DK || CONTIKI_TARGET_ZOUL \
       || CONTIKI_TARGET_OPENMOTE_CC2538 */

/* RPL */

#undef NBR_TABLE_CONF_MAX_NEIGHBORS
#undef UIP_CONF_MAX_ROUTES
#define NBR_TABLE_CONF_MAX_NEIGHBORS      16
#define UIP_CONF_MAX_ROUTES               3

/* Define as minutes */
#define RPL_CONF_DEFAULT_LIFETIME_UNIT   60

/* 10 minutes lifetime of routes */
#define RPL_CONF_DEFAULT_LIFETIME        10

#define RPL_CONF_WITH_STORING 1

//8 is the default value
#define QUEUEBUF_CONF_NUM                    8

// 4 is the default value
#define TSCH_CONF_MAX_INCOMING_PACKETS      4

// enable IPv6 Neigborg solicitation
#define UIP_CONF_ND6_SEND_NS          1



// Set fixed RPL Parent for all nodes
#if NODEID == 0x01
    #define RPL_DIO_DISCARD_RULE if( ((uint8_t*) &from)[14] != 0x0 && ((uint8_t*) &from)[15] != 0x1){\
  goto discard; \
 } 

#elif NODEID == 0x02
    #define RPL_DIO_DISCARD_RULE if( ((uint8_t*) &from)[14] != 0x0 && ((uint8_t*) &from)[15] != 0x1){\
  goto discard; \
 } 

#elif NODEID == 0x03
    #define RPL_DIO_DISCARD_RULE if( ((uint8_t*) &from)[14] != 0x0 && ((uint8_t*) &from)[15] != 0x1){\
  goto discard; \
 } 

#elif NODEID == 0x04
    #define RPL_DIO_DISCARD_RULE if( ((uint8_t*) &from)[14] != 0x0 && ((uint8_t*) &from)[15] != 0x1){\
  goto discard; \
 } 

#elif NODEID == 0x05
    #define RPL_DIO_DISCARD_RULE if( ((uint8_t*) &from)[14] != 0x0 && ((uint8_t*) &from)[15] != 0x6){\
  goto discard; \
 } 

#elif NODEID == 0x06
    #define RPL_DIO_DISCARD_RULE if( ((uint8_t*) &from)[14] != 0x0 && ((uint8_t*) &from)[15] != 0x1){\
  goto discard; \
 } 

#elif NODEID == 0x07
    #define RPL_DIO_DISCARD_RULE if( ((uint8_t*) &from)[14] != 0x0 && ((uint8_t*) &from)[15] != 0x3){\
  goto discard; \
 } 

#elif NODEID == 0x08
    #define RPL_DIO_DISCARD_RULE if( ((uint8_t*) &from)[14] != 0x0 && ((uint8_t*) &from)[15] != 0x6){\
  goto discard; \
 } 

#elif NODEID == 0x09
    #define RPL_DIO_DISCARD_RULE if( ((uint8_t*) &from)[14] != 0x0 && ((uint8_t*) &from)[15] != 0x2){\
  goto discard; \
 } 

#elif NODEID == 0x0A
    #define RPL_DIO_DISCARD_RULE if( ((uint8_t*) &from)[14] != 0x0 && ((uint8_t*) &from)[15] != 0x1){\
  goto discard; \
 } 

#elif NODEID == 0x0B
    #define RPL_DIO_DISCARD_RULE if( ((uint8_t*) &from)[14] != 0x0 && ((uint8_t*) &from)[15] != 0xA){\
  goto discard; \
 } 

#elif NODEID == 0x0C
    #define RPL_DIO_DISCARD_RULE if( ((uint8_t*) &from)[14] != 0x0 && ((uint8_t*) &from)[15] != 0xA){\
  goto discard; \
 } 

#elif NODEID == 0x0D
    #define RPL_DIO_DISCARD_RULE if( ((uint8_t*) &from)[14] != 0x0 && ((uint8_t*) &from)[15] != 0x10){\
  goto discard; \
 } 

#elif NODEID == 0x0E
    #define RPL_DIO_DISCARD_RULE if( ((uint8_t*) &from)[14] != 0x0 && ((uint8_t*) &from)[15] != 0x10){\
  goto discard; \
 } 

#elif NODEID == 0x0F
    #define RPL_DIO_DISCARD_RULE if( ((uint8_t*) &from)[14] != 0x0 && ((uint8_t*) &from)[15] != 0x10){\
  goto discard; \
 } 

#elif NODEID == 0x10
    #define RPL_DIO_DISCARD_RULE if( ((uint8_t*) &from)[14] != 0x0 && ((uint8_t*) &from)[15] != 0x7){\
  goto discard; \
 } 

#elif NODEID == 0x11
    #define RPL_DIO_DISCARD_RULE if( ((uint8_t*) &from)[14] != 0x0 && ((uint8_t*) &from)[15] != 0x10){\
  goto discard; \
 } 

#elif NODEID == 0x12
    #define RPL_DIO_DISCARD_RULE if( ((uint8_t*) &from)[14] != 0x0 && ((uint8_t*) &from)[15] != 0x3){\
  goto discard; \
 } 

#endif /* NODEID */


#endif /* __PROJECT_CONF_H__ */
