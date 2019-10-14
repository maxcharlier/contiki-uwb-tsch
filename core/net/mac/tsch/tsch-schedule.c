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
 *         IEEE 802.15.4 TSCH MAC schedule manager.
 * \author
 *         Simon Duquennoy <simonduq@sics.se>
 *         Beshr Al Nahas <beshr@sics.se>
 */

#include "contiki.h"
#include "dev/leds.h"
#include "lib/memb.h"
#include "net/nbr-table.h"
#include "net/packetbuf.h"
#include "net/queuebuf.h"
#include "net/mac/tsch/tsch.h"
#include "net/mac/tsch/tsch-queue.h"
#include "net/mac/tsch/tsch-private.h"
#include "net/mac/tsch/tsch-packet.h"
#include "net/mac/tsch/tsch-schedule.h"
#include "net/mac/tsch/tsch-log.h"
#include "net/mac/frame802154.h"
#include "sys/process.h"
#include "sys/rtimer.h"
#include <string.h>

#if TSCH_LOG_LEVEL >= 1
#define DEBUG DEBUG_PRINT
#else /* TSCH_LOG_LEVEL */
#define DEBUG DEBUG_NONE
#endif /* TSCH_LOG_LEVEL */
#include "net/net-debug.h"

#define INDEX_NODE_ID   (sizeof(linkaddr_t)-1)

/* 802.15.4 broadcast MAC address  */
// static linkaddr_t node_1_address = { { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff } };

static linkaddr_t node_1_address = { { 0XFF, 0XFF } };
static linkaddr_t node_2_address = { { 0XFF, 0XFF } };
static linkaddr_t node_3_address = { { 0XFF, 0XFF } };
static linkaddr_t node_4_address = { { 0XFF, 0XFF } };
static linkaddr_t node_5_address = { { 0XFF, 0XFF } };
static linkaddr_t node_6_address = { { 0XFF, 0XFF } };
static linkaddr_t node_7_address = { { 0XFF, 0XFF } };
static linkaddr_t node_8_address = { { 0XFF, 0XFF } };
static linkaddr_t node_9_address = { { 0XFF, 0XFF } };
static linkaddr_t node_A_address = { { 0XFF, 0XFF } };
static linkaddr_t node_B_address = { { 0XFF, 0XFF } };
static linkaddr_t anchor_1_address = { { 0XFF, 0XFF } };
static linkaddr_t anchor_2_address = { { 0XFF, 0XFF } };
static linkaddr_t anchor_3_address = { { 0XFF, 0XFF } };
static linkaddr_t anchor_4_address = { { 0XFF, 0XFF } };
static linkaddr_t sink_1_address = { { 0XFF, 0XFF } };

static linkaddr_t * anchors_addr[5] = {&anchor_1_address, &anchor_2_address, &anchor_3_address, 
  &anchor_4_address, &sink_1_address};

/* Pre-allocated space for links */
MEMB(link_memb, struct tsch_link, TSCH_SCHEDULE_MAX_LINKS);
/* Pre-allocated space for slotframes */
MEMB(slotframe_memb, struct tsch_slotframe, TSCH_SCHEDULE_MAX_SLOTFRAMES);
/* List of slotframes (each slotframe holds its own list of links) */
LIST(slotframe_list);

/* Adds and returns a slotframe (NULL if failure) */
struct tsch_slotframe *
tsch_schedule_add_slotframe(uint16_t handle, uint16_t size)
{
  if(size == 0) {
    return NULL;
  }

  if(tsch_schedule_get_slotframe_by_handle(handle)) {
    /* A slotframe with this handle already exists */
    return NULL;
  }

  if(tsch_get_lock()) {
    struct tsch_slotframe *sf = memb_alloc(&slotframe_memb);
    if(sf != NULL) {
      /* Initialize the slotframe */
      sf->handle = handle;
      TSCH_ASN_DIVISOR_INIT(sf->size, size);
      LIST_STRUCT_INIT(sf, links_list);
      /* Add the slotframe to the global list */
      list_add(slotframe_list, sf);
    }
    PRINTF("TSCH-schedule: add_slotframe %u %u\n",
           handle, size);
    tsch_release_lock();
    return sf;
  }
  return NULL;
}
/*---------------------------------------------------------------------------*/
/* Removes all slotframes, resulting in an empty schedule */
int
tsch_schedule_remove_all_slotframes(void)
{
  struct tsch_slotframe *sf;
  while((sf = list_head(slotframe_list))) {
    if(tsch_schedule_remove_slotframe(sf) == 0) {
      return 0;
    }
  }
  return 1;
}
/*---------------------------------------------------------------------------*/
/* Removes a slotframe Return 1 if success, 0 if failure */
int
tsch_schedule_remove_slotframe(struct tsch_slotframe *slotframe)
{
  if(slotframe != NULL) {
    /* Remove all links belonging to this slotframe */
    struct tsch_link *l;
    while((l = list_head(slotframe->links_list))) {
      tsch_schedule_remove_link(slotframe, l);
    }

    /* Now that the slotframe has no links, remove it. */
    if(tsch_get_lock()) {
      PRINTF("TSCH-schedule: remove slotframe %u %u\n", slotframe->handle, slotframe->size.val);
      memb_free(&slotframe_memb, slotframe);
      list_remove(slotframe_list, slotframe);
      tsch_release_lock();
      return 1;
    }
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
/* Looks for a slotframe from a handle */
struct tsch_slotframe *
tsch_schedule_get_slotframe_by_handle(uint16_t handle)
{
  if(!tsch_is_locked()) {
    struct tsch_slotframe *sf = list_head(slotframe_list);
    while(sf != NULL) {
      if(sf->handle == handle) {
        return sf;
      }
      sf = list_item_next(sf);
    }
  }
  return NULL;
}
/*---------------------------------------------------------------------------*/
/* Looks for a link from a handle */
struct tsch_link *
tsch_schedule_get_link_by_handle(uint16_t handle)
{
  if(!tsch_is_locked()) {
    struct tsch_slotframe *sf = list_head(slotframe_list);
    while(sf != NULL) {
      struct tsch_link *l = list_head(sf->links_list);
      /* Loop over all items. Assume there is max one link per timeslot */
      while(l != NULL) {
        if(l->handle == handle) {
          return l;
        }
        l = list_item_next(l);
      }
      sf = list_item_next(sf);
    }
  }
  return NULL;
}
/*---------------------------------------------------------------------------*/
/* Adds a link to a slotframe, return a pointer to it (NULL if failure) */
struct tsch_link *
tsch_schedule_add_link(struct tsch_slotframe *slotframe,
                       uint8_t link_options, enum link_type link_type, const linkaddr_t *address,
                       uint16_t timeslot, uint16_t channel_offset)
{
  struct tsch_link *l = NULL;
  if(slotframe != NULL) {
    /* We currently support only one link per timeslot in a given slotframe. */
    /* Start with removing the link currently installed at this timeslot (needed
     * to keep neighbor state in sync with link options etc.) */
    tsch_schedule_remove_link_by_timeslot(slotframe, timeslot);
    if(!tsch_get_lock()) {
      PRINTF("TSCH-schedule:! add_link memb_alloc couldn't take lock\n");
    } else {
      l = memb_alloc(&link_memb);
      if(l == NULL) {
        PRINTF("TSCH-schedule:! add_link memb_alloc failed\n");
        tsch_release_lock();
      } else {
        static int current_link_handle = 0;
        struct tsch_neighbor *n;
        /* Add the link to the slotframe */
        list_add(slotframe->links_list, l);
        /* Initialize link */
        l->handle = current_link_handle++;
        l->link_options = link_options;
        l->link_type = link_type;
        l->slotframe_handle = slotframe->handle;
        l->timeslot = timeslot;
        l->channel_offset = channel_offset;
        l->data = NULL;
        if(address == NULL) {
          address = &linkaddr_null;
        }
        linkaddr_copy(&l->addr, address);

        PRINTF("TSCH-schedule: add_link %u %u %u %u %u %u\n",
               slotframe->handle, link_options, link_type, timeslot, channel_offset, TSCH_LOG_ID_FROM_LINKADDR(address));

        /* Release the lock before we update the neighbor (will take the lock) */
        tsch_release_lock();

        if(l->link_options & LINK_OPTION_TX) {
          n = tsch_queue_add_nbr(&l->addr);
          /* We have a tx link to this neighbor, update counters */
          if(n != NULL && !(l->link_type == LINK_TYPE_LOC)) {
            n->tx_links_count++;
            if(!(l->link_options & LINK_OPTION_SHARED)) {
              n->dedicated_tx_links_count++;
            }
          }
        }
      }
    }
  }
  return l;
}
/*---------------------------------------------------------------------------*/
/* Removes a link from slotframe. Return 1 if success, 0 if failure */
int
tsch_schedule_remove_link(struct tsch_slotframe *slotframe, struct tsch_link *l)
{
  if(slotframe != NULL && l != NULL && l->slotframe_handle == slotframe->handle) {
    if(tsch_get_lock()) {
      uint8_t link_options;
      linkaddr_t addr;

      /* Save link option and addr in local variables as we need them
       * after freeing the link */
      link_options = l->link_options;
      linkaddr_copy(&addr, &l->addr);

      /* The link to be removed is scheduled as next, set it to NULL
       * to abort the next link operation */
      if(l == current_link) {
        current_link = NULL;
      }
      PRINTF("TSCH-schedule: remove_link %u %u %u %u %u\n",
             slotframe->handle, l->link_options, l->timeslot, l->channel_offset,
             TSCH_LOG_ID_FROM_LINKADDR(&l->addr));

      list_remove(slotframe->links_list, l);
      memb_free(&link_memb, l);

      /* Release the lock before we update the neighbor (will take the lock) */
      tsch_release_lock();

      /* This was a tx link to this neighbor, update counters */
      if(link_options & LINK_OPTION_TX) {
        struct tsch_neighbor *n = tsch_queue_add_nbr(&addr);
        if(n != NULL) {
          n->tx_links_count--;
          if(!(link_options & LINK_OPTION_SHARED)) {
            n->dedicated_tx_links_count--;
          }
        }
      }

      return 1;
    } else {
      PRINTF("TSCH-schedule:! remove_link memb_alloc couldn't take lock\n");
    }
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
/* Removes a link from slotframe and timeslot. Return a 1 if success, 0 if failure */
int
tsch_schedule_remove_link_by_timeslot(struct tsch_slotframe *slotframe, uint16_t timeslot)
{
  return slotframe != NULL &&
         tsch_schedule_remove_link(slotframe, tsch_schedule_get_link_by_timeslot(slotframe, timeslot));
}
/*---------------------------------------------------------------------------*/
/* Looks within a slotframe for a link with a given timeslot */
struct tsch_link *
tsch_schedule_get_link_by_timeslot(struct tsch_slotframe *slotframe, uint16_t timeslot)
{
  if(!tsch_is_locked()) {
    if(slotframe != NULL) {
      struct tsch_link *l = list_head(slotframe->links_list);
      /* Loop over all items. Assume there is max one link per timeslot */
      while(l != NULL) {
        if(l->timeslot == timeslot) {
          return l;
        }
        l = list_item_next(l);
      }
      return l;
    }
  }
  return NULL;
}
/*---------------------------------------------------------------------------*/
/* Returns the next active link after a given ASN, and a backup link (for the same ASN, with Rx flag) */
struct tsch_link *
tsch_schedule_get_next_active_link(struct tsch_asn_t *asn, uint16_t *time_offset,
    struct tsch_link **backup_link)
{
  uint16_t time_to_curr_best = 0;
  struct tsch_link *curr_best = NULL;
  struct tsch_link *curr_backup = NULL; /* Keep a back link in case the current link
  turns out useless when the time comes. For instance, for a Tx-only link, if there is
  no outgoing packet in queue. In that case, run the backup link instead. The backup link
  must have Rx flag set. */
  if(!tsch_is_locked()) {
    struct tsch_slotframe *sf = list_head(slotframe_list);
    /* For each slotframe, look for the earliest occurring link */
    while(sf != NULL) {
      /* Get timeslot from ASN, given the slotframe length */
      uint16_t timeslot = TSCH_ASN_MOD(*asn, sf->size);
      struct tsch_link *l = list_head(sf->links_list);
      while(l != NULL) {
        uint16_t time_to_timeslot =
          l->timeslot > timeslot ?
          l->timeslot - timeslot :
          sf->size.val + l->timeslot - timeslot;
        if(curr_best == NULL || time_to_timeslot < time_to_curr_best) {
          time_to_curr_best = time_to_timeslot;
          curr_best = l;
          curr_backup = NULL;
        } else if(time_to_timeslot == time_to_curr_best) {
          struct tsch_link *new_best = NULL;
          /* Two links are overlapping, we need to select one of them.
           * By standard: prioritize Tx links first, second by lowest handle */
          if((curr_best->link_options & LINK_OPTION_TX) == (l->link_options & LINK_OPTION_TX)) {
            /* Both or neither links have Tx, select the one with lowest handle */
            if(l->slotframe_handle < curr_best->slotframe_handle) {
              new_best = l;
            }
          } else {
            /* Select the link that has the Tx option */
            if(l->link_options & LINK_OPTION_TX) {
              new_best = l;
            }
          }

          /* Maintain backup_link */
          if(curr_backup == NULL) {
            /* Check if 'l' best can be used as backup */
            if(new_best != l && (l->link_options & LINK_OPTION_RX)) { /* Does 'l' have Rx flag? */
              curr_backup = l;
            }
            /* Check if curr_best can be used as backup */
            if(new_best != curr_best && (curr_best->link_options & LINK_OPTION_RX)) { /* Does curr_best have Rx flag? */
              curr_backup = curr_best;
            }
          }

          /* Maintain curr_best */
          if(new_best != NULL) {
            curr_best = new_best;
          }
        }

        l = list_item_next(l);
      }
      sf = list_item_next(sf);
    }
    if(time_offset != NULL) {
      *time_offset = time_to_curr_best;
    }
  }
  if(backup_link != NULL) {
    *backup_link = curr_backup;
  }
  return curr_best;
}

/*---------------------------------------------------------------------------*/
/* Module initialization, call only once at startup. Returns 1 is success, 0 if failure. */
int
tsch_schedule_init(void)
{
  if(tsch_get_lock()) {
    memb_init(&link_memb);
    memb_init(&slotframe_memb);
    list_init(slotframe_list);
    tsch_release_lock();
    return 1;
  } else {
    return 0;
  }
}
/*---------------------------------------------------------------------------*/
/* Create a 6TiSCH minimal schedule */
void
tsch_schedule_create_minimal_old(void)
// tsch_schedule_create_minimal(void)
{
  struct tsch_slotframe *sf_min;
  /* First, empty current schedule */
  tsch_schedule_remove_all_slotframes();
  /* Build 6TiSCH minimal schedule.
   * We pick a slotframe length of TSCH_SCHEDULE_DEFAULT_LENGTH */
  sf_min = tsch_schedule_add_slotframe(0, TSCH_SCHEDULE_DEFAULT_LENGTH);
  /* Add a single Tx|Rx|Shared slot using broadcast address (i.e. usable for unicast and broadcast).
   * We set the link type to advertising, which is not compliant with 6TiSCH minimal schedule
   * but is required according to 802.15.4e if also used for EB transmission.
   * Timeslot: 0, channel offset: 0. */
  tsch_schedule_add_link(sf_min,
      LINK_OPTION_RX | LINK_OPTION_TX | LINK_OPTION_SHARED | LINK_OPTION_TIME_KEEPING,
      LINK_TYPE_ADVERTISING, &tsch_broadcast_address,
      0, 0);
}
/*---------------------------------------------------------------------------*/
/* Create a 6TiSCH schedule to test the propagation time feature. */
void
tsch_schedule_create_minimal_test_loc(void)
// tsch_schedule_create_minimal(void)
{
  struct tsch_slotframe *sf_custom;
  uint8_t offset = 5;
  /* First, empty current schedule */
  tsch_schedule_remove_all_slotframes();
  /* Build 6TiSCH minimal schedule.
   * We pick a slotframe length of TSCH_SCHEDULE_DEFAULT_LENGTH */
  sf_custom = tsch_schedule_add_slotframe(0, TSCH_SCHEDULE_DEFAULT_LENGTH);
  /* Add a single Tx|Rx|Shared slot using broadcast address (i.e. usable for unicast and broadcast).
   * We set the link type to advertising, which is not compliant with 6TiSCH minimal schedule
   * but is required according to 802.15.4e if also used for EB transmission.
   * Timeslot: 0, channel offset: 0. */
  tsch_schedule_add_link(sf_custom,
      LINK_OPTION_RX | LINK_OPTION_TX | LINK_OPTION_SHARED | LINK_OPTION_TIME_KEEPING,
      LINK_TYPE_ADVERTISING, &tsch_broadcast_address,
      0, 0);

  linkaddr_copy(&node_1_address, &linkaddr_node_addr);
  node_1_address.u8[INDEX_NODE_ID] = 0X01;
  linkaddr_copy(&node_2_address, &linkaddr_node_addr);
  node_2_address.u8[INDEX_NODE_ID] = 0x02;

  for (int i = 1; i < 5; i++){
    if(linkaddr_node_addr.u8[INDEX_NODE_ID] == node_1_address.u8[INDEX_NODE_ID]){
      tsch_schedule_add_link(sf_custom,
          LINK_OPTION_RX, LINK_TYPE_LOC, &node_2_address, offset * i, 0);
    printf("localisation schedule1\n");
    }
    if(linkaddr_node_addr.u8[INDEX_NODE_ID] == node_2_address.u8[INDEX_NODE_ID]){
      tsch_schedule_add_link(sf_custom,
          LINK_OPTION_TX, LINK_TYPE_LOC, &node_1_address, offset * i, 0);
    printf("localisation schedule2\n");
    }
  }

  tsch_schedule_print();
}
/*---------------------------------------------------------------------------*/
/* Create a 6TiSCH linear schedule with concurrent communications*/
void
tsch_schedule_create_minimal5(void)
{
  struct tsch_slotframe *sf_custom;
  /* First, empty current schedule */
  tsch_schedule_remove_all_slotframes();
  /* A slotframe is define by an handle (a unique number) and a length
   * We pick a slotframe length of TSCH_SCHEDULE_DEFAULT_LENGTH */
  sf_custom = tsch_schedule_add_slotframe(0, TSCH_SCHEDULE_DEFAULT_LENGTH);


  linkaddr_copy(&node_6_address, &linkaddr_node_addr);
  node_6_address.u8[INDEX_NODE_ID] = 0x6;
  linkaddr_copy(&node_7_address, &linkaddr_node_addr);
  node_7_address.u8[INDEX_NODE_ID] = 0x7;
  linkaddr_copy(&node_8_address, &linkaddr_node_addr);
  node_8_address.u8[INDEX_NODE_ID] = 0x8;
  linkaddr_copy(&node_9_address, &linkaddr_node_addr);
  node_9_address.u8[INDEX_NODE_ID] = 0x9;
  linkaddr_copy(&node_A_address, &linkaddr_node_addr);
  node_A_address.u8[INDEX_NODE_ID] = 0xA;
  linkaddr_copy(&node_B_address, &linkaddr_node_addr);
  node_B_address.u8[INDEX_NODE_ID] = 0xB;

  // printf("link node addr %02x%02x::%02x%02x::%02x%02x::%02x%02x \n",  linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],  linkaddr_node_addr.u8[2],  
  //     linkaddr_node_addr.u8[3],  linkaddr_node_addr.u8[4],  linkaddr_node_addr.u8[5],  linkaddr_node_addr.u8[6], linkaddr_node_addr.u8[INDEX_NODE_ID]);
  
  // printf("0x01 addr %02x%02x::%02x%02x::%02x%02x::%02x%02x \n",  node_1_address.u8[0], node_1_address.u8[1],  node_1_address.u8[2],  
  //     node_1_address.u8[3],  node_1_address.u8[4],  node_1_address.u8[5],  node_1_address.u8[6], node_1_address.u8[INDEX_NODE_ID]);
  // printf("0x02 addr %02x%02x::%02x%02x::%02x%02x::%02x%02x \n",  node_2_address.u8[0], node_2_address.u8[1],  node_2_address.u8[2],  
  //     node_2_address.u8[3],  node_2_address.u8[4],  node_2_address.u8[5],  node_2_address.u8[6], node_2_address.u8[INDEX_NODE_ID]); 

  uint8_t tx_option = LINK_OPTION_TX;


  /* Add a Tx|Rx|Shared slot using broadcast address (i.e. usable for unicast and broadcast).
   * We set the link type to advertising, which is not compliant with 6TiSCH minimal schedule
   * but is required according to 802.15.4e if also used for EB transmission.
   * Timeslot: 0, channel offset: 0. */
  tsch_schedule_add_link(sf_custom,
      LINK_OPTION_RX | LINK_OPTION_TX | LINK_OPTION_SHARED | LINK_OPTION_TIME_KEEPING,
      LINK_TYPE_ADVERTISING, &tsch_broadcast_address,
      0, 0);

  if(linkaddr_node_addr.u8[INDEX_NODE_ID] == 0x0B){
    /* Node TX*/
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_A_address, 4, 0);
  }
  if(linkaddr_node_addr.u8[INDEX_NODE_ID] == 0x0A){
    /* Node RX*/ 
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 4, 0);
    /* Node TX*/
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_9_address, 7, 0);
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_9_address, 13, 0);
  }

  if(linkaddr_node_addr.u8[INDEX_NODE_ID] == 0x09){
    /* Node RX*/ 
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 7, 0);
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 13, 0);
    /* Node TX*/
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_8_address, 10, 0);
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_8_address, 16, 0);
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_8_address, 22, 0);
  }
  if(linkaddr_node_addr.u8[INDEX_NODE_ID] == 0x08){
    /* Node RX*/ 
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 10, 0);
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 16, 0);
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 22, 0);
    /* Node TX*/
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_7_address, 6, 0);
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_7_address, 30, 0);
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_7_address, 33, 0);
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_7_address, 36, 0);
  }
  if(linkaddr_node_addr.u8[INDEX_NODE_ID] == 0x07){
    /* Node RX*/ 
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 6, 0);
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 30, 0);
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 33, 0);
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 36, 0);
    /* Node TX*/
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_6_address, 3, 0);
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_6_address, 9, 0);
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_6_address, 15, 0);
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_6_address, 21, 0);
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_6_address, 27, 0);
  }

  if(linkaddr_node_addr.u8[INDEX_NODE_ID] == 0x06){
    /* Node RX*/ 
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 3, 0);
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 9, 0);
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 15, 0);
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 21, 0);
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 27, 0);
  }


  tsch_schedule_print();
}
/*---------------------------------------------------------------------------*/
/* Create a 6TiSCH linear schedule with concurrent communications*/
void
tsch_schedule_create_minimal6(void)
{
  struct tsch_slotframe *sf_custom;
  /* First, empty current schedule */
  tsch_schedule_remove_all_slotframes();
  /* A slotframe is define by an handle (a unique number) and a length
   * We pick a slotframe length of TSCH_SCHEDULE_DEFAULT_LENGTH */
  sf_custom = tsch_schedule_add_slotframe(0, TSCH_SCHEDULE_DEFAULT_LENGTH);


  linkaddr_copy(&node_6_address, &linkaddr_node_addr);
  node_6_address.u8[INDEX_NODE_ID] = 0x6;
  linkaddr_copy(&node_7_address, &linkaddr_node_addr);
  node_7_address.u8[INDEX_NODE_ID] = 0x7;
  linkaddr_copy(&node_8_address, &linkaddr_node_addr);
  node_8_address.u8[INDEX_NODE_ID] = 0x8;
  linkaddr_copy(&node_9_address, &linkaddr_node_addr);
  node_9_address.u8[INDEX_NODE_ID] = 0x9;
  linkaddr_copy(&node_A_address, &linkaddr_node_addr);
  node_A_address.u8[INDEX_NODE_ID] = 0xA;
  linkaddr_copy(&node_B_address, &linkaddr_node_addr);
  node_B_address.u8[INDEX_NODE_ID] = 0xB;

  // printf("link node addr %02x%02x::%02x%02x::%02x%02x::%02x%02x \n",  linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],  linkaddr_node_addr.u8[2],  
  //     linkaddr_node_addr.u8[3],  linkaddr_node_addr.u8[4],  linkaddr_node_addr.u8[5],  linkaddr_node_addr.u8[6], linkaddr_node_addr.u8[INDEX_NODE_ID]);
  
  // printf("0x01 addr %02x%02x::%02x%02x::%02x%02x::%02x%02x \n",  node_1_address.u8[0], node_1_address.u8[1],  node_1_address.u8[2],  
  //     node_1_address.u8[3],  node_1_address.u8[4],  node_1_address.u8[5],  node_1_address.u8[6], node_1_address.u8[INDEX_NODE_ID]);
  // printf("0x02 addr %02x%02x::%02x%02x::%02x%02x::%02x%02x \n",  node_2_address.u8[0], node_2_address.u8[1],  node_2_address.u8[2],  
  //     node_2_address.u8[3],  node_2_address.u8[4],  node_2_address.u8[5],  node_2_address.u8[6], node_2_address.u8[INDEX_NODE_ID]); 

  uint8_t tx_option = LINK_OPTION_TX;


  /* Add a Tx|Rx|Shared slot using broadcast address (i.e. usable for unicast and broadcast).
   * We set the link type to advertising, which is not compliant with 6TiSCH minimal schedule
   * but is required according to 802.15.4e if also used for EB transmission.
   * Timeslot: 0, channel offset: 0. */
  tsch_schedule_add_link(sf_custom,
      LINK_OPTION_RX | LINK_OPTION_TX | LINK_OPTION_SHARED | LINK_OPTION_TIME_KEEPING,
      LINK_TYPE_ADVERTISING, &tsch_broadcast_address,
      0, 0);

  if(linkaddr_node_addr.u8[INDEX_NODE_ID] == 0x0B){
    /* Node TX*/
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_A_address, 3, 1);
  }
  if(linkaddr_node_addr.u8[INDEX_NODE_ID] == 0x0A){
    /* Node RX*/ 
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 3, 1);
    /* Node TX*/
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_9_address, 6, 1);
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_9_address, 12, 1);
  }

  if(linkaddr_node_addr.u8[INDEX_NODE_ID] == 0x09){
    /* Node RX*/ 
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 6, 1);
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 12, 1);
    /* Node TX*/
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_8_address, 9, 1);
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_8_address, 15, 1);
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_8_address, 21, 1);
  }
  if(linkaddr_node_addr.u8[INDEX_NODE_ID] == 0x08){
    /* Node RX*/ 
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 9, 1);
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 15, 1);
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 21, 1);
    /* Node TX*/
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_7_address, 6, 0);
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_7_address, 12, 0);
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_7_address, 18, 0);
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_7_address, 24, 0);
  }
  if(linkaddr_node_addr.u8[INDEX_NODE_ID] == 0x07){
    /* Node RX*/ 
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 6, 0);
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 12, 0);
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 18, 0);
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 24, 0);
    /* Node TX*/
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_6_address, 3, 0);
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_6_address, 9, 0);
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_6_address, 15, 0);
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_6_address, 21, 0);
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_6_address, 27, 0);
  }

  if(linkaddr_node_addr.u8[INDEX_NODE_ID] == 0x06){
    /* Node RX*/ 
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 3, 0);
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 9, 0);
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 15, 0);
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 21, 0);
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, 27, 0);
  }


  tsch_schedule_print();
}
/*---------------------------------------------------------------------------*/
/* Create a 6TiSCH linear schedule with concurrent communications*/
void
tsch_schedule_create_minimal4(void)
{
  struct tsch_slotframe *sf_custom;
  /* First, empty current schedule */
  tsch_schedule_remove_all_slotframes();
  /* A slotframe is define by an handle (a unique number) and a length
   * We pick a slotframe length of TSCH_SCHEDULE_DEFAULT_LENGTH */
  sf_custom = tsch_schedule_add_slotframe(0, TSCH_SCHEDULE_DEFAULT_LENGTH);


  linkaddr_copy(&node_6_address, &linkaddr_node_addr);
  node_6_address.u8[INDEX_NODE_ID] = 0x6;
  linkaddr_copy(&node_7_address, &linkaddr_node_addr);
  node_7_address.u8[INDEX_NODE_ID] = 0x7;
  linkaddr_copy(&node_8_address, &linkaddr_node_addr);
  node_8_address.u8[INDEX_NODE_ID] = 0x8;
  linkaddr_copy(&node_9_address, &linkaddr_node_addr);
  node_9_address.u8[INDEX_NODE_ID] = 0x9;
  linkaddr_copy(&node_A_address, &linkaddr_node_addr);
  node_A_address.u8[INDEX_NODE_ID] = 0xA;
  linkaddr_copy(&node_B_address, &linkaddr_node_addr);
  node_B_address.u8[INDEX_NODE_ID] = 0xB;

  uint8_t tx_option = LINK_OPTION_TX;
  uint8_t timeslotBA = 3;
  uint8_t timeslotA9 = 6;
  uint8_t timeslot98 = 9;
  uint8_t timeslot87 = 4;
  uint8_t timeslot76 = 7;


  /* Add a Tx|Rx|Shared slot using broadcast address (i.e. usable for unicast and broadcast).
   * We set the link type to advertising, which is not compliant with 6TiSCH minimal schedule
   * but is required according to 802.15.4e if also used for EB transmission.
   * Timeslot: 0, channel offset: 0. */
  tsch_schedule_add_link(sf_custom,
      LINK_OPTION_RX | LINK_OPTION_TX | LINK_OPTION_SHARED | LINK_OPTION_TIME_KEEPING,
      LINK_TYPE_ADVERTISING, &tsch_broadcast_address,
      0, 0);

  if(linkaddr_node_addr.u8[INDEX_NODE_ID] == 0x0B){
    /* Node TX*/
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_A_address, timeslotBA, 0);
  }
  if(linkaddr_node_addr.u8[INDEX_NODE_ID] == 0x0A){
    /* Node RX*/ 
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, timeslotBA, 0);
    /* Node TX*/
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_9_address, timeslotA9, 0);
  }

  if(linkaddr_node_addr.u8[INDEX_NODE_ID] == 0x09){
    /* Node RX*/ 
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, timeslotA9, 0);
    /* Node TX*/
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_8_address, timeslot98, 0);
  }
  if(linkaddr_node_addr.u8[INDEX_NODE_ID] == 0x08){
    /* Node RX*/ 
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, timeslot98, 0);
    /* Node TX*/
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_7_address, timeslot87, 0);
  }
  if(linkaddr_node_addr.u8[INDEX_NODE_ID] == 0x07){
    /* Node RX*/ 
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, timeslot87, 0);
    /* Node TX*/
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_6_address, timeslot76, 0);
  }

  if(linkaddr_node_addr.u8[INDEX_NODE_ID] == 0x06){
    /* Node RX*/ 
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, timeslot76, 0);
  }


  tsch_schedule_print();
}
/*---------------------------------------------------------------------------*/
/* Create a 6TiSCH linear schedule with concurrent communications*/
void
tsch_schedule_create_minimal3(void)
{
  struct tsch_slotframe *sf_custom;
  /* First, empty current schedule */
  tsch_schedule_remove_all_slotframes();
  /* A slotframe is define by an handle (a unique number) and a length
   * We pick a slotframe length of TSCH_SCHEDULE_DEFAULT_LENGTH */
  sf_custom = tsch_schedule_add_slotframe(0, TSCH_SCHEDULE_DEFAULT_LENGTH);


  linkaddr_copy(&node_1_address, &linkaddr_node_addr);
  node_1_address.u8[INDEX_NODE_ID] = 0x1;
  linkaddr_copy(&node_2_address, &linkaddr_node_addr);
  node_2_address.u8[INDEX_NODE_ID] = 0x2;
  linkaddr_copy(&node_3_address, &linkaddr_node_addr);
  node_3_address.u8[INDEX_NODE_ID] = 0x3;
  linkaddr_copy(&node_4_address, &linkaddr_node_addr);
  node_4_address.u8[INDEX_NODE_ID] = 0x4;
  linkaddr_copy(&node_5_address, &linkaddr_node_addr);
  node_5_address.u8[INDEX_NODE_ID] = 0x5;

  uint8_t tx_option = LINK_OPTION_TX;
  uint8_t timeslot54 = 3;
  uint8_t timeslot43 = 6;
  uint8_t timeslot32 = 9;
  uint8_t timeslot21 = 12;


  /* Add a Tx|Rx|Shared slot using broadcast address (i.e. usable for unicast and broadcast).
   * We set the link type to advertising, which is not compliant with 6TiSCH minimal schedule
   * but is required according to 802.15.4e if also used for EB transmission.
   * Timeslot: 0, channel offset: 0. */
  tsch_schedule_add_link(sf_custom,
      LINK_OPTION_RX | LINK_OPTION_TX | LINK_OPTION_SHARED | LINK_OPTION_TIME_KEEPING,
      LINK_TYPE_ADVERTISING, &tsch_broadcast_address,
      0, 0);

  if(linkaddr_node_addr.u8[INDEX_NODE_ID] == 0x05){
    /* Node TX 5 to 4  */ 
    tsch_schedule_add_link(sf_custom,
       tx_option , LINK_TYPE_NORMAL, &node_4_address, timeslot54, 0);
  }

  if(linkaddr_node_addr.u8[INDEX_NODE_ID] == 0x04){
    /* Node RX 4 from 5 */ 
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, timeslot54, 0);
    /* Node TX 4 to 3 */
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_3_address, timeslot43, 4);
  }

  if(linkaddr_node_addr.u8[INDEX_NODE_ID] == 0x03){
    /* Node TX 3 to 2 */
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_2_address, timeslot32, 2);
    /* Node RX 3 from 4 */ 
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, timeslot43, 4);
  }

  if(linkaddr_node_addr.u8[INDEX_NODE_ID] == 0x02){
    /* Node RX 2 from 3 */ 
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, timeslot32, 2);
    /* Node TX 2 to 1 */
    tsch_schedule_add_link(sf_custom,
        tx_option, LINK_TYPE_NORMAL, &node_1_address, timeslot21, 2);
  }

  if(linkaddr_node_addr.u8[INDEX_NODE_ID] == 0x01){
    /* Node RX 1 from 2 */ 
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, timeslot21, 2);
  }


  tsch_schedule_print();
}
/*---------------------------------------------------------------------------*/
/* Create a 6TiSCH schedule for concurrent localisation*/
void
// tsch_schedule_create_minimal_loc(void)
tsch_schedule_create_minimal(void)
{
  struct tsch_slotframe *sf_custom;
  int i;
  int channel_offset = 5;
  /* First, empty current schedule */
  tsch_schedule_remove_all_slotframes();
  /* Build 6TiSCH minimal schedule.
   * We pick a slotframe length of TSCH_SCHEDULE_DEFAULT_LENGTH */
  sf_custom = tsch_schedule_add_slotframe(0, TSCH_SCHEDULE_DEFAULT_LENGTH);
  /* Add a single Tx|Rx|Shared slot using broadcast address (i.e. usable for unicast and broadcast).
   * We set the link type to advertising, which is not compliant with 6TiSCH minimal schedule
   * but is required according to 802.15.4e if also used for EB transmission.
   * Timeslot: 0, channel offset: 0. */
  tsch_schedule_add_link(sf_custom,
      LINK_OPTION_RX | LINK_OPTION_TX | LINK_OPTION_SHARED | LINK_OPTION_TIME_KEEPING,
      LINK_TYPE_ADVERTISING, &tsch_broadcast_address,
      0, 0);

  /* initialise real node ADDR */
  for(i = 0; i<4; i++){
    linkaddr_copy(anchors_addr[i], &linkaddr_node_addr);
    (anchors_addr[i]->u8[INDEX_NODE_ID]) = 0xA0 +i+1;
  }

  linkaddr_copy(&sink_1_address, &linkaddr_node_addr);
  sink_1_address.u8[INDEX_NODE_ID] = 0xA5;

  // printf("current node id %02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X\n", 
  //   linkaddr_node_addr.u8[0],
  //   linkaddr_node_addr.u8[1],
  //   linkaddr_node_addr.u8[2],
  //   linkaddr_node_addr.u8[3],
  //   linkaddr_node_addr.u8[4],
  //   linkaddr_node_addr.u8[5],
  //   linkaddr_node_addr.u8[6],
  //   linkaddr_node_addr.u8[INDEX_NODE_ID]
  //   );  
  // printf("anchor1 id %02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X\n", 
  //   anchors_addr[0]->u8[0],
  //   anchors_addr[0]->u8[1],
  //   anchors_addr[0]->u8[2],
  //   anchors_addr[0]->u8[3],
  //   anchors_addr[0]->u8[4],
  //   anchors_addr[0]->u8[5],
  //   anchors_addr[0]->u8[6],
  //   anchors_addr[0]->u8[INDEX_NODE_ID]
  //   );

  linkaddr_copy(&node_1_address, &linkaddr_node_addr);
  node_1_address.u8[INDEX_NODE_ID] = 0x1;
  linkaddr_copy(&node_2_address, &linkaddr_node_addr);
  node_2_address.u8[INDEX_NODE_ID] = 0x2;
  linkaddr_copy(&node_3_address, &linkaddr_node_addr);
  node_3_address.u8[INDEX_NODE_ID] = 0x3;
  linkaddr_copy(&node_4_address, &linkaddr_node_addr);
  node_4_address.u8[INDEX_NODE_ID] = 0x4;
  linkaddr_copy(&node_5_address, &linkaddr_node_addr);
  node_5_address.u8[INDEX_NODE_ID] = 0x5;


static linkaddr_t * nodes_addr[4] = {&node_1_address, &node_2_address, &node_3_address, 
  &node_4_address};

  /* check if we are a mobile node */
  if(linkaddr_node_addr.u8[INDEX_NODE_ID] <= 0x05){
    printf("Schedule localisation for mobiles nodes\n");
    for(i = 0; i<5; i++){
      uint8_t linktype = LINK_TYPE_LOC;
      /* if sink addr */
      if(anchors_addr[(linkaddr_node_addr.u8[INDEX_NODE_ID]-1+i)%5]->u8[INDEX_NODE_ID] == sink_1_address.u8[INDEX_NODE_ID]){
        linktype = LINK_TYPE_NORMAL;

      // tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, linktype, 
      //   anchors_addr[(linkaddr_node_addr.u8[INDEX_NODE_ID]-1+i)%5], channel_offset*(i+1), linkaddr_node_addr.u8[INDEX_NODE_ID]);
      }

      // if(anchors_addr[(linkaddr_node_addr.u8[INDEX_NODE_ID]-1+i)%5]->u8[INDEX_NODE_ID] == anchors_addr[0]->u8[INDEX_NODE_ID]){
      //   tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, linktype, 
      //     anchors_addr[(linkaddr_node_addr.u8[INDEX_NODE_ID]-1+i)%5], channel_offset*(i+1), linkaddr_node_addr.u8[INDEX_NODE_ID]);
      // }
      tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, linktype, 
          anchors_addr[(linkaddr_node_addr.u8[INDEX_NODE_ID]-1+i)%5], channel_offset*(i+1), linkaddr_node_addr.u8[INDEX_NODE_ID]);

    }
  }
  /* check if we are an anchor node */
  if(linkaddr_node_addr.u8[INDEX_NODE_ID] >= anchors_addr[0]->u8[INDEX_NODE_ID] && 
     linkaddr_node_addr.u8[INDEX_NODE_ID] <= anchors_addr[3]->u8[INDEX_NODE_ID]){
    printf("Schedule localisation for anchors nodes\n");
    for(i = 0; i<4; i++){
      int8_t offset = (linkaddr_node_addr.u8[INDEX_NODE_ID]-0XA0)-i;
      if(offset <=0)
        offset = 5+offset;
      tsch_schedule_add_link(sf_custom,
          LINK_OPTION_RX, LINK_TYPE_LOC, nodes_addr[i], 
          channel_offset*offset, 
          i+1);
    }
    /* direct link to the sink */
    tsch_schedule_add_link(sf_custom,
          LINK_OPTION_TX  | LINK_OPTION_SHARED | LINK_OPTION_TIME_KEEPING, LINK_TYPE_NORMAL, anchors_addr[4], 
          channel_offset*6, 0);
  }
  /* check if we are the sink node */
  if(linkaddr_node_addr.u8[INDEX_NODE_ID] == sink_1_address.u8[INDEX_NODE_ID]){
    printf("Schedule localisation for the sink node\n");
    for(i = 0; i<4; i++){
      tsch_schedule_add_link(sf_custom,
          LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, channel_offset*((5-i)%6), i+1);
    }

    tsch_schedule_add_link(sf_custom,
          LINK_OPTION_RX  | LINK_OPTION_SHARED | LINK_OPTION_TIME_KEEPING, LINK_TYPE_NORMAL, &tsch_broadcast_address, 
          channel_offset*6, 0);
  }
  printf("Schedule localisation initialised\n");
  tsch_schedule_print();
}
/*---------------------------------------------------------------------------*/
/* Prints out the current schedule (all slotframes and links) */
void
tsch_schedule_print(void)
{
  if(!tsch_is_locked()) {
    struct tsch_slotframe *sf = list_head(slotframe_list);

    printf("Schedule: slotframe list\n");

    while(sf != NULL) {
      struct tsch_link *l = list_head(sf->links_list);

      printf("[Slotframe] Handle %u, size %u\n", sf->handle, sf->size.val);
      printf("List of links:\n");

      while(l != NULL) {
        printf("[Link] Options %02x, type %u, timeslot %u, channel offset %u, address 0X%02X\n",
               l->link_options, l->link_type, l->timeslot, l->channel_offset, l->addr.u8[INDEX_NODE_ID]);
        l = list_item_next(l);
      }

      sf = list_item_next(sf);
    }

    printf("Schedule: end of slotframe list\n");
  }
}
/*---------------------------------------------------------------------------*/
/* Return the slotframe duration */
rtimer_clock_t tsch_schedule_get_slotframe_duration(void){
 return (rtimer_clock_t) TSCH_SCHEDULE_DEFAULT_LENGTH * US_TO_RTIMERTICKS(TSCH_DEFAULT_TS_TIMESLOT_LENGTH);
}
/*---------------------------------------------------------------------------*/