
#include "contiki.h"
#include "net/mac/tsch/tsch.h"
#include "net/mac/tsch/tsch-schedule.h" 
#include "examples/zolertia/zoul/dw1000/testbed/schedule-thomas/schedule-thomas.h"

const linkaddr_t node_1_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x01 } };
const linkaddr_t node_2_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x02 } };
const linkaddr_t node_3_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x03 } };
const linkaddr_t node_4_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x04 } };
const linkaddr_t node_5_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x05 } };
const linkaddr_t node_6_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x06 } };
const linkaddr_t node_7_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x07 } };
const linkaddr_t node_8_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x08 } };
const linkaddr_t node_9_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x09 } };
const linkaddr_t node_10_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x0A } };
const linkaddr_t node_11_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x0B } };
const linkaddr_t node_12_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x0C } };
const linkaddr_t node_13_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x0D } };
const linkaddr_t node_14_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x0E } };
const linkaddr_t node_15_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x0F } };
const linkaddr_t node_16_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x10 } };


const linkaddr_t node_b1_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x11 } };
const linkaddr_t node_b2_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x12 } };

const linkaddr_t * mac_neighborg_addr[] = { 
  &node_1_address, 
  &node_2_address, 
  &node_3_address, 
  &node_4_address, 
  &node_5_address, 
  &node_6_address, 
  &node_7_address, 
  &node_8_address, 
  &node_9_address, 
  &node_10_address, 
  &node_11_address, 
  &node_12_address, 
  &node_13_address, 
  &node_14_address, 
  &node_15_address, 
  &node_16_address, 
  &node_b1_address,
  &node_b2_address,
};


/* Ensure that the NODEID macro is correctly passed by the Makefile.
   That's not the case for each target platform. */
#ifndef NODEID
#  error "Node ID not defined. Perhaps you need to tweak target Makefile."
#endif /* NODEID */

void tsch_schedule_create_testbed_localization_for_2_mobiles(void)
{
  struct tsch_slotframe *sf_custom;

  /* First, empty current schedule */
  tsch_schedule_remove_all_slotframes();

  /* Build schedule.
   * We pick a slotframe length of TSCH_SCHEDULE_DEFAULT_LENGTH */
  sf_custom = tsch_schedule_add_slotframe(0, 53);

  const struct {
    struct tsch_slotframe *slotframe;
    uint8_t                link_options;
    enum link_type         link_type;
    const linkaddr_t      *address;
    uint16_t               timeslot;
    uint16_t               channel_offset;
  } timeslots[] = {
    { sf_custom, LINK_OPTION_TX | LINK_OPTION_RX | LINK_OPTION_SHARED | LINK_OPTION_TIME_KEEPING, LINK_TYPE_ADVERTISING, &tsch_broadcast_address, 0, 0 },
#if NODEID == 0x01
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 10, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 13, 1 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 16, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 19, 1 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 22, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 28, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 31, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 34, 1 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 37, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 40, 1 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 43, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 49, 0 },
#elif NODEID == 0x02
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_PROP, &node_b2_address, 9, 1 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 13, 1 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 34, 1 },
#elif NODEID == 0x03
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_PROP, &node_b2_address, 6, 1 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 10, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 13, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 16, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 19, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 22, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 25, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 28, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 31, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 34, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 37, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 40, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 43, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 46, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 49, 0 },
#elif NODEID == 0x04
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_PROP, &node_b2_address, 3, 1 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 19, 1 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 40, 1 },
#elif NODEID == 0x05
#elif NODEID == 0x06
#elif NODEID == 0x07
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 10, 1 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 13, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 16, 1 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 19, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 22, 1 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 25, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 31, 1 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 34, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 37, 1 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 40, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 43, 1 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 46, 0 },
#elif NODEID == 0x08
#elif NODEID == 0x09
#elif NODEID == 0x0A
#elif NODEID == 0x0B
#elif NODEID == 0x0C
#elif NODEID == 0x0D
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_PROP, &node_b1_address, 6, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 13, 2 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 34, 2 },
#elif NODEID == 0x0E
#elif NODEID == 0x0F
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_PROP, &node_b1_address, 9, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 19, 2 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 40, 2 },
#elif NODEID == 0x10
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_PROP, &node_b1_address, 3, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 10, 1 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 13, 2 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 16, 1 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 19, 2 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 22, 1 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 31, 1 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 34, 2 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 37, 1 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 40, 2 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 43, 1 },
#elif NODEID == 0x11
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_PROP, &node_16_address, 3, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_PROP, &node_13_address, 6, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_PROP, &node_15_address, 9, 0 },
#elif NODEID == 0x12
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_PROP, &node_4_address, 3, 1 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_PROP, &node_3_address, 6, 1 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_PROP, &node_2_address, 9, 1 },
#else
#  error "Unhandled NODEID for static schedule."
#endif /* NODEID */
    { 0 }
  }, *l;

  for(l = timeslots ; l->slotframe ; l++)
    tsch_schedule_add_link(l->slotframe, l->link_options, l->link_type, l->address, l->timeslot, l->channel_offset);
}