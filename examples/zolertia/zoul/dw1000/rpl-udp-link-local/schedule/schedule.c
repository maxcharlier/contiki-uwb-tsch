
#include "contiki.h"
#include "net/mac/tsch/tsch.h"
#include "net/mac/tsch/tsch-schedule.h" 

#include "examples/zolertia/zoul/dw1000/rpl-udp-link-local/schedule/schedule.h"



const linkaddr_t node_1_address  = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x01 } };
const linkaddr_t node_2_address  = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x02 } };
const linkaddr_t node_3_address  = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x03 } };
const linkaddr_t node_4_address  = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x04 } };
const linkaddr_t node_5_address  = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x05 } };
const linkaddr_t node_6_address  = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x06 } };
const linkaddr_t node_7_address  = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x07 } };
const linkaddr_t node_8_address  = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x08 } };
const linkaddr_t node_9_address  = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x09 } };
const linkaddr_t node_10_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x0A } };
const linkaddr_t node_11_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x0B } };
const linkaddr_t node_12_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x0C } };
const linkaddr_t node_13_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x0D } };
const linkaddr_t node_15_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x0F } };
const linkaddr_t node_16_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x10 } };
const linkaddr_t node_17_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x11 } };
const linkaddr_t node_14_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x0E } };
const linkaddr_t node_m1_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x12 } };
const linkaddr_t node_m2_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0X13 } };

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
  &node_17_address,
  &node_m1_address,
  &node_m2_address
};
 

/* Ensure that the NODEID macro is correctly passed by the Makefile.
   That's not the case for each target platform. */
#ifndef NODEID
#  error "Node ID not defined. Perhaps you need to tweak target Makefile."
#endif /* NODEID */

void tsch_schedule_create_udp_server(void)
{
  struct tsch_slotframe *sf_custom;

  /* First, empty current schedule */
  tsch_schedule_remove_all_slotframes();

  /* Build schedule.
   * We pick a slotframe length of TSCH_SCHEDULE_DEFAULT_LENGTH */
  sf_custom = tsch_schedule_add_slotframe(0, 100);

  const struct {
    struct tsch_slotframe *slotframe;
    uint8_t                link_options;
    enum link_type         link_type;
    const linkaddr_t    *address;
    uint16_t               timeslot;
    uint16_t               channel_offset;
  } timeslots[] = {
    { sf_custom, LINK_OPTION_TX | LINK_OPTION_RX | LINK_OPTION_SHARED | LINK_OPTION_TIME_KEEPING, LINK_TYPE_ADVERTISING, &tsch_broadcast_address, 0, 0 },
#if NODEID == 0x01
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 2, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 4, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 6, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 8, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 10, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 12, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 14, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 16, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_m2_address, 18, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_m2_address, 20, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 22, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 24, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 26, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 28, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 30, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 32, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 34, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 36, 0 },
#elif NODEID == 0x02
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 2, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 4, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 11, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 13, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 7, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 15, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 9, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 19, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_m2_address, 23, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_m2_address, 25, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 17, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 27, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 21, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 31, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 35, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 37, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 29, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 39, 0 },
#elif NODEID == 0x03
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 6, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 8, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 11, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 13, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 3, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 33, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 38, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 40, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_m2_address, 42, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_m2_address, 44, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 46, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 48, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 50, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 52, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 54, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 56, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 58, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 60, 0 },
#elif NODEID == 0x04
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 10, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 12, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 7, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 15, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 3, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 33, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 5, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 43, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_m2_address, 47, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_m2_address, 49, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 41, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 51, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 45, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 55, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 59, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 61, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 53, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 63, 0 },
#elif NODEID == 0x07
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 14, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 16, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 9, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 19, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 38, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 40, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 5, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 43, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_m2_address, 57, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_m2_address, 62, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 64, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 66, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 68, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 70, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 72, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 74, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 76, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 78, 0 },
#elif NODEID == 0x13
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 18, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 20, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 23, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 25, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 42, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 44, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 47, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 49, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 57, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 62, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 69, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 71, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 65, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 73, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 67, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 77, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 80, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 82, 0 },
#elif NODEID == 0x0D
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 22, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 24, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 17, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 27, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 46, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 48, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 41, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 51, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 64, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 66, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_m2_address, 69, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_m2_address, 71, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 75, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 79, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 81, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 83, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 85, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 87, 0 },
#elif NODEID == 0x0E
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 26, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 28, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 21, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 31, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 50, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 52, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 45, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 55, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 68, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 70, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_m2_address, 65, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_m2_address, 73, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 75, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 79, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 86, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 88, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 90, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 92, 0 },
#elif NODEID == 0x0F
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 30, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 32, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 35, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 37, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 54, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 56, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 59, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 61, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 72, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 74, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_m2_address, 67, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_m2_address, 77, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 81, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 83, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 86, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 88, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 94, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 96, 0 },
#elif NODEID == 0x10
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 34, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 36, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 29, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 39, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 58, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 60, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 53, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 63, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 76, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 78, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_m2_address, 80, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_m2_address, 82, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 85, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 87, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 90, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 92, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 94, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 96, 0 },
#else
#  error "Unhandled NODEID for static schedule."
#endif /* NODEID */
    { 0 }
  }, *l;

  for(l = timeslots ; l->slotframe ; l++)
    tsch_schedule_add_link(l->slotframe, l->link_options, l->link_type, l->address, l->timeslot, l->channel_offset);
}