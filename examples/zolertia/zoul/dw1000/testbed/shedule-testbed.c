
#include "contiki.h"
#include "net/mac/tsch/tsch.h"
#include "net/mac/tsch/tsch-schedule.h" 
#include "examples/zolertia/zoul/dw1000/testbed/shedule-testbed.h"

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

// const linkaddr_t node_1_address = { { 0X00, 0x01 } };
// const linkaddr_t node_2_address = { { 0X00, 0x02 } };
// const linkaddr_t node_3_address = { { 0X00, 0x03 } };
// const linkaddr_t node_4_address = { { 0X00, 0x04 } };
// const linkaddr_t node_5_address = { { 0X00, 0x05 } };
// const linkaddr_t node_6_address = { { 0X00, 0x06 } };
// const linkaddr_t node_7_address = { { 0X00, 0x07 } };
// const linkaddr_t node_8_address = { { 0X00, 0x08 } };
// const linkaddr_t node_9_address = { { 0X00, 0x09 } };
// const linkaddr_t node_10_address = { { 0X00, 0x0A } };
// const linkaddr_t node_11_address = { { 0X00, 0x0B } };
// const linkaddr_t node_12_address = { { 0X00, 0x0C } };
// const linkaddr_t node_13_address = { { 0X00, 0x0D } };
// const linkaddr_t node_14_address = { { 0X00, 0x0E } };
// const linkaddr_t node_15_address = { { 0X00, 0x0F } };
// const linkaddr_t node_16_address = { { 0X00, 0x10 } };
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
};


/* Ensure that the NODEID macro is correctly passed by the Makefile.
   That's not the case for each target platform. */
#ifndef NODEID
#  error "Node ID not defined. Perhaps you need to tweak target Makefile."
#endif /* NODEID */

/**
 * TSCH schedule that allow one time slot in each direction for all node to be full mesh */
void tsch_schedule_fullmesh_data(void)
{
  struct tsch_slotframe *sf_custom;

  /* First, empty current schedule */
  tsch_schedule_remove_all_slotframes();

  /* Build schedule.
   * We pick a slotframe length of TSCH_SCHEDULE_DEFAULT_LENGTH */
  sf_custom = tsch_schedule_add_slotframe(0, 301);

  #define INDEX_NODE_ID (sizeof(linkaddr_t)-1)

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
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 4, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 8, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 12, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 16, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 20, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 24, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 28, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 32, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 36, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 40, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 44, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 48, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 52, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 56, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 60, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 64, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 68, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 72, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 76, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 80, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 84, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 88, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 92, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 96, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 100, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 104, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 108, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 112, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 116, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 120, 0 },
#elif NODEID == 0x02
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 4, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 8, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 21, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 25, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 13, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 29, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 17, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 37, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 45, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 49, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 33, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 53, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 41, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 61, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 69, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 73, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 57, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 77, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 65, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 85, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 93, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 97, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 81, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 101, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 89, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 109, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 117, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 121, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 105, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 125, 0 },
#elif NODEID == 0x03
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 12, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 16, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 21, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 25, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 5, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 34, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 42, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 46, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 30, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 54, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 38, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 58, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 66, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 70, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 50, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 78, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 62, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 82, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 90, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 94, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 74, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 102, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 86, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 106, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 113, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 118, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 98, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 126, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 130, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 134, 0 },
#elif NODEID == 0x04
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 20, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 24, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 13, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 29, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 5, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 34, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 9, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 51, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 59, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 63, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 67, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 71, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 47, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 75, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 39, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 43, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 87, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 91, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 55, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 99, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 79, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 107, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 111, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 115, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 83, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 95, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 103, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 131, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 138, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 142, 0 },
#elif NODEID == 0x05
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 28, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 32, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 17, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 37, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 42, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 46, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 9, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 51, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 22, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 110, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 114, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 119, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 123, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 127, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 132, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 136, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 140, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 144, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 148, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 152, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 156, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 160, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 164, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 168, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 172, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 176, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 180, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 184, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 188, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 192, 0 },
#elif NODEID == 0x06
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 36, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 40, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 45, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 49, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 30, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 54, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 59, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 63, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 22, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 110, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 6, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 10, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 14, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 18, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 26, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 122, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 128, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 133, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 137, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 141, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 145, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 149, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 153, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 157, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 161, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 165, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 169, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 173, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 177, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 181, 0 },
#elif NODEID == 0x07
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 44, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 48, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 33, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 53, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 38, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 58, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 67, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 71, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 114, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 119, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 6, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 10, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 23, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 27, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 15, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 19, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 124, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 150, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 129, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 158, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 135, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 139, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 143, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 174, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 154, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 182, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 162, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 189, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 166, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 170, 0 },
#elif NODEID == 0x08
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 52, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 56, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 41, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 61, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 66, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 70, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 47, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 75, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 123, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 127, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 14, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 18, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 23, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 27, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 7, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 31, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 35, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 155, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 163, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 167, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 171, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 175, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 147, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 179, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 186, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 190, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 151, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 194, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 159, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 198, 0 },
#elif NODEID == 0x09
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 60, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 64, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 69, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 73, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 50, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 78, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 39, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 43, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 132, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 136, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 26, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 122, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 15, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 19, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 7, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 31, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 11, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 178, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 183, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 187, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 191, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 195, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 199, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 203, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 146, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 207, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 211, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 215, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 219, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 223, 0 },
#elif NODEID == 0x0A
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 68, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 72, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 57, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 77, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 62, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 82, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 87, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 91, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 140, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 144, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 128, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 133, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 124, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 150, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 35, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 155, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 11, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 178, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 193, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 197, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 185, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 201, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 208, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 212, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 216, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 220, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 224, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 228, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 232, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 236, 0 },
#elif NODEID == 0x0B
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 76, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 80, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 65, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 85, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 90, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 94, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 55, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 99, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 148, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 152, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 137, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 141, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 129, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 158, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 163, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 167, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 183, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 187, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 193, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 197, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 205, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 209, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 217, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 221, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 225, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 229, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 233, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 237, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 213, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 241, 0 },
#elif NODEID == 0x0C
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 84, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 88, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 93, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 97, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 74, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 102, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 79, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 107, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 156, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 160, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 145, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 149, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 135, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 139, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 171, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 175, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 191, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 195, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 185, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 201, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 205, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 209, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 226, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 230, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 234, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 238, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 242, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 246, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 250, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 254, 0 },
#elif NODEID == 0x0D
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 92, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 96, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 81, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 101, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 86, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 106, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 111, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 115, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 164, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 168, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 153, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 157, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 143, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 174, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 147, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 179, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 199, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 203, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 208, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 212, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 217, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 221, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 226, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 230, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 243, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 247, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 251, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 255, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 259, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 263, 0 },
#elif NODEID == 0x0E
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 100, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 104, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 89, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 109, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 113, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 118, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 83, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 95, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 172, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 176, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 161, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 165, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 154, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 182, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 186, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 190, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 146, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 207, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 216, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 220, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 225, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 229, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 234, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 238, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 243, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 247, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 200, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 260, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 267, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 271, 0 },
#elif NODEID == 0x0F
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 108, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 112, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 117, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 121, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 98, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 126, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 103, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 131, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 180, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 184, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 169, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 173, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 162, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 189, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 151, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 194, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 211, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 215, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 224, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 228, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 233, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 237, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 242, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 246, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 251, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 255, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 200, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 260, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 204, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 275, 0 },
#elif NODEID == 0x10
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 116, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 120, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 105, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 125, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 130, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 134, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 138, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 142, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 188, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 192, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 177, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 181, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 166, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 170, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 159, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 198, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 219, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 223, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 232, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 236, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 213, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 241, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 250, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 254, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 259, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 263, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 267, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 271, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 204, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 275, 0 },
#else
#  error "Unhandled NODEID for static schedule."
#endif /* NODEID */
    { 0 }
  }, *l;

  for(l = timeslots ; l->slotframe ; l++)
    tsch_schedule_add_link(l->slotframe, l->link_options, l->link_type, l->address, l->timeslot, l->channel_offset);
}

void tsch_schedule_alt(void)
{
  struct tsch_slotframe *sf_custom;

  /* First, empty current schedule */
  tsch_schedule_remove_all_slotframes();

  /* Build schedule.
   * We pick a slotframe length of TSCH_SCHEDULE_DEFAULT_LENGTH */
  sf_custom = tsch_schedule_add_slotframe(0, TSCH_SCHEDULE_DEFAULT_LENGTH);

  
      static linkaddr_t node_1_address;
      linkaddr_copy(&node_1_address, &linkaddr_node_addr);
      node_1_address.u8[INDEX_NODE_ID] = 0x01;
      static linkaddr_t node_2_address;
      linkaddr_copy(&node_2_address, &linkaddr_node_addr);
      node_2_address.u8[INDEX_NODE_ID] = 0x02;
      static linkaddr_t node_3_address;
      linkaddr_copy(&node_3_address, &linkaddr_node_addr);
      node_3_address.u8[INDEX_NODE_ID] = 0x03;
      static linkaddr_t node_4_address;
      linkaddr_copy(&node_4_address, &linkaddr_node_addr);
      node_4_address.u8[INDEX_NODE_ID] = 0x04;
      static linkaddr_t node_5_address;
      linkaddr_copy(&node_5_address, &linkaddr_node_addr);
      node_5_address.u8[INDEX_NODE_ID] = 0x05;
      static linkaddr_t node_6_address;
      linkaddr_copy(&node_6_address, &linkaddr_node_addr);
      node_6_address.u8[INDEX_NODE_ID] = 0x06;
      static linkaddr_t node_7_address;
      linkaddr_copy(&node_7_address, &linkaddr_node_addr);
      node_7_address.u8[INDEX_NODE_ID] = 0x07;
      static linkaddr_t node_8_address;
      linkaddr_copy(&node_8_address, &linkaddr_node_addr);
      node_8_address.u8[INDEX_NODE_ID] = 0x08;
      static linkaddr_t node_9_address;
      linkaddr_copy(&node_9_address, &linkaddr_node_addr);
      node_9_address.u8[INDEX_NODE_ID] = 0x09;
      static linkaddr_t node_10_address;
      linkaddr_copy(&node_10_address, &linkaddr_node_addr);
      node_10_address.u8[INDEX_NODE_ID] = 0x0A;
      static linkaddr_t node_11_address;
      linkaddr_copy(&node_11_address, &linkaddr_node_addr);
      node_11_address.u8[INDEX_NODE_ID] = 0x0B;
      static linkaddr_t node_12_address;
      linkaddr_copy(&node_12_address, &linkaddr_node_addr);
      node_12_address.u8[INDEX_NODE_ID] = 0x0C;tsch_schedule_add_link(sf_custom, LINK_OPTION_TX | LINK_OPTION_RX | LINK_OPTION_SHARED | LINK_OPTION_TIME_KEEPING, LINK_TYPE_ADVERTISING, &tsch_broadcast_address, 0, 0);
#if NODEID == 0x01
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 4, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 8, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 12, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 16, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 20, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 24, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 28, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 32, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 36, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 40, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 44, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 48, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 52, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 56, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 60, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 64, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 68, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 72, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 76, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 80, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 84, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 88, 0);
#elif NODEID == 0x02
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 4, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 8, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 21, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 25, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 13, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 29, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 17, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 37, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 45, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 49, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 33, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 53, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 41, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 61, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 69, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 73, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 57, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 77, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 65, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 85, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 92, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 96, 0);
#elif NODEID == 0x03
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 12, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 16, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 21, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 25, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 5, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 34, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 42, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 46, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 30, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 54, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 38, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 58, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 66, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 70, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 50, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 78, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 62, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 82, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 89, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 93, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 74, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 100, 0);
#elif NODEID == 0x04
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 20, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 24, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 13, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 29, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 5, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 34, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 9, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 51, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 59, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 63, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 67, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 71, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 47, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 75, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 39, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 43, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 86, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 90, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 55, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 97, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 79, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 104, 0);
#elif NODEID == 0x05
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 28, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 32, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 17, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 37, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 42, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 46, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 9, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 51, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 22, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 81, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 87, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 91, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 95, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 99, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 103, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 107, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 111, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 115, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 119, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 123, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 127, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 131, 0);
#elif NODEID == 0x06
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 36, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 40, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 45, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 49, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 30, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 54, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 59, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 63, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 22, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 81, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 6, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 10, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 14, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 18, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 26, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 94, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 98, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 102, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 106, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 110, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 114, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 118, 0);
#elif NODEID == 0x07
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 44, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 48, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 33, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 53, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 38, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 58, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 67, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 71, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 87, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 91, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 6, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 10, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 23, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 27, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 15, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 19, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 120, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 124, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 101, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 128, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 108, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 135, 0);
#elif NODEID == 0x08
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 52, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 56, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 41, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 61, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 66, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 70, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 47, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 75, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 95, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 99, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 14, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 18, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 23, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 27, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 7, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 31, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 35, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 129, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 133, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 137, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 122, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 141, 0);
#elif NODEID == 0x09
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 60, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 64, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 69, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 73, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 50, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 78, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 39, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 43, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 103, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 107, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 26, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 94, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 15, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 19, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 7, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 31, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 11, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 134, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 142, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 146, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 150, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 154, 0);
#elif NODEID == 0x0A
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 68, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 72, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 57, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 77, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 62, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 82, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 86, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 90, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 111, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 115, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 98, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 102, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 120, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 124, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 35, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 129, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 11, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 134, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 151, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 155, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 145, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 159, 0);
#elif NODEID == 0x0B
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 76, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 80, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 65, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 85, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 89, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 93, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 55, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 97, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 119, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 123, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 106, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 110, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 101, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 128, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 133, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 137, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 142, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 146, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 151, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 155, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 163, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 167, 0);
#elif NODEID == 0x0C
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 84, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 88, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 92, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 96, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 74, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 100, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 79, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 104, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 127, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 131, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 114, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 118, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 108, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 135, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 122, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 141, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 150, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 154, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 145, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 159, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 163, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 167, 0);
#else
#  error "Unhandled NODEID for static schedule."
#endif /* NODEID */
}


void tsch_schedule_fullmesh_data_2nodes(void)
{
  struct tsch_slotframe *sf_custom;

  /* First, empty current schedule */
  tsch_schedule_remove_all_slotframes();

  /* Build schedule.
   * We pick a slotframe length of TSCH_SCHEDULE_DEFAULT_LENGTH */
  sf_custom = tsch_schedule_add_slotframe(0, 101);

  
      static linkaddr_t node_1_address;
      linkaddr_copy(&node_1_address, &linkaddr_node_addr);
      node_1_address.u8[INDEX_NODE_ID] = 0x01;
      static linkaddr_t node_2_address;
      linkaddr_copy(&node_2_address, &linkaddr_node_addr);
      node_2_address.u8[INDEX_NODE_ID] = 0x02;

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
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 4, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 8, 0 },
#elif NODEID == 0x02
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 4, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 8, 0 },
#endif /* NODEID */
    { 0 }
  }, *l;

  for(l = timeslots ; l->slotframe ; l++)
    tsch_schedule_add_link(l->slotframe, l->link_options, l->link_type, l->address, l->timeslot, l->channel_offset);
}


/* Ensure that the NODEID macro is correctly passed by the Makefile.
   That's not the case for each target platform. */
#ifndef NODEID
#  error "Node ID not defined. Perhaps you need to tweak target Makefile."
#endif /* NODEID */

void tsch_schedule_fullmesh_data_110kbps(void)
{
  struct tsch_slotframe *sf_custom;

  /* First, empty current schedule */
  tsch_schedule_remove_all_slotframes();

  /* Build schedule.
   * We pick a slotframe length of TSCH_SCHEDULE_DEFAULT_LENGTH */
  sf_custom = tsch_schedule_add_slotframe(0, 241);

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
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 1, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 2, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 3, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 4, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 5, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 6, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 7, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 8, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 9, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 10, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 11, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 12, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 13, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 14, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 15, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 16, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 17, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 18, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 19, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 20, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 21, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 22, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 23, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 24, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 25, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 26, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 27, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 28, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 29, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 30, 0 },
#elif NODEID == 0x02
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 1, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 2, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 31, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 32, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 33, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 34, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 35, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 36, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 37, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 38, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 39, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 40, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 41, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 42, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 43, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 44, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 45, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 46, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 47, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 48, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 49, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 50, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 51, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 52, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 53, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 54, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 55, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 56, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 57, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 58, 0 },
#elif NODEID == 0x03
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 3, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 4, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 31, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 32, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 59, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 60, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 61, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 62, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 63, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 64, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 65, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 66, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 67, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 68, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 69, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 70, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 71, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 72, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 73, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 74, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 75, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 76, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 77, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 78, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 79, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 80, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 81, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 82, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 83, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 84, 0 },
#elif NODEID == 0x04
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 5, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 6, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 33, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 34, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 59, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 60, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 85, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 86, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 87, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 88, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 89, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 90, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 91, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 92, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 93, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 94, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 95, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 96, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 97, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 98, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 99, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 100, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 101, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 102, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 103, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 104, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 105, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 106, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 107, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 108, 0 },
#elif NODEID == 0x05
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 7, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 8, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 35, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 36, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 61, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 62, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 85, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 86, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 109, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 110, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 111, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 112, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 113, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 114, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 115, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 116, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 117, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 118, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 119, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 120, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 121, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 122, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 123, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 124, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 125, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 126, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 127, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 128, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 129, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 130, 0 },
#elif NODEID == 0x06
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 9, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 10, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 37, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 38, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 63, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 64, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 87, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 88, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 109, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 110, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 131, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 132, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 133, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 134, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 135, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 136, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 137, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 138, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 139, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 140, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 141, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 142, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 143, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 144, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 145, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 146, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 147, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 148, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 149, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 150, 0 },
#elif NODEID == 0x07
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 11, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 12, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 39, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 40, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 65, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 66, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 89, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 90, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 111, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 112, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 131, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 132, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 151, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 152, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 153, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 154, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 155, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 156, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 157, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 158, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 159, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 160, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 161, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 162, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 163, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 164, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 165, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 166, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 167, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 168, 0 },
#elif NODEID == 0x08
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 13, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 14, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 41, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 42, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 67, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 68, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 91, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 92, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 113, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 114, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 133, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 134, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 151, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 152, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 169, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 170, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 171, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 172, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 173, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 174, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 175, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 176, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 177, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 178, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 179, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 180, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 181, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 182, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 183, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 184, 0 },
#elif NODEID == 0x09
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 15, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 16, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 43, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 44, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 69, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 70, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 93, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 94, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 115, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 116, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 135, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 136, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 153, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 154, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 169, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 170, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 185, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 186, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 187, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 188, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 189, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 190, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 191, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 192, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 193, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 194, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 195, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 196, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 197, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 198, 0 },
#elif NODEID == 0x0A
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 17, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 18, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 45, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 46, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 71, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 72, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 95, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 96, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 117, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 118, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 137, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 138, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 155, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 156, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 171, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 172, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 185, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 186, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 199, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 200, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 201, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 202, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 203, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 204, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 205, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 206, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 207, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 208, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 209, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 210, 0 },
#elif NODEID == 0x0B
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 19, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 20, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 47, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 48, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 73, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 74, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 97, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 98, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 119, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 120, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 139, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 140, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 157, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 158, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 173, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 174, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 187, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 188, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 199, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 200, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 211, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 212, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 213, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 214, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 215, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 216, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 217, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 218, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 219, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 220, 0 },
#elif NODEID == 0x0C
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 21, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 22, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 49, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 50, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 75, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 76, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 99, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 100, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 121, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 122, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 141, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 142, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 159, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 160, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 175, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 176, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 189, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 190, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 201, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 202, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 211, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 212, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 221, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 222, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 223, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 224, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 225, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 226, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 227, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 228, 0 },
#elif NODEID == 0x0D
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 23, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 24, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 51, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 52, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 77, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 78, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 101, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 102, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 123, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 124, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 143, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 144, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 161, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 162, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 177, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 178, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 191, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 192, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 203, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 204, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 213, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 214, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 221, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 222, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 229, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 230, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 231, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 232, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 233, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 234, 0 },
#elif NODEID == 0x0E
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 25, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 26, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 53, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 54, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 79, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 80, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 103, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 104, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 125, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 126, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 145, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 146, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 163, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 164, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 179, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 180, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 193, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 194, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 205, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 206, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 215, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 216, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 223, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 224, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 229, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 230, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 235, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 236, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 237, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 238, 0 },
#elif NODEID == 0x0F
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 27, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 28, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 55, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 56, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 81, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 82, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 105, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 106, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 127, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 128, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 147, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 148, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 165, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 166, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 181, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 182, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 195, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 196, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 207, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 208, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 217, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 218, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 225, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 226, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 231, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 232, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 235, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 236, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_16_address, 239, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_16_address, 240, 0 },
#elif NODEID == 0x10
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 29, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 30, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 57, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 58, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 83, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 84, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 107, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 108, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 129, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 130, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 149, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 150, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 167, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 168, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_8_address, 183, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_8_address, 184, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_9_address, 197, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_9_address, 198, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_10_address, 209, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_10_address, 210, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_11_address, 219, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_11_address, 220, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_12_address, 227, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_12_address, 228, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_13_address, 233, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_13_address, 234, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_14_address, 237, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_14_address, 238, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_15_address, 239, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_15_address, 240, 0 },
#else
#  error "Unhandled NODEID for static schedule."
#endif /* NODEID */
    { 0 }
  }, *l;

  for(l = timeslots ; l->slotframe ; l++)
    tsch_schedule_add_link(l->slotframe, l->link_options, l->link_type, l->address, l->timeslot, l->channel_offset);
}
