#include "contiki.h"
#include "net/mac/tsch/tsch.h"
#include "net/mac/tsch/tsch-schedule.h" 
#include "examples/zolertia/zoul/dw1000/localisation-mobility/libs/schedule-onecell6A1T.h"


const linkaddr_t node_1_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x01 } };
const linkaddr_t node_2_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x02 } };
const linkaddr_t node_3_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x03 } };
const linkaddr_t node_4_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x04 } };
const linkaddr_t node_6_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x05 } };
const linkaddr_t node_6_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x06 } };
const linkaddr_t node_relay_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x07 } };
const linkaddr_t node_tag_address = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x08 } };

const linkaddr_t * mac_neighborg_addr[] = { 
  &node_1_address, 
  &node_2_address, 
  &node_3_address,
  &node_4_address, 
  &node_5_address, 
  &node_6_address,
  &node_relay_address,
  &node_tag_address
};
 



/* Ensure that the NODEID macro is correctly passed by the Makefile.
   That's not the case for each target platform. */
#ifndef NODEID
#  error "Node ID not defined. Perhaps you need to tweak target Makefile."
#endif /* NODEID */

struct tsch_slotframe *tsch_schedule_create_initial(void)
{
  struct tsch_slotframe *sf_custom;

  /* First, empty current schedule */
  tsch_schedule_remove_all_slotframes();

  /* Build schedule.
   * We pick a slotframe length of TSCH_SCHEDULE_DEFAULT_LENGTH */
  sf_custom = tsch_schedule_add_slotframe(0, 30);

  const struct {
    struct tsch_slotframe *slotframe;
    uint8_t                link_options;
    enum link_type         link_type;
    const linkaddr_t      *address;
    uint16_t               timeslot;
    uint16_t               channel_offset;
  } timeslots[] = {
    { sf_custom, LINK_OPTION_TX | LINK_OPTION_RX | LINK_OPTION_SHARED | LINK_OPTION_TIME_KEEPING, LINK_TYPE_ADVERTISING, &tsch_broadcast_address, 0, 0 },
#if NODEID == 0x01      // RPL ROOT
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_tag_address, 2, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_tag_address, 8, 0 },
#elif NODEID == 0x02
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_tag_address, 4, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_tag_address, 10, 0 },
#elif NODEID == 0x03
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_tag_address, 6, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_tag_address, 12, 0 },
#elif NODEID == 0x04
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_tag_address, 2, 1 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_tag_address, 8, 1 },
#elif NODEID == 0x05
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_tag_address, 4, 1 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_tag_address, 10, 1 },
#elif NODEID == 0x06
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_tag_address, 6, 1 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_tag_address, 12, 1 },
#elif NODEID == 0x07    // REPLAY
#elif NODEID == 0x08    // TAG
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_1_address, 2, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, 4, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_3_address, 6, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, 8, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_2_address, 10, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_3_address, 12, 0 },

    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_4_address, 2, 1 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 4, 1 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_6_address, 6, 1 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_4_address, 8, 1 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 10, 1 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_6_address, 12, 1 },
#else
#  error "Unhandled NODEID for static schedule."
#endif /* NODEID */
    { 0 }
  }, *l;

  for(l = timeslots ; l->slotframe ; l++)
    tsch_schedule_add_link(l->slotframe, l->link_options, l->link_type, l->address, l->timeslot, l->channel_offset);
  
  return sf_custom;
}