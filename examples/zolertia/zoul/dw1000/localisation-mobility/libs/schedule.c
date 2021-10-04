#include "contiki.h"
#include "net/mac/tsch/tsch.h"
#include "net/mac/tsch/tsch-schedule.h" 



const linkaddr_t node_5_address  = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x05 } };
const linkaddr_t node_7_address  = { { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0X00, 0x07 } };

const linkaddr_t * mac_neighborg_addr[] = { 
  &node_5_address, 
  &node_7_address,
};
 



/* Ensure that the NODEID macro is correctly passed by the Makefile.
   That's not the case for each target platform. */
#ifndef NODEID
#  error "Node ID not defined. Perhaps you need to tweak target Makefile."
#endif /* NODEID */

void tsch_schedule_create_initial(void)
{
  struct tsch_slotframe *sf_custom;

  /* First, empty current schedule */
  tsch_schedule_remove_all_slotframes();

  /* Build schedule.
   * We pick a slotframe length of TSCH_SCHEDULE_DEFAULT_LENGTH */
  sf_custom = tsch_schedule_add_slotframe(0, 20);

  const struct {
    struct tsch_slotframe *slotframe;
    uint8_t                link_options;
    enum link_type         link_type;
    const linkaddr_t      *address;
    uint16_t               timeslot;
    uint16_t               channel_offset;
  } timeslots[] = {
    { sf_custom, LINK_OPTION_TX | LINK_OPTION_RX | LINK_OPTION_SHARED | LINK_OPTION_TIME_KEEPING, LINK_TYPE_ADVERTISING, &tsch_broadcast_address, 0, 0 },
#if NODEID == 0x05
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_7_address, 2, 0 },
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_7_address, 4, 0 },
#elif NODEID == 0x07
    { sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_5_address, 2, 0 },
    { sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_5_address, 4, 0 },
#else
#  error "Unhandled NODEID for static schedule."
#endif /* NODEID */
    { 0 }
  }, *l;

  for(l = timeslots ; l->slotframe ; l++)
    tsch_schedule_add_link(l->slotframe, l->link_options, l->link_type, l->address, l->timeslot, l->channel_offset);
}