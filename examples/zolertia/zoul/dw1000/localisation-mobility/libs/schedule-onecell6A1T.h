#include "contiki.h"
#include "net/mac/tsch/tsch.h"

extern const linkaddr_t node_1_address;
extern const linkaddr_t node_2_address;
extern const linkaddr_t node_3_address;
extern const linkaddr_t node_4_address;
extern const linkaddr_t node_5_address;
extern const linkaddr_t node_6_address;
extern const linkaddr_t node_relay_address;
extern const linkaddr_t node_tag_address;


extern const linkaddr_t * mac_neighborg_addr[8];


struct tsch_slotframe *tsch_schedule_create_initial(void);