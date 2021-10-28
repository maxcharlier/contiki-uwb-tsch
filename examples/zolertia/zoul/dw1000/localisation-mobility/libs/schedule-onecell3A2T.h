#include "contiki.h"
#include "net/mac/tsch/tsch.h"

extern const linkaddr_t node_1_address;
extern const linkaddr_t node_2_address;
extern const linkaddr_t node_3_address;
extern const linkaddr_t node_tag1_address;
extern const linkaddr_t node_tag2_address;


extern const linkaddr_t * mac_neighborg_addr[5];


struct tsch_slotframe *tsch_schedule_create_initial(void);