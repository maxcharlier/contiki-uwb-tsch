#include "contiki.h"
#include "net/mac/tsch/tsch.h"


extern const linkaddr_t node_5_address;
extern const linkaddr_t node_7_address;

struct tsch_slotframe *tsch_schedule_create_initial(void);