
#include "contiki.h"
#include "net/mac/tsch/tsch.h"

extern const linkaddr_t node_1_address;
extern const linkaddr_t node_2_address;
extern const linkaddr_t node_3_address;
extern const linkaddr_t node_4_address;
extern const linkaddr_t node_5_address;
extern const linkaddr_t node_6_address;
extern const linkaddr_t node_7_address;
extern const linkaddr_t node_8_address;
extern const linkaddr_t node_9_address;
extern const linkaddr_t node_10_address;
extern const linkaddr_t node_11_address;
extern const linkaddr_t node_12_address;
extern const linkaddr_t node_13_address;
extern const linkaddr_t node_14_address;
extern const linkaddr_t node_15_address;
extern const linkaddr_t node_16_address;

extern const linkaddr_t node_b1_address; /* virtual mobile node */
extern const linkaddr_t node_b2_address;

extern const linkaddr_t * mac_neighborg_addr[18];


void tsch_schedule_create_testbed_localization_for_2_mobiles(void);