
#include "contiki.h"
#include "net/mac/tsch/tsch.h"
#include "net/mac/tsch/tsch-schedule.h" 
#include "examples/zolertia/zoul/dw1000/testbed/shedule-testbed.h"



#define INDEX_NODE_ID   (sizeof(linkaddr_t)-1)

/* 802.15.4 broadcast MAC address  */
// static linkaddr_t node_1_address = { { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff } };

static linkaddr_t node_1_address = { { 0xFF, 0xFF } };
      static linkaddr_t node_2_address = { { 0xFF, 0xFF } };
      static linkaddr_t node_3_address = { { 0xFF, 0xFF } };
      static linkaddr_t node_4_address = { { 0xFF, 0xFF } };
      static linkaddr_t node_5_address = { { 0xFF, 0xFF } };
      static linkaddr_t node_6_address = { { 0xFF, 0xFF } };
      static linkaddr_t node_7_address = { { 0xFF, 0xFF } };
      static linkaddr_t node_8_address = { { 0xFF, 0xFF } };
      static linkaddr_t node_9_address = { { 0xFF, 0xFF } };
      static linkaddr_t node_10_address = { { 0xFF, 0xFF } };
      static linkaddr_t node_11_address = { { 0xFF, 0xFF } };
      static linkaddr_t node_12_address = { { 0xFF, 0xFF } };

static linkaddr_t node_13_address = { { 0XFF, 0XFF } };
static linkaddr_t node_14_address = { { 0XFF, 0XFF } };
static linkaddr_t node_15_address = { { 0XFF, 0XFF } };
static linkaddr_t node_16_address = { { 0XFF, 0XFF } };
static linkaddr_t node_A_address = { { 0XFF, 0XFF } };
static linkaddr_t node_B_address = { { 0XFF, 0XFF } };
static linkaddr_t anchor_1_address = { { 0XFF, 0XFF } };
static linkaddr_t anchor_2_address = { { 0XFF, 0XFF } };
static linkaddr_t anchor_3_address = { { 0XFF, 0XFF } };
static linkaddr_t anchor_4_address = { { 0XFF, 0XFF } };
static linkaddr_t anchor_5_address = { { 0XFF, 0XFF } };
static linkaddr_t sink_1_address = { { 0XFF, 0XFF } };

static linkaddr_t * anchors_addr[6] = {&anchor_1_address, &anchor_2_address, &anchor_3_address, 
  &anchor_4_address, &anchor_5_address, &sink_1_address};

  
/*---------------------------------------------------------------------------*/
/* Create a 6TiSCH schedule to test the propagation time feature. */
void
tsch_schedule_create_minimal_test_loc(void)
// tsch_schedule_create_minimal(void)
{
  struct tsch_slotframe *sf_custom;
  uint8_t offset = 15;
  uint8_t nb_loc_slot = 1;
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
  node_1_address.u8[INDEX_NODE_ID] = 0XD0;
  linkaddr_copy(&node_2_address, &linkaddr_node_addr);
  node_2_address.u8[INDEX_NODE_ID] = 0XD1;

  for (int i = 1; i < 1 + nb_loc_slot; i++){
    if(linkaddr_node_addr.u8[INDEX_NODE_ID] == node_1_address.u8[INDEX_NODE_ID]){
      tsch_schedule_add_link(sf_custom,
          LINK_OPTION_TX, LINK_TYPE_LOC, &node_2_address, offset * i, 0);
    printf("localisation schedule1\n");
    }
    if(linkaddr_node_addr.u8[INDEX_NODE_ID] == node_2_address.u8[INDEX_NODE_ID]){
      tsch_schedule_add_link(sf_custom,
          LINK_OPTION_RX, LINK_TYPE_LOC, &node_1_address, offset * i, 0);
    printf("localisation schedule2\n");
    }
  }

  if(linkaddr_node_addr.u8[INDEX_NODE_ID] == node_1_address.u8[INDEX_NODE_ID]){
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_RX, LINK_TYPE_NORMAL, &node_2_address, offset * (1 + nb_loc_slot), 0);
  printf("localisation schedule1\n");
  }
  if(linkaddr_node_addr.u8[INDEX_NODE_ID] == node_2_address.u8[INDEX_NODE_ID]){
    tsch_schedule_add_link(sf_custom,
        LINK_OPTION_TX, LINK_TYPE_NORMAL, &node_1_address, offset * (1 + nb_loc_slot), 0);
  printf("localisation schedule2\n");
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
tsch_schedule_create_minimal_loc(void)
// tsch_schedule_create_minimal(void)
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
  for(i = 0; i<5; i++){
    linkaddr_copy(anchors_addr[i], &linkaddr_node_addr);
    (anchors_addr[i]->u8[INDEX_NODE_ID]) = 0xA0 +i+1;
  }

  linkaddr_copy(&sink_1_address, &linkaddr_node_addr);
  sink_1_address.u8[INDEX_NODE_ID] = 0xBB;

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


static linkaddr_t * nodes_addr[5] = {&node_1_address, &node_2_address, &node_3_address, 
  &node_4_address, &node_5_address};

  /* check if we are a mobile node */
  if(linkaddr_node_addr.u8[INDEX_NODE_ID] <= 0x05){
    printf("Schedule localisation for mobiles nodes\n");
    for(i = 0; i<6; i++){
      uint8_t linktype = LINK_TYPE_LOC;
      /* if sink addr */
      if(anchors_addr[(linkaddr_node_addr.u8[INDEX_NODE_ID]-1+i)%6]->u8[INDEX_NODE_ID] == sink_1_address.u8[INDEX_NODE_ID]){
        linktype = LINK_TYPE_NORMAL;

      // tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, linktype, 
      //   anchors_addr[(linkaddr_node_addr.u8[INDEX_NODE_ID]-1+i)%5], channel_offset*(i+1), linkaddr_node_addr.u8[INDEX_NODE_ID]);
      }

      // if(anchors_addr[(linkaddr_node_addr.u8[INDEX_NODE_ID]-1+i)%5]->u8[INDEX_NODE_ID] == anchors_addr[0]->u8[INDEX_NODE_ID]){
      //   tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, linktype, 
      //     anchors_addr[(linkaddr_node_addr.u8[INDEX_NODE_ID]-1+i)%5], channel_offset*(i+1), linkaddr_node_addr.u8[INDEX_NODE_ID]);
      // }
      tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, linktype, 
          anchors_addr[(linkaddr_node_addr.u8[INDEX_NODE_ID]-1+i)%6], channel_offset*(i+1), linkaddr_node_addr.u8[INDEX_NODE_ID]);

    }
  }
  /* check if we are an anchor node */
  if(linkaddr_node_addr.u8[INDEX_NODE_ID] >= anchors_addr[0]->u8[INDEX_NODE_ID] && 
     linkaddr_node_addr.u8[INDEX_NODE_ID] <= anchors_addr[4]->u8[INDEX_NODE_ID]){
    printf("Schedule localisation for anchors nodes\n");

    for(i = 0; i<5; i++){
      int8_t offset = (linkaddr_node_addr.u8[INDEX_NODE_ID]-0XA0)-i;
      if(offset <=0)
        offset = 6+offset;
      tsch_schedule_add_link(sf_custom,
          LINK_OPTION_RX, LINK_TYPE_LOC, nodes_addr[i], 
          channel_offset*offset, 
          i+1);
    }
    /* direct link to the sink */
    tsch_schedule_add_link(sf_custom,
          LINK_OPTION_TX  | LINK_OPTION_SHARED | LINK_OPTION_TIME_KEEPING, LINK_TYPE_NORMAL, anchors_addr[5], 
          channel_offset*7, 0);
  }
  /* check if we are the sink node */
  if(linkaddr_node_addr.u8[INDEX_NODE_ID] == sink_1_address.u8[INDEX_NODE_ID]){
    printf("Schedule localisation for the sink node\n");
    for(i = 0; i<5; i++){
      tsch_schedule_add_link(sf_custom,
          LINK_OPTION_RX, LINK_TYPE_NORMAL, &tsch_broadcast_address, channel_offset*((6-i)%7), i+1);
    }

    tsch_schedule_add_link(sf_custom,
          LINK_OPTION_RX  | LINK_OPTION_SHARED | LINK_OPTION_TIME_KEEPING, LINK_TYPE_NORMAL, &tsch_broadcast_address, 
          channel_offset*7, 0);
  }
  printf("Schedule localisation initialised\n");
  tsch_schedule_print();
}
/*---------------------------------------------------------------------------*/
/* Create a 6TiSCH schedule for 2D demo*/
void
tsch_schedule_create_demo_printemps(void)
// tsch_schedule_create_minimal(void)
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

  linkaddr_copy(&sink_1_address, &linkaddr_node_addr);
  sink_1_address.u8[INDEX_NODE_ID] = 0xD0;

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


static linkaddr_t * nodes_addr[5] = {&node_1_address, &node_2_address, &node_3_address, 
  &node_4_address, &node_5_address};

  /* check if we are a mobile node */
  if(linkaddr_node_addr.u8[INDEX_NODE_ID] <= 0x05){
    i = linkaddr_node_addr.u8[INDEX_NODE_ID];
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_LOC, 
        &sink_1_address, channel_offset*i, 0);
  }
  /* check if we are the sink node */
  if(linkaddr_node_addr.u8[INDEX_NODE_ID] == sink_1_address.u8[INDEX_NODE_ID]){
    printf("Schedule localisation for the sink node\n");
    for(i = 0; i<5; i++){
      tsch_schedule_add_link(sf_custom,
          LINK_OPTION_TX, LINK_TYPE_LOC, nodes_addr[i], channel_offset*(i+1), 0);
    }
  }
  printf("Schedule localisation initialised\n");
  tsch_schedule_print();
}
/*---------------------------------------------------------------------------*/
/* Create a 6TiSCH schedule for proptime demo*/
void
tsch_schedule_create_demo_prop(void)
// tsch_schedule_create_minimal(void)
{
  struct tsch_slotframe *sf_custom;
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

  linkaddr_copy(&sink_1_address, &linkaddr_node_addr);
  sink_1_address.u8[INDEX_NODE_ID] = 0xD0;

  linkaddr_copy(&node_1_address, &linkaddr_node_addr);
  node_1_address.u8[INDEX_NODE_ID] = 0x1;


  /* check if we are a mobile node */
  if(linkaddr_node_addr.u8[INDEX_NODE_ID] == 0X01){
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_LOC, 
        &sink_1_address, channel_offset, 0);
  }
  /* check if we are the sink node */
  if(linkaddr_node_addr.u8[INDEX_NODE_ID] == sink_1_address.u8[INDEX_NODE_ID]){
    printf("Schedule localisation for the sink node\n");
    tsch_schedule_add_link(sf_custom,
          LINK_OPTION_TX, LINK_TYPE_LOC, &node_1_address, channel_offset, 0);
  }
  printf("Schedule localisation initialised\n");
  tsch_schedule_print();
}