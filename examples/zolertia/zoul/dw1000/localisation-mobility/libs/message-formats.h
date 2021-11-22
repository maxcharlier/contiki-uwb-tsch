#include "contiki.h"
#include "core/net/ip/uip.h"

typedef enum message_type {
  ALLOCATION_REQUEST,
  ALLOCATION_SLOT,
  ALLOCATION_ACK,
  DEALLOCATION_REQUEST,
  DEALLOCATION_SLOT,
  DEALLOCATION_ACK,
  CLEAR_SLOTFRAME,
  CLEAR_ACK,
  PROPAGATION_TIME,
  DEBUGGING = 255
} message_type;

typedef struct allocation_request_t {
  message_type message_type : 8;      // 1 byte
  uint8_t signal_power;               // 1 byte
  uint16_t padding1;                  // 2 bytes of padding
  uip_ipaddr_t mobile_addr;           // 16 bytes
  uip_ipaddr_t anchor_addr;           // 16 bytes
} allocation_request;

typedef struct allocation_slot_t {
  message_type message_type : 8;      // 1 byte
  uint8_t ttl;                        // 1 bytes
  uint8_t timeslot;                   // 1 byte
  uint8_t channel;                    // 1 byte
  uip_ipaddr_t mobile_addr;           // 16 bytes
  uip_ipaddr_t anchor_addr;           // 16 bytes

} allocation_slot;

typedef struct allocation_ack_t {
  message_type message_type : 8;
  uint8_t padding1;
  uint8_t timeslot;
  uint8_t channel;
  uip_ipaddr_t mobile_addr;
  uip_ipaddr_t anchor_addr;
} allocation_ack;

typedef struct deallocation_resquest_t {
  message_type message_type : 8;      // 1 byte
  uint8_t padding1;
  uint8_t timeslot;                   // needed ?
  uint8_t channel;                    // needed ?
  uip_ipaddr_t mobile_addr;           // 16 bytes
  uip_ipaddr_t anchor_addr;           // 16 bytes
} deallocation_resquest;

typedef struct deallocation_slot_t {
  message_type message_type : 8;    // 1 byte
  uint8_t padding1;                 // 1 byte
  uint8_t timeslot;                 // 1 byte
  uint8_t channel;                  // 1 byte
  uip_ipaddr_t mobile_addr;         // 16 bytes
  uip_ipaddr_t anchor_addr;         // 16 bytes

} deallocation_slot;

typedef struct deallocation_ack_t {
  message_type message_type : 8;
  uint8_t padding1;
  uint8_t timeslot;
  uint8_t channel;
  uip_ipaddr_t mobile_addr;
  uip_ipaddr_t anchor_addr;
} deallocation_ack;

typedef struct clear_slotframe_t {
  message_type message_type : 8;
} clear_slotframe;

typedef struct clear_ack_t {
  message_type message_type : 8;
  uip_ipaddr_t from_addr;
} clear_ack;

typedef struct propagation_time_t {
  message_type message_type: 8;
  uint8_t padding1;
  uint8_t padding2;
  uint8_t tsch_channel;
  uip_ipaddr_t mobile_addr;
  uip_ipaddr_t anchor_addr;
  int32_t prop_time;
  struct tsch_asn_t asn;  /* ASN of the slot used for the propagation time measurement */
} propagation_time;

typedef struct debug_packet_t {
  message_type message_type : 8;
  char debug_message[30];
} debug_packet;