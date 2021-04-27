#include "contiki.h"
#include "core/net/ip/uip.h"

typedef enum message_type {
  ALLOCATION_REQUEST,
  ALLOCATION_SLOT,
  ALLOCATION_ACK,
  DEALLOCATION_REQUEST,
  DEALLOCATION_SLOT
} message_type;

typedef struct allocation_request_t {
  message_type message_type : 8;      // 1 byte
  uint8_t signal_power;               // 1 byte
  uip_ipaddr_t mobile_addr;           // 16 bytes
  uip_ipaddr_t anchor_addr;           // 16 bytes
} allocation_request;

typedef struct allocation_slot_t {
  message_type message_type : 8;    // 1 byte
  uint16_t ttl;
  uip_ipaddr_t mobile_addr;
  uip_ipaddr_t anchor_addr;
  uint8_t timeslot;
  uint8_t channel;
} allocation_slot;

typedef struct allocation_ack_t {
  message_type message_type : 8;
  uip_ipaddr_t mobile_addr;
  uip_ipaddr_t anchor_addr;
  uint8_t timeslot;
  uint8_t channel;
} allocation_ack;

typedef struct deallocation_resquest_t {
  message_type message_type : 8;      // 1 byte
  uip_ipaddr_t mobile_addr;           // 16 bytes
  uip_ipaddr_t anchor_addr;           // 16 bytes
} deallocation_resquest;

typedef struct deallocation_slot_t {
  message_type message_type : 8;    // 1 byte
  uip_ipaddr_t mobile_addr;
  uip_ipaddr_t anchor_addr;
  uint8_t timeslot;
  uint8_t channel;
} deallocation_slot;