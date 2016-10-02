#include "contiki.h"
#include "net/rime/rime.h"
#include "net/packetbuf.h"

#include <stdio.h>

PROCESS(frame_receiver_process, "Frame receiver");

AUTOSTART_PROCESSES(&frame_receiver_process);

static unsigned long rx_count= 0;

//#define BROADCAST

#ifdef BROADCAST
# define RIME_CHANNEL 146
# define RIME_TYPE    "broadcast"
static void recv_callback(struct broadcast_conn *c, const linkaddr_t *from);
static struct broadcast_conn bc;
static const struct broadcast_callbacks bc_cb = { recv_callback };
#else
# define RIME_CHANNEL 147
# define RIME_TYPE    "unicast"
static void recv_callback(struct unicast_conn *c, const linkaddr_t *from);
static struct unicast_conn uc;
static const struct unicast_callbacks uc_cb = { recv_callback };
#endif


#ifdef BROADCAST
static void recv_callback(struct broadcast_conn *c, const linkaddr_t *from)
#else
static void recv_callback(struct unicast_conn *c, const linkaddr_t *from)
#endif
{
  int tx_count= -1;
  if (packetbuf_datalen() == sizeof(tx_count))
    packetbuf_copyto(&tx_count);
  rx_count++;
  // printf("%s message received from %d.%d (%i/%i) -- RSSI = %d\r\n",
	 // RIME_TYPE,
	 // from->u8[0], from->u8[1], rx_count, tx_count,
	 // (signed short) packetbuf_attr(PACKETBUF_ATTR_RSSI));
}

PROCESS_THREAD(frame_receiver_process, ev, data)
{
  static struct etimer timer;

#ifdef BROADCAST
  PROCESS_EXITHANDLER(broadcast_close(&bc);)
#else
  PROCESS_EXITHANDLER(unicast_close(&uc);)
#endif
  
  PROCESS_BEGIN();

  printf("BQU:0:%u:%u\r\n", RTIMER_NOW(), RTIMER_ARCH_SECOND);
  printf("receiver %d.%d starting...\r\n", linkaddr_node_addr.u8[0],
	 linkaddr_node_addr.u8[1]);

#ifdef BROADCAST
  broadcast_open(&bc, RIME_CHANNEL, &bc_cb);
#else
  unicast_open(&uc, RIME_CHANNEL, &uc_cb);
#endif

  etimer_set(&timer, CLOCK_SECOND*10);
  
  while (1) {
    PROCESS_WAIT_EVENT();
    if (ev == PROCESS_EVENT_TIMER) {
      printf("Receiver %lu\r\n", rx_count);
      // print_sys_status(dw_read_reg_64( DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS ));
      etimer_reset(&timer);
    }
    
  }
  
  PROCESS_END();
}
