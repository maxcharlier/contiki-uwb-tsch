#include "contiki.h"
#include "net/rime/rime.h"
#include <stdio.h>
#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#include "dw1000-driver.h"


# define RIME_CHANNEL 147
# define RIME_TYPE    "unicast"
static struct unicast_conn uc;
static const struct unicast_callbacks uc_cb = { };


PROCESS(frame_sender_process, "Frame anchor");

AUTOSTART_PROCESSES(&frame_sender_process);

PROCESS_THREAD(frame_sender_process, ev, data)
{
  static struct etimer timer;

  PROCESS_EXITHANDLER(unicast_close(&uc);)

  
  PROCESS_BEGIN();
  
  printf("%s sender %d.%d starting...\r\n",
	 RIME_TYPE,
	 linkaddr_node_addr.u8[0],
	 linkaddr_node_addr.u8[1]);
  
  unicast_open(&uc, RIME_CHANNEL, &uc_cb);


  etimer_set(&timer, CLOCK_SECOND * 10);
  /* timer_set(&timer_count, CLOCK_SECOND *10); */ 
  while (1) {
    PROCESS_WAIT_EVENT();
    if (ev == PROCESS_EVENT_TIMER) {

      
#ifdef BROADCAST

      broadcast_send(&bc);
#else
      linkaddr_t addr;
      addr.u8[0]= 10;
      addr.u8[1]= 0;
      /* rtimer_clock_t t1 = RTIMER_NOW(); */

      printf("start\n");

      dw1000_driver_ranging_request(); 

      packetbuf_copyfrom("", 0);

      /* printf("sending %s message %d to %d.%d\r\n",
	RIME_TYPE, tx_count, addr.u8[0], addr.u8[1]); */
      /* packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 1); */
      unicast_send(&uc, &addr);


      while(dw1000_driver_is_ranging_request()){
        PROCESS_PAUSE();
      }
      /* rtimer_clock_t t2 = RTIMER_NOW();

      printf("dw send time %u \r\n", (((t2 - t1)*1000000)/RTIMER_SECOND)); */
      // printf("rx timestamp %"PRIu64"\n", 
      //   (long long unsigned int) dw_get_rx_timestamp());
      // printf("tx timestamp %"PRIu64"\n", 
      //   (long long unsigned int) dw_get_tx_timestamp());

      printf("Propagation time %d\n", 
          (int) dw1000_driver_get_propagation_time());
      printf("reply time %"PRIu64"\n", 
          (long long unsigned int) dw1000_driver_get_reply_time());

#endif
      etimer_reset(&timer);
    }
    
  }
  
  PROCESS_END();
}
