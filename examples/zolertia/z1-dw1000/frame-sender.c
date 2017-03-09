#include "contiki.h"
#include "net/rime/rime.h"
#include <stdio.h>

// #define BROADCAST

#ifdef BROADCAST
# define RIME_CHANNEL 146
# define RIME_TYPE    "broadcast"
static struct broadcast_conn bc;
static const struct broadcast_callbacks bc_cb = {  };
#else
# define RIME_CHANNEL 147
# define RIME_TYPE    "unicast"
static struct unicast_conn uc;
static const struct unicast_callbacks uc_cb = { };
#endif


PROCESS(frame_sender_process, "Frame sender");

AUTOSTART_PROCESSES(&frame_sender_process);

PROCESS_THREAD(frame_sender_process, ev, data)
{
  static struct etimer timer;
  // static struct timer timer_count;
  static unsigned long tx_count= 0;

  
#ifdef BROADCAST
  PROCESS_EXITHANDLER(broadcast_close(&bc);)
#else
  PROCESS_EXITHANDLER(unicast_close(&uc);)
#endif
  
  PROCESS_BEGIN();
  
  printf("%s sender %d.%d starting...\r\n",
	 RIME_TYPE,
	 linkaddr_node_addr.u8[0],
	 linkaddr_node_addr.u8[1]);
  
#ifdef BROADCAST
  broadcast_open(&bc, RIME_CHANNEL, &bc_cb);
#else
  unicast_open(&uc, RIME_CHANNEL, &uc_cb);
#endif

  etimer_set(&timer, CLOCK_SECOND * 8);
  // timer_set(&timer_count, CLOCK_SECOND *10);
  static uint16_t i =23;
  while (1) {
    PROCESS_WAIT_EVENT();
    if (ev == PROCESS_EVENT_TIMER) {
      if(tx_count % 100 == 0 && tx_count > 0){
        i++;
        // printf("change size: %lu, %u\n", tx_count, i);
        if(i > 248){
          etimer_set(&timer, CLOCK_SECOND * 3600 );
          PROCESS_WAIT_EVENT();
          i = 0;
        }
      }

      tx_count++;
      // packetbuf_copyfrom(&tx_count, sizeof(tx_count));
      
#ifdef BROADCAST
      // printf("sending %s message %d\r\n",
	     //         RIME_TYPE tx_count);

      broadcast_send(&bc);
#else
      linkaddr_t addr;
      addr.u8[0]= 7;
      addr.u8[1]= 0;
      // rtimer_clock_t t1 = RTIMER_NOW();
      i ++; 
      printf("i %d\n", i);
      packetbuf_copyfrom("Hello World Hello World Hello World Hello World Hello World Hello World Hello World Hello World Hello World HELHello World Hello World Hello World Hello World Hello World Hello World Hello World Hello World Hello World HELHello World Hello World Hello World Hello World Hello World Hello World Hello World Hello World Hello World HELHello World Hello World Hello World Hello World Hello World Hello World Hello World Hello World Hello World HELHello World Hello World Hello World Hello World Hello World Hello World Hello World Hello World Hello World HELHello World Hello World Hello World Hello World Hello World Hello Wo", i); //max110 //max248
      /*printf("sending %s message %d to %d.%d\r\n",
	RIME_TYPE, tx_count, addr.u8[0], addr.u8[1]);*/
      // packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 1);
      unicast_send(&uc, &addr);
      PROCESS_PAUSE();
      // rtimer_clock_t t2 = RTIMER_NOW();

      // printf("dw send time %u \r\n", (((t2 - t1)*1000000)/RTIMER_SECOND));

#endif
    //   if(timer_expired(&timer_count)){
    //     printf("Send: %lu\n", tx_count);
    //     timer_reset(&timer_count);
    //   }

      etimer_reset(&timer);
    }
    
  }
  
  PROCESS_END();
}
