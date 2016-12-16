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
  static int i = 0;
  static long long unsigned int sum_propragation_time = 0;
  PROCESS_EXITHANDLER(unicast_close(&uc);)

  
  PROCESS_BEGIN();
  
  printf("%s sender %d.%d starting...\r\n",
	 RIME_TYPE,
	 linkaddr_node_addr.u8[0],
	 linkaddr_node_addr.u8[1]);
  
  unicast_open(&uc, RIME_CHANNEL, &uc_cb);


  etimer_set(&timer, CLOCK_SECOND / 100);
  while (1) {
    PROCESS_WAIT_EVENT();
    if (ev == PROCESS_EVENT_TIMER) {

      linkaddr_t addr;
      addr.u8[0]= 10;
      addr.u8[1]= 0;

      /* initialize a ranging request */
      dw1000_driver_ranging_request(); 

      packetbuf_copyfrom("", 0);
      unicast_send(&uc, &addr);


      while(dw1000_driver_is_ranging_request()){
        PROCESS_PAUSE();
      }

      sum_propragation_time += dw1000_driver_get_propagation_time();

      etimer_reset(&timer);
      i++;

      /* we want to make 1000 ranging message to make an antenna calibration */
      if(i == 1000){
        etimer_set(&timer, CLOCK_SECOND * 3600);
        printf("mean propagation time %"PRIu64"\n", 
                      sum_propragation_time / 1000);
      }
    }
    
  }
  
  PROCESS_END();
}
