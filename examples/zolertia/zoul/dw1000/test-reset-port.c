#include "contiki.h"
#include "net/rime/rime.h"
#include "dev/serial-line.h"
#include <stdio.h>
#include <stdlib.h>
#include "lib/random.h"
#include "net/netstack.h"

#include "reg.h"
#include "spi-arch.h"
#include "dev/ioc.h"
#include "dev/sys-ctrl.h"
#include "dev/spi.h"
#include "dev/ssi.h"
#include "dev/gpio.h" 
#include "dev/udma.h"
#include "watchdog.h"

#include "dev/leds.h"



#include "dev/gpio.h"
// #define DEBUG 1


#define DWM1000_RESET_PORT           GPIO_A_NUM
#define DWM1000_RESET_PIN            7
#define DWM1000_RESET_PORT_BASE    GPIO_PORT_TO_BASE(DWM1000_RESET_PORT)
#define DWM1000_RESET_PIN_MASK     GPIO_PIN_MASK(DWM1000_RESET_PIN)

/*---------------------------------------------------------------------------*/
PROCESS(test_reset, "Test reset port");

AUTOSTART_PROCESSES(&test_reset);
/*---------------------------------------------------------------------------*/

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while(0)
#endif
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(test_reset, ev, data)
{
  static struct etimer blinck_timer;
  PROCESS_BEGIN();


  NETSTACK_RADIO.off();

  /* init the RESET port in OUTPUT, LOW */
  GPIO_SET_OUTPUT(DWM1000_RESET_PORT_BASE, DWM1000_RESET_PIN_MASK);
  GPIO_CLR_PIN(DWM1000_RESET_PORT_BASE, DWM1000_RESET_PIN_MASK);

  static uint8_t output_value = 1;

  etimer_set(&blinck_timer, CLOCK_SECOND/4);

  for(;;) {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&blinck_timer));

    if(output_value == 1){
      printf("Clear RESET\n");
      GPIO_CLR_PIN(DWM1000_RESET_PORT_BASE, DWM1000_RESET_PIN_MASK);
      output_value = 0;
    }
    else{
      printf("Set RESET\n");
      GPIO_SET_PIN(DWM1000_RESET_PORT_BASE, DWM1000_RESET_PIN_MASK);
      output_value = 1;
    }
    etimer_reset(&blinck_timer);
  }

  PROCESS_END();
}
