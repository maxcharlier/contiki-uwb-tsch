#include "contiki.h"
#include "net/rime/rime.h"
#include "dev/serial-line.h"
#include <stdio.h>
#include <stdlib.h>
#include "net/netstack.h"

#include "reg.h"
#include "spi-arch.h"
#include "dev/ioc.h"
#include "dev/sys-ctrl.h"
#include "dev/spi.h"
#include "dev/ssi.h"
#include "dev/gpio.h" 

// #include "dw1000-arch.h"

// #define DEBUG 1

/*---------------------------------------------------------------------------*/
PROCESS(dw1000_time, "Frame master");

AUTOSTART_PROCESSES(&dw1000_time);
/*---------------------------------------------------------------------------*/

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while(0)
#endif

#define TIME_PORT     GPIO_A_NUM
#define TIME_PIN      2

void init_gpio(void);
void gpio_up(void);
void gpio_down(void);


PROCESS_THREAD(dw1000_time, ev, data)
{
  PROCESS_BEGIN();

  // init_gpio();
  spix_cs_init(TIME_PORT, TIME_PIN);

  printf("Node addr 0x%02X%02X\n", 
                      linkaddr_node_addr.u8[1], 
                      linkaddr_node_addr.u8[0]);
  printf("     0 modify the channel value:    0 [channel value]\n");
  printf("     1 Set the radio to on\n");
  printf("     2 Set the radio to off\n");
  printf("     3 get the sfd time\n");


  for(;;) {
    PROCESS_YIELD();
    if(ev == serial_line_event_message) {
      /* we convert the input string data to tow int using strlol see 
      https://www.tutorialspoint.com/c_standard_library/c_function_strtol.htm */
      char * str;
      uint16_t mode = strtol(data, &str, 16);

      printf("Received line: %s\n", (char *)data);
      if(mode == 0x0 ){ 
        uint16_t channel = 0;
        NETSTACK_RADIO.get_value(RADIO_PARAM_CHANNEL, (radio_value_t*) &channel);
        printf("previous channel: 0x%.4X %u\n", channel, (unsigned int) channel);

        channel = strtol(str, &str, 16);
        printf("new channel: 0x%.4X %u\n", channel, (unsigned int) channel);

        gpio_down();
        NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, (radio_value_t) channel);
        gpio_up();
      }
      else if(mode == 0x01){ 
        printf("radio on\n");
        
        gpio_down();
        NETSTACK_RADIO.on();
        gpio_up();
      }
      else if(mode == 0x02){ 
        printf("radio off\n");
        
        gpio_down();
        NETSTACK_RADIO.off();
        gpio_up();
      }
      else if(mode == 0x03){ 
        printf("radio getlast sfd\n");
        rtimer_clock_t last_timestamp;
        gpio_down();
        NETSTACK_RADIO.get_object(RADIO_PARAM_LAST_PACKET_TIMESTAMP, &last_timestamp, sizeof(rtimer_clock_t));
        gpio_up();
        printf("last_timestamp %lu\n", last_timestamp);
      }
      else if(mode == 0x04){ 
        uint16_t delay = strtol(str, &str, 16);
        printf("delay: 0x%.4X %u\n", delay, (unsigned int) delay);

        gpio_down();
        // dw1000_us_delay(delay);
        gpio_up();
      }
    }
  }
  printf("end\n");
  PROCESS_END();
}


void 
init_gpio(void)
{
  GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(TIME_PORT),
                        GPIO_PIN_MASK(TIME_PORT));
  ioc_set_over(TIME_PORT, TIME_PIN, IOC_OVERRIDE_DIS);
  GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(TIME_PORT), GPIO_PIN_MASK(TIME_PIN));
  GPIO_CLR_PIN(GPIO_PORT_TO_BASE(TIME_PORT), GPIO_PIN_MASK(TIME_PIN));
}

void
gpio_up(void)
{
  GPIO_SET_PIN(GPIO_PORT_TO_BASE(TIME_PORT), GPIO_PIN_MASK(TIME_PIN));
}
void
gpio_down(void)
{
  GPIO_CLR_PIN(GPIO_PORT_TO_BASE(TIME_PORT), GPIO_PIN_MASK(TIME_PIN));
}