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

#include "dw1000.h"
#include "dw1000-const.h"
#include "dw1000-arch.h"

// #define DEBUG 1


/*---------------------------------------------------------------------------*/
PROCESS(dw1000_dma, "Frame master");

AUTOSTART_PROCESSES(&dw1000_dma);
/*---------------------------------------------------------------------------*/

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while(0)
#endif
/*---------------------------------------------------------------------------*/
static void
fade(unsigned char l)
{
  volatile int i;
  int k, j;
  for(k = 0; k < 800; ++k) {
    j = k > 400 ? 800 - k : k;

    leds_on(l);
    for(i = 0; i < j; ++i) {
      asm("nop");
    }
    leds_off(l);
    for(i = 0; i < 400 - j; ++i) {
      asm("nop");
    }
  }
}

PROCESS_THREAD(dw1000_dma, ev, data)
{
  static struct etimer blinck_timer;
  PROCESS_BEGIN();

  // init_gpio();

  random_init(2018);

#ifndef RADIO_DRIVER_UWB
  /* initialize SPI 1 */
  dw1000_arch_init();
#endif


  printf("Node addr 0x%02X%02X\n", 
                      linkaddr_node_addr.u8[1], 
                      linkaddr_node_addr.u8[0]);
  printf("NodeID %08lX\n", dw_read_reg_32(DW_REG_DEV_ID, DW_LEN_DEV_ID));
  uint32_t node_id = 0XDECA0130;
  if(dw_read_reg_32(DW_REG_DEV_ID, DW_LEN_DEV_ID) != node_id){
    while(1){
      printf("error led\n");
      fade(LEDS_RED);

      fade(LEDS_GREEN);

      fade(LEDS_BLUE);
      
      PROCESS_YIELD();
    } 
  }

  NETSTACK_RADIO.off();

    // dw1000_arch_spi_set_clock_freq(DW_SPI_CLOCK_FREQ_INIT_STATE);
  static uint8_t value = 1;


  etimer_set(&blinck_timer, CLOCK_SECOND/2);
  for(;;) {
    printf("blinck led\n");
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&blinck_timer));
    uint32_t data;  /* active blinking mode */
    data = dw_read_subreg_32(DW_REG_PMSC, DW_SUBREG_PMSC_LEDC, 
                              DW_SUBLEN_PMSC_LEDC);
    data |= (1UL << DW_BLNKEN) & DW_BLNKEN_MASK; /* enable blink mode */
    data |= (value << DW_BLNKNOW) & DW_BLNKNOW_MASK; /* force LEDs to blink 
                                                        once */
    data &= ~DW_BLINK_TIM_MASK; /* set Blink time count value to 0 */
    data |= (0xF << DW_BLINK_TIM) & DW_BLINK_TIM_MASK; /* blink time to 20 ms 
                                                          (default 400 ms) */

    dw_write_subreg(DW_REG_PMSC, DW_SUBREG_PMSC_LEDC, DW_SUBLEN_PMSC_LEDC, 
                    (uint8_t *)&data);
    // value = (value + 1) %15;
    value = (value << 1);
    if(value == 0X10){
      value= 1;
    }
    // PROCESS_YIELD();
    // clock_delay_usec(0XFFFF);
    // clock_delay_usec(0XFFFF);
    // clock_delay_usec(0XFFFF);
    // PROCESS_YIELD();
    // printf("value %d\n", value);
    
    etimer_reset(&blinck_timer);
  }
  printf("end\n");
  PROCESS_END();
}
