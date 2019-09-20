#include "contiki.h"
#include "net/rime/rime.h"
#include "dev/serial-line.h"
#include <stdio.h>
#include <stdlib.h>
#include "random.h"
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

#include "dw1000.h"
#include "dw1000-const.h"
#include "dw1000-arch.h"

// #define DEBUG 1
/**
 /!\ You need to ennable the led in the DW1000 driver file with the MACRO 
 DEGUB_LED
 */

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

PROCESS_THREAD(dw1000_dma, ev, data)
{
  static struct etimer et;
  PROCESS_BEGIN();

  random_init(2018);

#ifndef RADIO_DRIVER_UWB
  /* initialize SPI 1 */
  dw1000_arch_init();
#endif

  printf("Node addr 0x%02X%02X\n", 
                      linkaddr_node_addr.u8[1], 
                      linkaddr_node_addr.u8[0]);
  printf("NodeID %08lX\n", dw_read_reg_32(DW_REG_DEV_ID, DW_LEN_DEV_ID));


  NETSTACK_RADIO.off();

  uint32_t data;  /* active blinking mode */
    // dw1000_arch_spi_set_clock_freq(DW_SPI_CLOCK_FREQ_INIT_STATE);
  static uint8_t value = 1;
  static int i;
  while(1){
    for(i = 0; i<=4; i++) {
      data = dw_read_subreg_32(DW_REG_PMSC, DW_SUBREG_PMSC_LEDC, 
                                DW_SUBLEN_PMSC_LEDC);

      printf("Data %08lX\n", data);
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
      clock_delay_usec(0XFFFF);

    watchdog_periodic();
      clock_delay_usec(0XFFFF);

    watchdog_periodic();
      clock_delay_usec(0XFFFF);
      // printf("value %d\n", value);
    }
    printf("end loop\n");

    /* request to place the transceiver in SLEEP mode to reduce energy consumption */
    NETSTACK_RADIO.set_value(RADIO_SLEEP_STATE, RADIO_SLEEP);

    /* Delay 2-4 seconds */
    etimer_set(&et, CLOCK_SECOND * 2 + random_rand() % (CLOCK_SECOND * 2));

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    watchdog_periodic();
    NETSTACK_RADIO.set_value(RADIO_SLEEP_STATE, RADIO_REQUEST_WAKEUP);
    etimer_set(&et, 1); // 7 ms
    //printf("CLOCK_SECOND %d\n", CLOCK_SECOND);

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    NETSTACK_RADIO.set_value(RADIO_SLEEP_STATE, RADIO_IDLE);
      printf("NodeID %08lX\n", dw_read_reg_32(DW_REG_DEV_ID, DW_LEN_DEV_ID));

    // dw_enable_gpio_led();
  }

  printf("end\n");
  PROCESS_END();
}
