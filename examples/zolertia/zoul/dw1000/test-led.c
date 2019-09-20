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

PROCESS_THREAD(dw1000_dma, ev, data)
{
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


  NETSTACK_RADIO.off();

    // dw1000_arch_spi_set_clock_freq(DW_SPI_CLOCK_FREQ_INIT_STATE);
  static uint8_t value = 1;
  for(;;) {
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
    clock_delay_usec(0XFFFF);
    clock_delay_usec(0XFFFF);
    clock_delay_usec(0XFFFF);
    // printf("value %d\n", value);
  }
  printf("end\n");
  PROCESS_END();
}
