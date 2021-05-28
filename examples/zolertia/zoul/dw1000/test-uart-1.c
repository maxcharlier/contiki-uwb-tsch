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
#include "dev/uart.h"

#include "dev/leds.h"

#include "dw1000.h"
#include "dw1000-const.h"
#include "dw1000-arch.h"


#include "dev/gpio.h"
#define DEBUG 1
#define write_byte(b) uart_write_byte(1, b)


/*---------------------------------------------------------------------------*/
PROCESS(test_uart_1, "Uart 1 tester");

AUTOSTART_PROCESSES(&test_uart_1);
/*---------------------------------------------------------------------------*/

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while(0)
#endif
/*---------------------------------------------------------------------------*/


int
uart1_input_byte(unsigned char c)
{
  write_byte(c);
  return 1;
}

PROCESS_THREAD(test_uart_1, ev, data)
{
  PROCESS_BEGIN();

  uart_set_input(1, uart1_input_byte);



  printf("test uart 1 process\n");
  while(1){
      PROCESS_YIELD();
  }

  PROCESS_END();
}
