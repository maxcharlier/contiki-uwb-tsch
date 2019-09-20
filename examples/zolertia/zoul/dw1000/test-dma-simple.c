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
#include "dw1000-arch.h"

// #define DEBUG 1

/* DW_REG_TX_BUFFER | DW_REG_RX_BUFFER | DW_REG_USR_SFD */
#define WRITE_BUFFER        DW_REG_TX_BUFFER 

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

#define TIME_PORT     GPIO_A_NUM
#define TIME_PIN      2

void init_gpio(void);
void gpio_up(void);
void gpio_down(void);


PROCESS_THREAD(dw1000_dma, ev, data)
{
  static uint16_t offset = 0;
  PROCESS_BEGIN();

  init_gpio();

  random_init(2018);

#ifndef RADIO_DRIVER_UWB
  /* initialize SPI 1 */
  dw1000_arch_init();
#endif

  spix_cs_init(TIME_PORT, TIME_PIN);

  printf("Node addr 0x%02X%02X\n", 
                      linkaddr_node_addr.u8[1], 
                      linkaddr_node_addr.u8[0]);

  printf("     [0, n] write n random byte in the RX buffer\n");
  printf("     [1, n] read n byte in the RX buffer\n");

  // dw1000_arch_spi_set_clock_freq(DW_SPI_CLOCK_FREQ_INIT_STATE);

  for(;;) {
    PROCESS_YIELD();
    if(ev == serial_line_event_message) {
      /* we convert the input string data to tow int using strlol see 
      https://www.tutorialspoint.com/c_standard_library/c_function_strtol.htm */
      char * str;
      uint16_t mode = strtol(data, &str, 16);

      printf("Received line: %s\n", (char *)data);

      if(mode == 0x0 ){ 
        uint16_t nb_bytes = strtol(str, &str, 10);
        // nb_bytes--;
        /* generate the random bytes tab */
        uint8_t random_bytes[1025];

        printf("Random tab   %04u : ", nb_bytes);
        for(int i = 0; i < nb_bytes; i++){
          random_bytes[i] = rand();
        }
        for(int i = 0; i < nb_bytes; i++){
          printf("%u ", random_bytes[i]);
          watchdog_periodic();
        }
        printf("\n");

        gpio_down();
        // dw_access_subreg(DW_WRITE, WRITE_BUFFER, offset, nb_bytes, (uint8_t *) &random_bytes[0]);
        dw_write_subreg(WRITE_BUFFER, offset, nb_bytes, (uint8_t *) &random_bytes[0]);
        gpio_up();
      }
      else if(mode == 0x01){ 
        uint16_t nb_bytes = strtol(str, &str, 10);
        // nb_bytes--;
        /* generate the random bytes tab */
        uint8_t random_bytes[1023];

        gpio_down();
        dw_read_subreg(WRITE_BUFFER, offset, nb_bytes, (uint8_t *) &random_bytes[0]);
        // dw_access_subreg(DW_READ, WRITE_BUFFER, offset, nb_bytes, (uint8_t *) &random_bytes[0]);

        gpio_up();

        printf("READ_BUFFER  %04u : ", nb_bytes);
        for(int i = 0; i < nb_bytes; i++){
          printf("%u ", random_bytes[i]);
          watchdog_periodic();
        }
        printf("\n");
      }
      else if(mode == 0x02){ 
        uint16_t nb_bytes = strtol(str, &str, 10);
        // nb_bytes--;
        nb_bytes = MIN(nb_bytes, 512);
        /* generate the random bytes tab */
        uint8_t random_bytes[512];
        uint8_t read_bytes[512];
        uint16_t nb_error = 0;
        uint16_t first_error = 0;

          watchdog_periodic();
        printf("Random tab   %04u : ", nb_bytes);
        for(int i = 0; i < nb_bytes; i++){
          random_bytes[i] = rand();
        }
        for(int i = 0; i < nb_bytes; i++){
          printf("%u ", random_bytes[i]);
          watchdog_periodic();
        }
        printf("\n");

        gpio_down();
        dw_write_subreg(WRITE_BUFFER, offset, nb_bytes, (uint8_t *) &random_bytes[0]);
        // dw_access_subreg(DW_WRITE, WRITE_BUFFER, offset, nb_bytes, (uint8_t *) &random_bytes[0]);

        gpio_up();

        watchdog_periodic();
        gpio_down();
        dw_read_subreg(WRITE_BUFFER, offset, nb_bytes, (uint8_t *) &read_bytes[0]);
        // dw_access_subreg(DW_READ, WRITE_BUFFER, offset, nb_bytes, (uint8_t *) &read_bytes[0]);

        gpio_up();

        printf("READ_BUFFER  %04u : ", nb_bytes);
        for(int i = 0; i < nb_bytes; i++){
          if(read_bytes[i] != random_bytes[i]){
            if (nb_error == 0){
              first_error = i;
            }
            nb_error++;
          }
          printf("%u ", read_bytes[i]);
          watchdog_periodic();
        }

        printf("\n");
        printf("Nb error : %d, first error : %d \n", nb_error, first_error);
      }
      else if(mode == 0x03){ 
        offset = strtol(str, &str, 16);
        printf("Offset  %04X \n ", offset);
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
