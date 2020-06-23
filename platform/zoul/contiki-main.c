/*
 * Copyright (c) 2012, Texas Instruments Incorporated - http://www.ti.com/
 * Copyright (c) 2015, Zolertia - http://www.zolertia.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \addtogroup platform
 * @{
 *
 * \defgroup zoul Zolertia Zoul core module
 *
 * The Zoul comprises the CC2538SF53 and CC1200 in a single module
 * format, which allows a fast reuse of its core components in different
 * formats and form-factors.
 * @{
 *
 * \file
 *   Main module for the Zolertia Zoul core and based platforms
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "dev/leds.h"
#include "dev/sys-ctrl.h"
#include "dev/nvic.h"
#include "dev/uart.h"
#include "dev/watchdog.h"
#include "dev/ioc.h"
#include "dev/button-sensor.h"
#include "dev/serial-line.h"
#include "dev/slip.h"
#include "dev/cc2538-rf.h"
#include "dw1000-driver.h"
#include "dev/udma.h"
#include "spi-arch.h"
#include "dev/crypto.h"
#include "dev/rtcc.h"
#include "usb/usb-serial.h"
#include "lib/random.h"
#include "net/netstack.h"
#include "net/queuebuf.h"
#include "net/ip/tcpip.h"
#include "net/ip/uip.h"
#include "net/mac/frame802154.h"
#include "soc.h"
#include "cpu.h"
#include "reg.h"
#include "ieee-addr.h"
#include "lpm.h"

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#if NETSTACK_CONF_WITH_IPV6
#include "net/ipv6/uip-ds6.h"
#endif /* NETSTACK_CONF_WITH_IPV6 */

#include "net/rime/rime.h"


  #include "sys/clock.h"
  #include "sys/etimer.h"

  #include "dev/uart.h"
  #define DBG_CONF_UART               0
  #define write_byte(b) uart_write_byte(DBG_CONF_UART, b)

  PROCESS(startup_process, "Startup delay process");
  // AUTOSTART_PROCESSES(&startup_process);
/*---------------------------------------------------------------------------*/
 #define STARTUP_CONF_VERBOSE 1
#if STARTUP_CONF_VERBOSE
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#if UART_CONF_ENABLE
#define PUTS(s) puts(s)
#else
#define PUTS(s)
#endif

#define START_DELAY_SECONDS   ((uint16_t) 600)

/*---------------------------------------------------------------------------*/
/** \brief Board specific iniatialisation */
void board_init(void);
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
/*---------------------------------------------------------------------------*/
static void
rtc_init(void)
{
#if RTC_CONF_INIT
#if RTC_CONF_SET_FROM_SYS
  char *next;
  simple_td_map td;
#endif

  /* Configure RTC and return structure with all parameters */
  rtcc_init();

#if RTC_CONF_SET_FROM_SYS
#ifndef DATE
#error Could not retrieve date from system
#endif

  /* Alternatively, for test only, undefine DATE and define it on your own as:
   * #define DATE "07 06 12 15 16 00 00"
   * Also note that if you restart the node at a given time, it will use the
   * already defined DATE, so if you want to update the device date/time you
   * need to reflash the node.
   */

  /* Get the system date in the following format: wd dd mm yy hh mm ss */
  PRINTF("Setting RTC from system date: %s\n", DATE);

  /* Configure the RTC with the current values */
  td.weekdays = (uint8_t)strtol(DATE, &next, 10);
  td.day      = (uint8_t)strtol(next, &next, 10);
  td.months   = (uint8_t)strtol(next, &next, 10);
  td.years    = (uint8_t)strtol(next, &next, 10);
  td.hours    = (uint8_t)strtol(next, &next, 10);
  td.minutes  = (uint8_t)strtol(next, &next, 10);
  td.seconds  = (uint8_t)strtol(next, NULL, 10);

  /* Don't care about the milliseconds... */
  td.miliseconds = 0;

  /* This example relies on 24h mode */
  td.mode = RTCC_24H_MODE;

  /*
   * And to simplify the configuration, it relies on the fact that it will be
   * executed in the present century
   */
  td.century = RTCC_CENTURY_20XX;

  /* Set the time and date */
  if(rtcc_set_time_date(&td) == AB08_ERROR) {
    PRINTF("Failed to set time and date\n");
  }
#endif
#endif
}
/*---------------------------------------------------------------------------*/
static void
set_rf_params(void)
{
  uint16_t short_addr;
  uint8_t ext_addr[8];

  ieee_addr_cpy_to(ext_addr, 8);

  short_addr = ext_addr[7];
  short_addr |= ext_addr[6] << 8;

  /* Populate linkaddr_node_addr. Maintain endianness */
  memcpy(&linkaddr_node_addr, &ext_addr[8 - LINKADDR_SIZE], LINKADDR_SIZE);

#if STARTUP_CONF_VERBOSE
  {
    int i;
    printf("Rime configured with address ");
    for(i = 0; i < LINKADDR_SIZE - 1; i++) {
      printf("%02x:", linkaddr_node_addr.u8[i]);
    }
    printf("%02x\n", linkaddr_node_addr.u8[i]);
  }
#endif

  NETSTACK_RADIO.set_value(RADIO_PARAM_PAN_ID, IEEE802154_PANID);
  NETSTACK_RADIO.set_value(RADIO_PARAM_16BIT_ADDR, short_addr);
  NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, CC2538_RF_CHANNEL);
  NETSTACK_RADIO.set_object(RADIO_PARAM_64BIT_ADDR, ext_addr, 8);
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Main routine for the Zoul-based platforms
 */
int
main(void)
{
  nvic_init();
  ioc_init();
  sys_ctrl_init();
  clock_init();
  lpm_init();
  rtimer_init();
  gpio_init();
  leds_init();
  fade(LEDS_RED);
  process_init();
  watchdog_init();

  /*
   * Character I/O Initialisation.
   * When the UART receives a character it will call serial_line_input_byte to
   * notify the core. The same applies for the USB driver.
   *
   * If slip-arch is also linked in afterwards (e.g. if we are a border router)
   * it will overwrite one of the two peripheral input callbacks. Characters
   * received over the relevant peripheral will be handled by
   * slip_input_byte instead
   */
#if UART_CONF_ENABLE
  uart_init(0);
  uart_init(1);
  uart_set_input(SERIAL_LINE_CONF_UART, serial_line_input_byte);
#endif

#if USB_SERIAL_CONF_ENABLE
  usb_serial_init();
  usb_serial_set_input(serial_line_input_byte);
#endif

  serial_line_init();

  INTERRUPTS_ENABLE();
  fade(LEDS_BLUE);

  PUTS(CONTIKI_VERSION_STRING);
  PUTS(BOARD_STRING);
#if STARTUP_CONF_VERBOSE
  soc_print_info();
#endif

  /* Initialise the H/W RNG engine. */
  random_init(0);

  udma_init();

  process_start(&etimer_process, NULL);
  ctimer_init();

  board_init();

#if CRYPTO_CONF_INIT
  crypto_init();
  crypto_disable();
#endif


  rtc_init();


  process_start(&startup_process, NULL);
  // netstack_init();

  // set_rf_params();
  // PRINTF(" Net: ");
  // PRINTF("%s\n", NETSTACK_NETWORK.name);
  // PRINTF(" MAC: ");
  // PRINTF("%s\n", NETSTACK_MAC.name);
  // PRINTF(" RDC: ");
  // PRINTF("%s\n", NETSTACK_RDC.name);

  // PRINTF(" SPI Clock: ");
  // PRINTF("SPI0: %lu Hz, SPI1: %lu Hz\n", spix_get_clock_freq(0), spix_get_clock_freq(1));

// #if NETSTACK_CONF_WITH_IPV6
//   memcpy(&uip_lladdr.addr, &linkaddr_node_addr, sizeof(uip_lladdr.addr));
//   queuebuf_init();
//   process_start(&tcpip_process, NULL);
// #endif /* NETSTACK_CONF_WITH_IPV6 */

  process_start(&sensors_process, NULL);
#if PLATFORM_HAS_BUTTON
  SENSORS_ACTIVATE(button_sensor);
#endif
  energest_init();
  ENERGEST_ON(ENERGEST_TYPE_CPU);

  // autostart_start(autostart_processes);

  watchdog_start();
  fade(LEDS_GREEN);


  while(1) {
    uint8_t r;
    do {
      /* Reset watchdog and handle polls and events */
      watchdog_periodic();

      r = process_run();
    } while(r > 0);

    /* We have serviced all pending events. Enter a Low-Power mode. */
    lpm_enter();
  }
}
/*---------------------------------------------------------------------------*/

/**
 * @}
 * @}
 */

PROCESS_THREAD(startup_process, ev, data)
{
  static struct etimer start_delay_timer;
  static uint16_t start_delay_iteration;
  PROCESS_BEGIN();

  start_delay_iteration = 0;

  etimer_set(&start_delay_timer, CLOCK_SECOND);

    // printf("test before while\n");
  while(start_delay_iteration < START_DELAY_SECONDS){
    // printf("test after while\n");
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&start_delay_timer));
    // printf("test\n");
    uint32_t value = RTIMER_NOW();
    write_byte((uint8_t) '-');
    write_byte((uint8_t) 'P');
    write_byte((uint8_t) ':');
    write_byte((uint8_t) 'T'); //TSCH
    write_byte((uint8_t) (9+8)); 
    for(int i = 0; i < 4 ; i++){
      write_byte((uint8_t) ((uint8_t*)&value)[i]);    
    }
    write_byte((uint8_t) 7);
    write_byte((uint8_t) 'D');
    write_byte((uint8_t) 'e');
    write_byte((uint8_t) 'l');
    write_byte((uint8_t) 'l'); 
    write_byte((uint8_t) 'a'); 
    write_byte((uint8_t) 'y'); 
    write_byte((uint8_t) (start_delay_iteration & 0XFF));  
    write_byte((uint8_t) '\n');

    start_delay_iteration += 1;

    etimer_reset(&start_delay_timer);
  }

  netstack_init();

  set_rf_params();
  PRINTF(" Net: ");
  PRINTF("%s\n", NETSTACK_NETWORK.name);
  PRINTF(" MAC: ");
  PRINTF("%s\n", NETSTACK_MAC.name);
  PRINTF(" RDC: ");
  PRINTF("%s\n", NETSTACK_RDC.name);

  PRINTF(" SPI Clock: ");
  PRINTF("SPI0: %lu Hz, SPI1: %lu Hz\n", spix_get_clock_freq(0), spix_get_clock_freq(1));

#if NETSTACK_CONF_WITH_IPV6
  memcpy(&uip_lladdr.addr, &linkaddr_node_addr, sizeof(uip_lladdr.addr));
  queuebuf_init();
  process_start(&tcpip_process, NULL);
#endif /* NETSTACK_CONF_WITH_IPV6 */

  autostart_start(autostart_processes);

  PROCESS_END();
}
