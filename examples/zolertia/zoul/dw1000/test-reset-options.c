/*
 * Copyright (c) 2018, UMONS University.
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
 *
 */

/**
 * \file
 *         File to get the some values for TSCH with the DW1000 transceiver.
 *
 * \author
 *         Charlier Maximilien  <maximilien.charlier@umons.ac.be>
 */

#include "contiki.h"
#include "net/rime/rime.h"
#include "dev/serial-line.h"
#include <stdio.h>
#include <stdlib.h>
#include "net/netstack.h"
#include "dw1000.h"
#include "dw1000-driver.h"


#include "dev/gpio.h"
#define DWM1000_RESET_PORT           GPIO_A_NUM
#define DWM1000_RESET_PIN            7
#define DWM1000_RESET_PORT_BASE    GPIO_PORT_TO_BASE(DWM1000_RESET_PORT)
#define DWM1000_RESET_PIN_MASK     GPIO_PIN_MASK(DWM1000_RESET_PIN)
// #define DEBUG 1

/*---------------------------------------------------------------------------*/
PROCESS(test_reset, "Test Reset feature");

AUTOSTART_PROCESSES(&test_reset);
/*---------------------------------------------------------------------------*/

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while(0)
#endif

void show_help(void){
  printf("Default state RESET port low \n");
  printf("     1 Clear reset port \n");
  printf("     2 Set reset port (trigger reste)\n");
  printf("     3 request deep sleep\n");
  printf("     4 request wake up\n");
  printf("     5 restore to IDLE\n");
  printf("     6 Get SLEEP state\n");
  printf("     7 Soft reset\n");
  printf("     8 Test SPI communication\n");
}

PROCESS_THREAD(test_reset, ev, data)
{
  PROCESS_BEGIN();
  // NETSTACK_RADIO.init();
  NETSTACK_RADIO.off();

  /* init the RESET port in OUTPUT, LOW */
  GPIO_SET_OUTPUT(DWM1000_RESET_PORT_BASE, DWM1000_RESET_PIN_MASK);
  GPIO_CLR_PIN(DWM1000_RESET_PORT_BASE, DWM1000_RESET_PIN_MASK);

  show_help();


  for(;;) {
    PROCESS_WAIT_EVENT();
    /* master part */
    if(ev == serial_line_event_message) {
      /* we convert the input string data to tow int using strlol see 
      https://www.tutorialspoint.com/c_standard_library/c_function_strtol.htm */
      char * str;
      uint8_t mode = strtol(data, &str, 16);

      PRINTF("Received line: %s\n", (char *)data);
      if(mode == 0x01){ 
        printf("Clear RESET\n");
        GPIO_CLR_PIN(DWM1000_RESET_PORT_BASE, DWM1000_RESET_PIN_MASK);
      }
      else if(mode == 0x2){
        printf("Set RESET\n");
        GPIO_SET_PIN(DWM1000_RESET_PORT_BASE, DWM1000_RESET_PIN_MASK);
      }
      else if(mode == 0x03 ){ 
        printf("Request deep sleep\n");
        uint16_t value = RADIO_SLEEP;
        NETSTACK_RADIO.set_value(RADIO_SLEEP_STATE, (radio_value_t) value);
      }
      else if(mode == 0x04){ 
        printf("Request wake up\n");
        uint16_t value = RADIO_REQUEST_WAKEUP;
        NETSTACK_RADIO.set_value(RADIO_SLEEP_STATE, (radio_value_t) value);
      }
      else if(mode == 0x05){ 
        printf("Restore to IDLE\n");
        uint16_t value = RADIO_IDLE;
        NETSTACK_RADIO.set_value(RADIO_SLEEP_STATE, (radio_value_t) value);
      }
      else if(mode == 0x06){
        printf("Get sleep state\n");
        uint16_t value = 0;
        NETSTACK_RADIO.get_value(RADIO_SLEEP_STATE, (radio_value_t*) &value);
        printf("Radio state : %u\n", (unsigned int) value);
        printf("Avalaible state : RADIO_SLEEP %u, RADIO_REQUEST_WAKEUP %u, RADIO_IDLE %u\n",   
          (unsigned int)RADIO_SLEEP,
          (unsigned int)RADIO_REQUEST_WAKEUP,
          (unsigned int)RADIO_IDLE);
      }
      else if(mode == 0x07){
        printf("Soft reset\n");
        printf("Call NETSTACK_RADIO init() (we start by doing a soft reset\n");
        NETSTACK_RADIO.init();
      }
      else if(mode == 0x08){
        printf("Test DW1000 SPI\n");
        if(dw1000_is_spi_working()) {
          printf("You can now talk with the device!\r\n");
        } else {
          printf("ERROR for talk with the device! %08X\r\n", (unsigned int)
                  dw_read_reg_32(DW_REG_DEV_ID, DW_LEN_DEV_ID));
        }
      }
      else{
        show_help();
      }
    }
  }
  PROCESS_END();
}