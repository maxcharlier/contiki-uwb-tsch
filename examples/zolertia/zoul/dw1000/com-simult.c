/*
 * Copyright (c) 2017, UMONS University.
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
 *         Test simultaneous communication of the DW1000.
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
#include "dw1000-driver.h"
#include "dw1000-util.h"
#include "dw1000.h"
#include "dev/watchdog.h"

#include "dev/gpio.h"
#include "dev/ioc.h"
#include "dev/leds.h"

#define DWM1000_TRIGER_OUT_PORT           GPIO_A_NUM
#define DWM1000_TRIGER_OUT_PIN            7
#define TRIGGER_OUT_CLR() do { \
    GPIO_CLR_PIN(GPIO_PORT_TO_BASE(DWM1000_TRIGER_OUT_PORT), GPIO_PIN_MASK(DWM1000_TRIGER_OUT_PIN)); \
} while(0)
#define TRIGGER_OUT_SET() do { \
    GPIO_SET_PIN(GPIO_PORT_TO_BASE(DWM1000_TRIGER_OUT_PORT), GPIO_PIN_MASK(DWM1000_TRIGER_OUT_PIN)); \
} while(0)
#define TRIGER_INT_PORT             GPIO_A_NUM
#define TRIGER_INT_PIN              6

#define INT_GPIOx_VECTOR         GPIO_A_IRQn

/*---------------------------------------------------------------------------*/
PROCESS(frame_master_process, "Frame master");

PROCESS(interrupt_process, "Frame master");

AUTOSTART_PROCESSES(&frame_master_process);
/*---------------------------------------------------------------------------*/

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while(0)
#endif

#define RIME_CHANNEL 146
#define RIME_TYPE    "unicast"
static void recv_callback(struct unicast_conn *c, const linkaddr_t *from);
static struct unicast_conn uc;
static const struct unicast_callbacks uc_cb = { recv_callback };


static uint16_t dest_addr; /* the destination address*/

static uint16_t message_received = 0; /* number of received messages */
static uint16_t interrupt_detected = 0; /* number of interrupt detected */

uint8_t frame[42];
static uint8_t seq_num = 0;
static uint8_t data[32] = {0, 0x0B, 0x92, 0, 0, 0x02, 0xFF, 0xFF, 0x48, 0X65, 0x6C, 0X6C, 0X6F, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t mode;

/* used when a message is received */
static void recv_callback(struct unicast_conn *c, const linkaddr_t *from)
{
  if(packetbuf_datalen() >= 1){ 
    PRINTF("Node receive message from 0x%02X%02X\n", 
                      from->u8[1], 
                      from->u8[0]);

    message_received +=1;
  }
}

void
com_simult_int_handler(uint8_t port, uint8_t pin)
{
  // dw_enable_gpio_led();
  // printf("interupt\n");
  /* To keep the gpio_register_callback happy */
  interrupt_detected++;
  process_poll(&interrupt_process);
}

PROCESS_THREAD(frame_master_process, ev, data)
{
  PROCESS_EXITHANDLER(unicast_close(&uc);)
  PROCESS_BEGIN();

  printf("Node addr 0x%02X%02X\n", 
                      linkaddr_node_addr.u8[1], 
                      linkaddr_node_addr.u8[0]);
  printf("     1 Configure the channel (0X1 CHANNEL)\n");
  printf("     2 Get the channel (0X2)\n");
  printf("     3 Config the distination (0X3 DEST_ADDR)\n");
  printf("     4 Trigger the GPIO\n");
  printf("     5 Show number of received messages\n");
  printf("     6 Reset number of received message\n");

  unicast_open(&uc, RIME_CHANNEL, &uc_cb);

  process_start(&interrupt_process, NULL);

  /* init output GPIO */
  GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(DWM1000_TRIGER_OUT_PORT), GPIO_PIN_MASK(DWM1000_TRIGER_OUT_PIN));
  GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(DWM1000_TRIGER_OUT_PORT), GPIO_PIN_MASK(DWM1000_TRIGER_OUT_PIN));
  GPIO_CLR_PIN(GPIO_PORT_TO_BASE(DWM1000_TRIGER_OUT_PORT), GPIO_PIN_MASK(DWM1000_TRIGER_OUT_PIN));
  
  /* active input GPIO */
  GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(TRIGER_INT_PORT), GPIO_PIN_MASK(TRIGER_INT_PIN));
  GPIO_SET_INPUT(GPIO_PORT_TO_BASE(TRIGER_INT_PORT), GPIO_PIN_MASK(TRIGER_INT_PIN));
  GPIO_DETECT_EDGE(GPIO_PORT_TO_BASE(TRIGER_INT_PORT), GPIO_PIN_MASK(TRIGER_INT_PIN));
  GPIO_TRIGGER_SINGLE_EDGE(GPIO_PORT_TO_BASE(TRIGER_INT_PORT), GPIO_PIN_MASK(TRIGER_INT_PIN));

  GPIO_DETECT_RISING(GPIO_PORT_TO_BASE(TRIGER_INT_PORT), GPIO_PIN_MASK(TRIGER_INT_PIN));
  // GPIO_DETECT_RISING(GPIO_PORT_TO_BASE(TRIGER_INT_PORT), GPIO_PIN_MASK(TRIGER_INT_PIN));
  GPIO_ENABLE_INTERRUPT(GPIO_PORT_TO_BASE(TRIGER_INT_PORT), GPIO_PIN_MASK(TRIGER_INT_PIN));
  ioc_set_over(TRIGER_INT_PORT, TRIGER_INT_PIN, IOC_OVERRIDE_PDE);
  NVIC_EnableIRQ(INT_GPIOx_VECTOR);
  gpio_register_callback(com_simult_int_handler, TRIGER_INT_PORT,
                         TRIGER_INT_PIN);

  for(;;) {
    PROCESS_WAIT_EVENT();
    /* master part */
    if(ev == serial_line_event_message) {
      /* we convert the input string data to tow int using strlol see 
      https://www.tutorialspoint.com/c_standard_library/c_function_strtol.htm */
      char * str;
      mode = strtol(data, &str, 16);

      PRINTF("Received line: %s\n", (char *)data);
      if(mode == 0x01 ){ /* ranging request */
        uint8_t channel = strtol(str, &str, 10);
      
        if(channel >= 0 && channel < 6){
          NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, (radio_value_t) channel);
          printf("Set the adio to the channel, %u\n", channel);
        NETSTACK_RADIO.get_value(RADIO_PARAM_CHANNEL, (radio_value_t*) &channel);
        printf("Radio is set to the channel, %u\n", channel);
        }
        else{
          printf("Invalide channel value.\n");
        }
      }
      else if(mode == 0x02){ 

        uint8_t channel = 0;
        NETSTACK_RADIO.get_value(RADIO_PARAM_CHANNEL, (radio_value_t*) &channel);
        printf("Radio is set to the channel, %u\n", channel);
      }
      else if(mode == 0x03){ 
        dest_addr = strtol(str, &str, 16);
        printf("Destination address set to %d\n", dest_addr);

      }
      else if(mode == 0x04){ 

        uint16_t n = strtol(str, &str, 10);
        if (n == 0){
          n = 1;
        }
        for (int i = 0; i < n; i++){
          /* trigger the gpio out */
          TRIGGER_OUT_SET();
          clock_delay_usec(10);
          TRIGGER_OUT_CLR();
          clock_delay_usec(1500);
          printf("Trigger interrupt\n");

          /* Reset watchdog and handle polls and events */
          watchdog_periodic();
        }
      }
      else if(mode == 0x5){
        printf("Received messages %d\n", message_received);
        printf("Number of interrupt detected %d\n", interrupt_detected);
      }
      else if(mode == 0x6){
        printf("Reset received messages\n");
        message_received = 0;
        printf("Reset number of interrupt detected\n");
        interrupt_detected = 0;
      }
      else if(mode == 0x7){
        printf("Identify node\n");
        leds_on(LEDS_GREEN);
        clock_delay_usec(1000);
        leds_off(LEDS_GREEN);
        leds_on(LEDS_BLUE);
        clock_delay_usec(1000);
        leds_off(LEDS_BLUE);
      }
    }
  }
  PROCESS_END();
}
PROCESS_THREAD(interrupt_process, ev, data2){
  PROCESS_BEGIN();

  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL); 

    seq_num++;
    uint16_t panid;
    NETSTACK_RADIO.get_value(RADIO_PARAM_PAN_ID, (radio_value_t*) &panid);
    data[4] = (dest_addr >> 8) & 0xFF;
    data[5] = dest_addr & 0xFF;
    data[6] = linkaddr_node_addr.u8[0];
    data[7] = linkaddr_node_addr.u8[1];

    uint8_t frame_len = make_frame(1 /* request ACK */,
              seq_num,
              panid,
              IEEE_SHORT_ADDR,
              dest_addr,
              panid,
              IEEE_SHORT_ADDR,
              linkaddr_node_addr.u8[1] | (linkaddr_node_addr.u8[0] << 8), /* src */
              32,
              data,
              41,
              frame
              );
    // printf("frame_len, %d\n", frame_len);
    NETSTACK_RADIO.send(frame, frame_len);
    // print_frame(frame_len, frame);

    // linkaddr_t addr;

    // packetbuf_copyfrom("Hello", 5);
    // addr.u8[0] = (dest_addr >> 8) & 0xFF;
    // addr.u8[1] = dest_addr & 0xFF;
    // if(!linkaddr_cmp(&addr, &linkaddr_node_addr)) {
    //   unicast_send(&uc, &addr);
    // }
    

    /* Reset watchdog and handle polls and events */
    watchdog_periodic();

  }
  PROCESS_END();
}