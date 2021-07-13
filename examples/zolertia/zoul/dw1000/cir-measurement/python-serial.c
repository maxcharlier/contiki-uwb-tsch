/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         Best-effort single-hop unicast example
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"
#include <stdio.h>
#include "dev/watchdog.h"
#include "dev/serial-line.h"

#include "net/netstack.h"

#include "dev/uart.h"
#define DBG_CONF_UART               0
#define write_byte(b) uart_write_byte(DBG_CONF_UART, b)

#define PRINT_BYTE 1

#undef PRINTF
#if !PRINT_BYTE
  #define PRINTF(...) printf(__VA_ARGS__)
#else /* !PRINT_BYTE */
  #define PRINTF(...) do {} while(0)
#endif /* PRINT_BYTE */


/*---------------------------------------------------------------------------*/
PROCESS(python_serial_process, "Python serial");
AUTOSTART_PROCESSES(&python_serial_process);
/*---------------------------------------------------------------------------*/
static void
python_serial(void)
{

  write_byte((uint8_t) '-');
  write_byte((uint8_t) 'R');
  write_byte((uint8_t) ':');
  write_byte((uint8_t) 0);
  write_byte((uint8_t) 1);
  write_byte((uint8_t) ':');
  /* send to serial the contain of the ACC memory */
  for( uint8_t i =0; i < 255; i++){
    write_byte((uint8_t) i);
    watchdog_periodic(); /* avoid watchdog timer to be reach */
  }
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(python_serial_process, ev, data)
{
    
  PROCESS_BEGIN();

  /* don't wake up the radio as we only use the serial with a timer*/ 
  NETSTACK_RADIO.off();

  while(1) {
    static struct etimer et;
    
    etimer_set(&et, CLOCK_SECOND * 5);
    
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    python_serial();
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
