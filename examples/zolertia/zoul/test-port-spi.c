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
 *         A simple program to test the SPI Ports.
 *
 * \author
 *         Charlier Maximilien  <maximilien.charlier@umons.ac.be>
 */

#include "contiki.h"
#include "contiki-net.h"
#include "sys/clock.h"
#include "assert.h"

#include "reg.h"
#include "spi-arch.h"
#include "dev/ioc.h"
#include "dev/sys-ctrl.h"
#include "dev/spi.h"
#include "dev/ssi.h"
#include "dev/gpio.h"

#include <stdio.h> /* For printf() */

/*---------------------------------------------------------------------------*/
#define DWM1000_SPI_CLK_PORT_BASE   GPIO_PORT_TO_BASE(DWM1000_CLK_PORT)
#define DWM1000_SPI_CLK_PIN_MASK    GPIO_PIN_MASK(DWM1000_CLK_PIN)
#define DWM1000_SPI_MOSI_PORT_BASE  GPIO_PORT_TO_BASE(DWM1000_MOSI_PORT)
#define DWM1000_SPI_MOSI_PIN_MASK   GPIO_PIN_MASK(DWM1000_MOSI_PIN)
#define DWM1000_SPI_MISO_PORT_BASE  GPIO_PORT_TO_BASE(DWM1000_MISO_PORT)
#define DWM1000_SPI_MISO_PIN_MASK   GPIO_PIN_MASK(DWM1000_MISO_PIN)
#define DWM1000_SPI_CSN_PORT_BASE   GPIO_PORT_TO_BASE(DWM1000_SPI_CSN_PORT)
#define DWM1000_SPI_CSN_PIN_MASK    GPIO_PIN_MASK(DWM1000_SPI_CSN_PIN)
#define DWM1000_INT_PORT_BASE      GPIO_PORT_TO_BASE(DWM1000_INT_PORT)
#define DWM1000_INT_PIN_MASK       GPIO_PIN_MASK(DWM1000_INT_PIN)
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
PROCESS(port_spi_test_process, "TEST SPI PORT process");
AUTOSTART_PROCESSES(&port_spi_test_process);
/*---------------------------------------------------------------------------*/

void set_output_port(void){
  spix_cs_init(DWM1000_SPI_CSN_PORT, DWM1000_SPI_CSN_PIN);
  spix_cs_init(DWM1000_CLK_PORT, DWM1000_CLK_PIN);
  spix_cs_init(DWM1000_MOSI_PORT, DWM1000_MOSI_PIN);
  spix_cs_init(DWM1000_MISO_PORT, DWM1000_MISO_PIN);
  spix_cs_init(DWM1000_INT_PORT, DWM1000_INT_PIN);
}

void set_port_hight(void){
  GPIO_SET_PIN(DWM1000_SPI_CSN_PORT_BASE, DWM1000_SPI_CSN_PIN_MASK);
  GPIO_SET_PIN(DWM1000_SPI_CLK_PORT_BASE, DWM1000_SPI_CLK_PIN_MASK);
  GPIO_SET_PIN(DWM1000_SPI_MOSI_PORT_BASE, DWM1000_SPI_MOSI_PIN_MASK);
  GPIO_SET_PIN(DWM1000_SPI_MISO_PORT_BASE, DWM1000_SPI_MISO_PIN_MASK);
  GPIO_SET_PIN(DWM1000_INT_PORT_BASE, DWM1000_INT_PIN_MASK);
}

void set_port_low(void){
  GPIO_CLR_PIN(DWM1000_SPI_CSN_PORT_BASE, DWM1000_SPI_CSN_PIN_MASK);
  GPIO_CLR_PIN(DWM1000_SPI_CLK_PORT_BASE, DWM1000_SPI_CLK_PIN_MASK);
  GPIO_CLR_PIN(DWM1000_SPI_MOSI_PORT_BASE, DWM1000_SPI_MOSI_PIN_MASK);
  GPIO_CLR_PIN(DWM1000_SPI_MISO_PORT_BASE, DWM1000_SPI_MISO_PIN_MASK);
  GPIO_CLR_PIN(DWM1000_INT_PORT_BASE, DWM1000_INT_PIN_MASK);
}

PROCESS_THREAD(port_spi_test_process, ev, data)
{
  static struct etimer timer;
  static int csn = 0;

  PROCESS_BEGIN();
  printf("Startup of the SPI TEST\n");

  set_output_port();

  etimer_set(&timer, CLOCK_SECOND);
  while (1) {
    PROCESS_WAIT_EVENT();
    if (ev == PROCESS_EVENT_TIMER) {

      printf("CSN %d\n", csn);
      if(csn == 1){
        set_port_hight();
        csn = 0;
      }
      else{
        set_port_low();
        csn = 1;
      }

      etimer_reset(&timer);
    }
  }

  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
