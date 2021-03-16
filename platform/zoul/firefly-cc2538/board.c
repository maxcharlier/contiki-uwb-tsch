/*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
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
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup firefly
 * @{
 *
 * \file
 *  Board-initialisation for the Zolertia's Firefly platform
 *
 */
/*---------------------------------------------------------------------------*/
#include "contiki-conf.h"
#include <stdint.h>
#include <string.h>

#include "spi-arch.h"
#include "dev/gpio.h"
#include "dev/spi.h"
#include "dev/ioc.h"

#define DWM1000_SPI_CLK_PORT_BASE   GPIO_PORT_TO_BASE(DWM1000_CLK_PORT)
#define DWM1000_SPI_CLK_PIN_MASK    GPIO_PIN_MASK(DWM1000_CLK_PIN)
#define DWM1000_SPI_MOSI_PORT_BASE  GPIO_PORT_TO_BASE(DWM1000_MOSI_PORT)
#define DWM1000_SPI_MOSI_PIN_MASK   GPIO_PIN_MASK(DWM1000_MOSI_PIN)
#define DWM1000_SPI_MISO_PORT_BASE  GPIO_PORT_TO_BASE(DWM1000_MISO_PORT)
#define DWM1000_SPI_MISO_PIN_MASK   GPIO_PIN_MASK(DWM1000_MISO_PIN)
#define DWM1000_SPI_CSN_PORT_BASE   GPIO_PORT_TO_BASE(DWM1000_SPI_CSN_PORT)
#define DWM1000_SPI_CSN_PIN_MASK    GPIO_PIN_MASK(DWM1000_SPI_CSN_PIN)
#define DWM1000_INT_PORT_BASE       GPIO_PORT_TO_BASE(DWM1000_INT_PORT)
#define DWM1000_INT_PIN_MASK        GPIO_PIN_MASK(DWM1000_INT_PIN)
#define DWM1000_WAKEUP_PORT_BASE    GPIO_PORT_TO_BASE(DWM1000_WAKEUP_PORT)
#define DWM1000_WAKEUP_PIN_MASK     GPIO_PIN_MASK(DWM1000_WAKEUP_PIN)
#define DWM1000_RESET_PORT_BASE    GPIO_PORT_TO_BASE(DWM1000_RESET_PORT)
#define DWM1000_RESET_PIN_MASK     GPIO_PIN_MASK(DWM1000_RESET_PIN)
/*---------------------------------------------------------------------------*/
/**
 * Configure the pin of the board to avoid problems with the DW1000 transceiver.
 * The transceiver is powered but the SPI Select pin is deselected.
 * The 
 */
void dw1000_arch_disable_for_cc2538(void){
  /* Initialize CSn */
  spix_cs_init(DWM1000_SPI_CSN_PORT, DWM1000_SPI_CSN_PIN);

  /* Leave CSn as default (unselected) */
  SPIX_CS_SET(DWM1000_SPI_CSN_PORT, DWM1000_SPI_CSN_PIN);

  /* init the wake up port in OUTPUT, LOW, with PULL DOWN resistor */
  GPIO_SET_OUTPUT(DWM1000_WAKEUP_PORT_BASE, DWM1000_WAKEUP_PIN_MASK);
  ioc_set_over(DWM1000_WAKEUP_PORT, DWM1000_WAKEUP_PIN, IOC_OVERRIDE_PDE);
  GPIO_CLR_PIN(DWM1000_WAKEUP_PORT_BASE, DWM1000_WAKEUP_PIN_MASK);
}
static void
configure_unused_pins(void)
{
  /* FIXME */
}
/*---------------------------------------------------------------------------*/
void
board_init()
{
  configure_unused_pins();
  dw1000_arch_disable_for_cc2538();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 */

