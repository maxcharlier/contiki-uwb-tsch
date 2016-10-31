/*
 * Copyright (c) 2016, UMons University & Luleå University of Technology
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
 *         Hardware depends declaration of function for the 
 *              Decawave DW1000 usage with a Zolertia Z1.
 *
 * \author
 *         Charlier Maximilien  <maximilien-charlier@outlook.com>
 *         Quoitin Bruno        <bruno.quoitin@umons.ac.be>
 */

#include "dw1000-arch.h"
#include "dw1000-driver.h" /* link to dw1000_driver_interrupt */

#include "contiki.h"
#include "msp430.h"
#include "sys/clock.h"
#include "watchdog.h"
#include "isr_compat.h"
#include "dev/spi.h"
 
#if DEBUG
  #include <stdio.h>
  #define PRINTF(...) printf(__VA_ARGS__)
#else
  #define PRINTF(...) do {} while (0)
#endif

/* === Port 2 interrupt vector === */
/* Note : on the z1-dw1000 platform, We have disable interrupt from the
 * "button-sensor", see platform/z1-dw1000/dev/button-sensor.c
 * A more long term solution would be to build a Port2 IRQ handler
 * that would distinguish between button, dw1000, ... and dispatch
 * to more specific ISRs 
 */
ISR(DW1000_IRQ, dw1000_irq_handler){
  PRINTF("DW1000-driver ISR \r\n");

  ENERGEST_ON(ENERGEST_TYPE_IRQ);
  if (P2IFG & BV(DW1000_INT_PIN)) {
    P2IFG &= ~BV(DW1000_INT_PIN);
    dw1000_driver_interrupt();
    LPM4_EXIT;
  }
  P2IFG= 0x00;
  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}


/** \brief Initialize the architecture specific part of the DW1000
**/
void dw1000_arch_init()
{
  spi_init();

  /* Configure port direction */
  DW1000_CSN_PORT(DIR) |= BV(DW1000_CSN_PIN);
  DW1000_INT_PORT(DIR) &= ~BV(DW1000_INT_PIN);

  /* Enable interrupts on INT pin */
  /* Note : DW1000's IRQ output is active high by default
   * but can be configured otherwise */
  dint(); /* Disable interrupt */
  P2IES &= ~BV(DW1000_INT_PIN); /* low to high edge */
  P2IE |= BV(DW1000_INT_PIN); /* enabled */
  eint(); /* Re enable interrupt */


  /* UCCKPL=0  (clock active high)
   * UCCKPH=1  (catch the first transition -- i.e. rising edge,
   *            send the second transition -- i.e. falling edge) */
  UCB0CTL0 &= ~UCCKPL;
  UCB0CTL0 |= UCCKPH;

  /* Chip select is disabled (=high) */
  DW1000_SPI_DISABLE();
}

/**
 * \brief     Wait a delay in microsecond.
 *
 * \param ms  The delay in microsecond.
 */
void dw1000_us_delay(int ms){
  int i;
  for (i= 0; i < ms; i++) {
    clock_delay(1);
    watchdog_periodic();
  }
}

/**
 * \brief                 Reads the value from a sub-register on the DW1000 as 
 *                        a byte stream.

 * \param[in] reg_addr    Register address as specified in the manual and by
 *                        the DW_REG_* defines.
 * \param[in] subreg_addr Sub-register address as specified in the manual and
 *                        by the DW_SUBREG_* defines.
 * \param[in] subreg_len  Number of bytes to read. Should not be longer than
 *                        the length specified in the manual or the
 *                        DW_SUBLEN_* defines.
 * \param[out] p_data     Data read from the device.
 */
void dw_read_subreg(uint32_t reg_addr, uint16_t subreg_addr, uint16_t subreg_len, uint8_t * p_data)
{

  /* SPI communications */ 

  /* Disable interrupt */
  dint();

  /* Asserting CS */
  DW1000_SPI_ENABLE();

  /* Read instruction */
  /* write bit = 0, sub-reg present bit = 1 */
  SPI_WRITE_FAST((subreg_addr > 0?0x40:0x00) | (reg_addr & 0x3F));
  if (subreg_addr > 0) {
    if (subreg_addr > 0x7F) {
      /* extended address bit = 1 */
      SPI_WRITE_FAST(0x80 | (subreg_addr & 0x7F));
      SPI_WRITE_FAST((subreg_addr >> 7) | 0xFF);
    } else {
      /* extended address bit = 0 */
      SPI_WRITE_FAST(0x00 | (subreg_addr & 0x7F));
    }
  }
  SPI_WAITFORTx_ENDED();

  SPI_FLUSH(); /* discard data read during previous write */

  while (subreg_len-- > 0)
    SPI_READ(*(p_data++));
  
  /* De-asserting CS */
  DW1000_SPI_DISABLE();

  /* Re enable interrupt */
  eint();
}

/**
 * \brief                 Writes a value to a sub-register on the DW1000 as a 
 *                        byte stream.
 *
 * \param[in] reg_addr    Register address as specified in the manual and by
 *                        the DW_REG_* defines.
 * \param[in] subreg_addr Sub-register address as specified in the manual and
 *                        by the DW_SUBREG_* defines.
 * \param[in] subreg_len  Number of bytes to write. Should not be longer
 *                        than the length specified in the manual or the
 *                        DW_SUBLEN_* defines.
 * \param[in] p_data      A stream of bytes to write to device.
 */
void dw_write_subreg(uint32_t reg_addr, uint16_t subreg_addr, uint16_t subreg_len, const uint8_t *p_data)
{
  /* SPI communications */

  /* Disable interrupt */
  dint();
  /* Asserting CS */
  DW1000_SPI_ENABLE();

  /* write bit = 1, sub-reg present bit = 1 */
  SPI_WRITE_FAST(0x80 | (subreg_addr > 0 ?0x40:0x00) | (reg_addr & 0x3F));
  if (subreg_addr > 0) {
    if (subreg_addr > 0x7F) {
      /* extended address bit = 1 */
      SPI_WRITE_FAST(0x80 | (subreg_addr & 0x7F));
      SPI_WRITE_FAST((subreg_addr >> 7) | 0xFF);
    } else {
      /* extended address bit = 0 */
      SPI_WRITE_FAST(0x00 | (subreg_addr & 0x7F));
    }
  }
  
  while (subreg_len-- > 0)
    SPI_WRITE_FAST( *(p_data++) );
  SPI_WAITFORTx_ENDED();
        
  /* De-asserting CS */
  DW1000_SPI_DISABLE();

  /* Re enable interrupt */
  eint();
}