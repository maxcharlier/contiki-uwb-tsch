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
 * 			Initialize SPI module & Pins for the usage of the DWM1000.
 *
 * 			The function has to accomplish the following tasks:
 * 				- Enable SPI and configure SPI (CPOL = 0, CPHA = 1)
 * 				- Configure MISO, MOSI, SCLK accordingly
 * 				- Configure GPIOx (input)
 * 				- Configure CSn (output)
 * \author
 *         Charlier Maximilien  <maximilien.charlier@umons.ac.be>
 */


#ifndef __DW1000_ARCH_H__
#define __DW1000_ARCH_H__

#include <stdint.h> // To add interger type uint32_t ...

/*---------------------------------------------------------------------------*/
/**
 * Initialize SPI module & Pins.
 *
 * The function has to accomplish the following tasks:
 * - Enable SPI and configure SPI (CPOL = 0, CPHA = 1)
 * - Configure MISO, MOSI, SCLK accordingly
 * - Configure GPIOx (input)
 * - Configure CSn (output)
 **/
void dw1000_arch_init();
/*---------------------------------------------------------------------------*/
void dw_read_subreg(uint32_t reg_addr, uint16_t subreg_addr, 
            uint16_t subreg_len, uint8_t *p_data);
void dw_write_subreg(uint32_t reg_addr, uint16_t subreg_addr, 
            uint16_t subreg_len, const uint8_t *data);
/*---------------------------------------------------------------------------*/
/**
 * Configure port IRQ for GPIO8.
 *
 *
 * If rising == 1: configure IRQ for rising edge, else falling edge
 * Interrupt has to call dw1000_driver_interrupt()!
 **/
void dw1000_arch_gpio8_setup_irq(void);
/*---------------------------------------------------------------------------*/
/** Reset interrupt flag and enable GPIO8 port IRQ. */
void dw1000_arch_gpio8_enable_irq(void);
/*---------------------------------------------------------------------------*/
/* Reset interrupt flag and enable GPIO8 port IRQ. */
void dw1000_arch_gpio8_disable_irq(void);
/*---------------------------------------------------------------------------*/
/**
 * Read back the status of the GPIO8 pin.
 * Returns 0 if the pin is low, otherwise 1
 **/
int dw1000_arch_gpio8_read_pin(void);
/*---------------------------------------------------------------------------*/
/**
 * Change the SPI frequency to freq.
 * If freq is bigger than the maximum SPI frequency value of the embedeed 
 * system set this maximum value.
 **/
void dw1000_arch_spi_set_clock_freq(uint32_t freq);
/*---------------------------------------------------------------------------*/
/**
 * Wait a delay in microsecond.
 **/
void dw1000_us_delay(int us);

#endif /* __DW1000_ARCH_H__ */
