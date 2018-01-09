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
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup zoul
 * @{
 *
 * \defgroup zoul-dwm1000 Zoul DWM1000 arch
 *
 *         Hardware depends declaration of function for the 
 *              Decawave DW1000 usage with a RE-Mote.
 * @{
 *
 * \file
 * CC1200 Zoul arch specifics
 */
/*---------------------------------------------------------------------------*/
#include "dw1000-arch.h"
#include "dw1000-driver.h" /* link to dw1000_driver_interrupt */

#include "contiki.h"
#include "sys/clock.h"

#include "reg.h"
#include "spi-arch.h"
#include "dev/ioc.h"
#include "dev/sys-ctrl.h"
#include "dev/spi.h"
#include "dev/ssi.h"
#include "dev/gpio.h"
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

#if DEBUG
  #include <stdio.h>
  #define PRINTF(...) printf(__VA_ARGS__)
#else
  #define PRINTF(...) do {} while (0)
#endif

/*---------------------------------------------------------------------------*/
void
dwm1000_int_handler(uint8_t port, uint8_t pin)
{
  /* To keep the gpio_register_callback happy */
  dw1000_driver_interrupt();
}
/*---------------------------------------------------------------------------*/
void
dwm1000_arch_spi_select(void)
{
  /* Set CSn to low (0) */
  GPIO_CLR_PIN(DWM1000_SPI_CLK_PORT_BASE, DWM1000_SPI_CLK_PIN_MASK);
  /* The MISO pin should go low before chip is fully enabled. */
  BUSYWAIT_UNTIL(
    GPIO_READ_PIN(DWM1000_SPI_MISO_PORT_BASE, DWM1000_SPI_MISO_PIN_MASK) == 0,
    RTIMER_SECOND / 100);
}
/*---------------------------------------------------------------------------*/
void
dwm1000_arch_spi_deselect(void)
{
  /* Set CSn to high (1) */
  GPIO_SET_PIN(DWM1000_SPI_CLK_PORT_BASE, DWM1000_SPI_CLK_PIN_MASK);
}
/*---------------------------------------------------------------------------*/
int
dwm1000_arch_spi_rw_byte(uint8_t c)
{
  SPI_WAITFORTx_BEFORE();
  SPIX_BUF(DWM1000_SPI_INSTANCE) = c;
  SPIX_WAITFOREOTx(DWM1000_SPI_INSTANCE);
  SPIX_WAITFOREORx(DWM1000_SPI_INSTANCE);
  c = SPIX_BUF(DWM1000_SPI_INSTANCE);

  return c;
}/*---------------------------------------------------------------------------*/
int
dwm1000_arch_spi_rw(uint8_t *inbuf, const uint8_t *write_buf, uint16_t len)
{
  int i;
  uint8_t c;

  if((inbuf == NULL && write_buf == NULL) || len <= 0) {
    return 1;
  } else if(inbuf == NULL) {
    for(i = 0; i < len; i++) {
      SPI_WAITFORTx_BEFORE();
      SPIX_BUF(DWM1000_SPI_INSTANCE) = write_buf[i];
      SPIX_WAITFOREOTx(DWM1000_SPI_INSTANCE);
      SPIX_WAITFOREORx(DWM1000_SPI_INSTANCE);
      c = SPIX_BUF(DWM1000_SPI_INSTANCE);
      /* read and discard to avoid "variable set but not used" warning */
      (void)c;
    }
  } else if(write_buf == NULL) {
    for(i = 0; i < len; i++) {
      SPI_WAITFORTx_BEFORE();
      SPIX_BUF(DWM1000_SPI_INSTANCE) = 0;
      SPIX_WAITFOREOTx(DWM1000_SPI_INSTANCE);
      SPIX_WAITFOREORx(DWM1000_SPI_INSTANCE);
      inbuf[i] = SPIX_BUF(DWM1000_SPI_INSTANCE);
    }
  } else {
    for(i = 0; i < len; i++) {
      SPI_WAITFORTx_BEFORE();
      SPIX_BUF(DWM1000_SPI_INSTANCE) = write_buf[i];
      SPIX_WAITFOREOTx(DWM1000_SPI_INSTANCE);
      SPIX_WAITFOREORx(DWM1000_SPI_INSTANCE);
      inbuf[i] = SPIX_BUF(DWM1000_SPI_INSTANCE);
    }
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
void
dwm1000_arch_gpio8_setup_irq(int rising)
{

  GPIO_SOFTWARE_CONTROL(DWM1000_INT_PORT_BASE, DWM1000_INT_PIN_MASK);
  GPIO_SET_INPUT(DWM1000_INT_PORT_BASE, DWM1000_INT_PIN_MASK);
  GPIO_DETECT_EDGE(DWM1000_INT_PORT_BASE, DWM1000_INT_PIN_MASK);
  GPIO_TRIGGER_SINGLE_EDGE(DWM1000_INT_PORT_BASE, DWM1000_INT_PIN_MASK);

  if(rising) {
    GPIO_DETECT_RISING(DWM1000_INT_PORT_BASE, DWM1000_INT_PIN_MASK);
  } else {
    GPIO_DETECT_FALLING(DWM1000_INT_PORT_BASE, DWM1000_INT_PIN_MASK);
  }

  GPIO_ENABLE_INTERRUPT(DWM1000_INT_PORT_BASE, DWM1000_INT_PIN_MASK);
  ioc_set_over(DWM1000_INT_PORT_BASE, DWM1000_INT_PIN_MASK, IOC_OVERRIDE_PUE);
  nvic_interrupt_enable(CC1200_GPIOx_VECTOR);
  gpio_register_callback(dwm1000_int_handler, DWM1000_INT_PORT_BASE,
                         DWM1000_INT_PIN_MASK);
}/*---------------------------------------------------------------------------*/
void
dwm1000_arch_gpio8_enable_irq(void)
{
  GPIO_ENABLE_INTERRUPT(DWM1000_INT_PORT_BASE, DWM1000_INT_PIN_MASK);
  ioc_set_over(DWM1000_INT_PORT_BASE, DWM1000_INT_PIN_MASK, IOC_OVERRIDE_PUE);
  nvic_interrupt_enable(DWM1000_GPIOx_VECTOR);
}
/*---------------------------------------------------------------------------*/
void
dwm1000_arch_gpio8_disable_irq(void)
{
  GPIO_DISABLE_INTERRUPT(DWM1000_INT_PORT_BASE, DWM1000_INT_PIN_MASK);
}
/*---------------------------------------------------------------------------*/
int
dwm1000_arch_gpio8_read_pin(void)
{
  return GPIO_READ_PIN(DWM1000_INT_PORT_BASE, DWM1000_INT_PIN_MASK);
}
/*---------------------------------------------------------------------------*/


/** \brief Initialize the architecture specific part of the DW1000
**/
void dw1000_arch_init()
{
  /* Initialize CSn, enable CSn and then wait for MISO to go low*/
  spix_cs_init(DWM1000_SPI_CSN_PORT, DWM1000_SPI_CSN_PIN);

  /* Initialize SPI */
  spix_init(DWM1000_SPI_INSTANCE);

  /* Configure GPIOx */
  GPIO_SOFTWARE_CONTROL(DWM1000_INT_PORT_BASE, DWM1000_INT_PIN_MASK);
  GPIO_SET_INPUT(DWM1000_INT_PORT_BASE, DWM1000_INT_PIN_MASK);

  /* Leave CSn as default */
  dwm1000_arch_spi_deselect();

  /* Ensure MISO is high */
  BUSYWAIT_UNTIL(
    GPIO_READ_PIN(DWM1000_SPI_MISO_PORT_BASE, DWM1000_SPI_MISO_PIN_MASK),
    RTIMER_SECOND / 10);

}

/**
 * \brief     Wait a delay in microsecond.
 *
 * \param ms  The delay in microsecond.
 */
void dw1000_us_delay(int ms){
 clock_delay_usec(ms);
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
void dw_read_subreg(uint32_t reg_addr, uint16_t subreg_addr, 
                    uint16_t subreg_len, uint8_t * p_data)
{

  /* SPI communications */ 

  /* Disable interrupt */
  // dint();

  dwm1000_arch_spi_select(); 
  /* write bit = 1, sub-reg present bit = 1 */
  dwm1000_arch_spi_rw_byte(subreg_addr > 0?0x40:0x00) | (reg_addr & 0x3F));
  if (subreg_addr > 0) {
    if (subreg_addr > 0x7F) {
      /* extended address bit = 1 */
      dwm1000_arch_spi_rw_byte(0x80 | (subreg_addr & 0x7F));
      dwm1000_arch_spi_rw_byte((subreg_addr >> 7) & 0xFF);
    } else {
      /* extended address bit = 0 */
      dwm1000_arch_spi_rw_byte(subreg_addr & 0x7F);
    }
  }
  dwm1000_arch_spi_rw(p_data, NULL, subreg_len);
  dwm1000_arch_spi_deselect();


  /* Re enable interrupt */
  // eint();

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
void dw_write_subreg(uint32_t reg_addr, uint16_t subreg_addr, 
                      uint16_t subreg_len, const uint8_t *p_data)
{
  /* SPI communications */

  /* Disable interrupt */
  // dint();

  dwm1000_arch_spi_select(); 
  /* write bit = 1, sub-reg present bit = 1 */
  dwm1000_arch_spi_rw_byte(0x80 | (subreg_addr > 0 ?0x40:0x00) | (reg_addr & 0x3F));
  if (subreg_addr > 0) {
    if (subreg_addr > 0x7F) {
      /* extended address bit = 1 */
      dwm1000_arch_spi_rw_byte(0x80 | (subreg_addr & 0x7F));
      dwm1000_arch_spi_rw_byte((subreg_addr >> 7) & 0xFF);
    } else {
      /* extended address bit = 0 */
      dwm1000_arch_spi_rw_byte(subreg_addr & 0x7F);
    }
  }
  dwm1000_arch_spi_rw(NULL, p_data, subreg_len);
  dwm1000_arch_spi_deselect();

  /* Re enable interrupt */
  // eint();
}