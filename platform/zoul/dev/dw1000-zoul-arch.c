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

#include "contiki.h"
#include "contiki-net.h"
#include "sys/clock.h"

#include "dev/leds.h"
#include "reg.h"
#include "spi-arch.h"
#include "dev/ioc.h"
#include "dev/sys-ctrl.h"
#include "dev/spi.h"
#include "dev/ssi.h"
#include "dev/gpio.h"
#include "dev/udma.h"

#include <stdio.h>
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
#ifndef DW1000_ARCH_CONF_DMA
#define DW1000_ARCH_CONF_DMA      1
#endif
#define UDMA_SIZE_THRESHOLD 8
/* Channel choose according to the Table 10-1. μDMA Channel Assignments of the 
  CC2538 User Manual */
#define DW1000_CONF_TX_DMA_SPI_ENC   UDMA_CH11_SSI1TX
/* Configure the channel with 8 bit source and destination size,
 * 8 source increment and 0 bit destination increment, one would need to pass */
#define DW1000_TX_CRTL_WORD (UDMA_CHCTL_DSTINC_NONE | \
  UDMA_CHCTL_SRCINC_8 | UDMA_CHCTL_SRCSIZE_8 | UDMA_CHCTL_DSTSIZE_8)
#define DW1000_CONF_RX_DMA_SPI_ENC   UDMA_CH10_SSI1RX
/* SSI receive/transmit data register (R/W) 
  See "SSI_DR" in the CC2538 user manual */
#define CC2538_DW1000_SPI_FIFO_REG    (SSI_BASE(DWM1000_SPI_INSTANCE) + SSI_DR)
#define CC2538_DW1000_SPI_DMA_REG     (SSI_BASE(DWM1000_SPI_INSTANCE) + SSI_DMACTL)
/* Configure the channel with 8 bit source and destination size,
 * 0 source increment and 8 bit destination increment, one would need to pass */
#define DW1000_RX_CRTL_WORD (UDMA_CHCTL_DSTINC_8 | \
  UDMA_CHCTL_SRCINC_NONE | UDMA_CHCTL_SRCSIZE_8 | UDMA_CHCTL_DSTSIZE_8)
/*---------------------------------------------------------------------------*/

// #define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define BUSYWAIT_UNTIL(cond, max_time)                                  \
  do {                                                                  \
    rtimer_clock_t t0;                                                  \
    t0 = RTIMER_NOW();                                                  \
    while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time))) {} \
    if(!(RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time)))) { \
      printf("ARCH: Timeout exceeded in line %d!\n", __LINE__); \
    } \
  } while(0)
#else
  #define PRINTF(...) do {} while (0)
  #define BUSYWAIT_UNTIL(cond, max_time) while(!cond)
#endif
/*---------------------------------------------------------------------------*/
extern int dw1000_driver_interrupt(void); /* declare in dw1000-driver.h */
/*---------------------------------------------------------------------------*/
void
dw1000_int_handler(uint8_t port, uint8_t pin)
{
  /* To keep the gpio_register_callback happy */
  dw1000_driver_interrupt();
}
/*---------------------------------------------------------------------------*/
void
dw1000_arch_spi_select(void)
{
  /* Set CSn to low (0) */
  GPIO_CLR_PIN(DWM1000_SPI_CSN_PORT_BASE, DWM1000_SPI_CSN_PIN_MASK);
  //  /* The MISO pin should go low before chip is fully enabled. */
  // BUSYWAIT_UNTIL(
  //   GPIO_READ_PIN(DWM1000_SPI_MISO_PORT_BASE, DWM1000_SPI_MISO_PIN_MASK) == 0,
  //   RTIMER_SECOND / 100);
}
/*---------------------------------------------------------------------------*/
void
dw1000_arch_spi_deselect(void)
{  
  /* Set CSn to high (1) */
  GPIO_SET_PIN(DWM1000_SPI_CSN_PORT_BASE, DWM1000_SPI_CSN_PIN_MASK);
}
/*---------------------------------------------------------------------------*/
int
dw1000_arch_spi_rw_byte(uint8_t c)
{
  SPIX_WAITFORTxREADY(DWM1000_SPI_INSTANCE);
  SPIX_BUF(DWM1000_SPI_INSTANCE) = c;
  SPIX_WAITFOREOTx(DWM1000_SPI_INSTANCE);
  SPIX_WAITFOREORx(DWM1000_SPI_INSTANCE);
  c = SPIX_BUF(DWM1000_SPI_INSTANCE);

  return c;
}/*---------------------------------------------------------------------------*/
int
dw1000_arch_spi_rw(uint8_t *inbuf, const uint8_t *write_buf, uint16_t len)
{
  int i;
  uint8_t c;

  if((inbuf == NULL && write_buf == NULL) || len <= 0) {
    return 1;
  } else if(inbuf == NULL) {
    /* write on the SPI */
    if(DW1000_ARCH_CONF_DMA && len > UDMA_SIZE_THRESHOLD)
    { /* We will do a uDMA transfert */

      /* Disable peripheral triggers for the channel and enable software usage of
      the channel.*/
      udma_channel_mask_set(DW1000_CONF_TX_DMA_SPI_CHAN);

      /* set the encoding param for the channel */
      udma_set_channel_assignment(DW1000_CONF_TX_DMA_SPI_CHAN, 
                                DW1000_CONF_TX_DMA_SPI_ENC);

      /* Set the transfer source's end address */
      udma_set_channel_src(DW1000_CONF_TX_DMA_SPI_CHAN,
                           (uint32_t)(write_buf) + len - 1);
      /*
       * Set the channel's DST : The SPI FIFO register.
       */
      udma_set_channel_dst(DW1000_CONF_TX_DMA_SPI_CHAN, 
                                CC2538_DW1000_SPI_FIFO_REG);

      udma_channel_use_primary(DW1000_CONF_TX_DMA_SPI_CHAN);
      // printf("write buf start %08X, end %08X\n", (uint32_t)(write_buf), (uint32_t)(write_buf) + len - 1);


      /* Configure the control word 
        In burst mode the arbitration size sould be the same as 
        the number of byte to transfert see 10.3.4.2 Burst Request*/
      udma_set_channel_control_word(DW1000_CONF_TX_DMA_SPI_CHAN,
                              DW1000_TX_CRTL_WORD | udma_xfer_size(len));
                              // UDMA_CHCTL_ARBSIZE_1024);
                              // udma_arb_size(len));

      // udma_channel_use_burst(DW1000_CONF_TX_DMA_SPI_CHAN);

      /* Enabled the RF TX uDMA channel */
      udma_channel_enable(DW1000_CONF_TX_DMA_SPI_CHAN);


      /* Enable uDMA for the transmit SSI FIFO */
      REG(CC2538_DW1000_SPI_DMA_REG) |= SSI_DMACTL_TXDMAE_M;

      // spix_enable(DWM1000_SPI_INSTANCE);

      /* Trigger the uDMA transfer */
      udma_channel_sw_request(DW1000_CONF_TX_DMA_SPI_CHAN);

      uint32_t result = udma_channel_get_mode(DW1000_CONF_TX_DMA_SPI_CHAN);
      // printf("max channel %d\n", UDMA_CONF_MAX_CHANNEL);
      printf("udma_channel_get_mode(DW1000_CONF_TX_DMA_SPI_CHAN) %08X\n", result);

      /* Wait for the transfer to complete. */
      while(udma_channel_get_mode(DW1000_CONF_TX_DMA_SPI_CHAN)
               != UDMA_CHCTL_XFERMODE_STOP);

      clock_delay_usec(200);
    
      /* Disable uDMA for the transmit SSI FIFO */
      REG(CC2538_DW1000_SPI_DMA_REG) &= ~SSI_DMACTL_TXDMAE_M;
    }
    else {
      for(i = 0; i < len; i++) {
        // PRINTF("dwm1000_arch_spi_rw write  0x%2x\n", write_buf[i]);

        SPIX_WAITFORTxREADY(DWM1000_SPI_INSTANCE);
        SPIX_BUF(DWM1000_SPI_INSTANCE) = write_buf[i];
        SPIX_WAITFOREOTx(DWM1000_SPI_INSTANCE);
        SPIX_WAITFOREORx(DWM1000_SPI_INSTANCE);
        c = SPIX_BUF(DWM1000_SPI_INSTANCE);
        /* read and discard to avoid "variable set but not used" warning */
        (void)c;
      }
    }
  } else if(write_buf == NULL) {
    /* read on the SPI */
    for(i = 0; i < len; i++) {

      SPIX_WAITFORTxREADY(DWM1000_SPI_INSTANCE);
      SPIX_BUF(DWM1000_SPI_INSTANCE) = 0;
      SPIX_WAITFOREOTx(DWM1000_SPI_INSTANCE);
      SPIX_WAITFOREORx(DWM1000_SPI_INSTANCE);
      inbuf[i] = SPIX_BUF(DWM1000_SPI_INSTANCE);
    }
  } else {
    for(i = 0; i < len; i++) {
      SPIX_WAITFORTxREADY(DWM1000_SPI_INSTANCE);
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
dw1000_arch_gpio8_setup_irq(void)
{

  GPIO_SOFTWARE_CONTROL(DWM1000_INT_PORT_BASE, DWM1000_INT_PIN_MASK);
  GPIO_SET_INPUT(DWM1000_INT_PORT_BASE, DWM1000_INT_PIN_MASK);
  GPIO_DETECT_EDGE(DWM1000_INT_PORT_BASE, DWM1000_INT_PIN_MASK);
  GPIO_TRIGGER_SINGLE_EDGE(DWM1000_INT_PORT_BASE, DWM1000_INT_PIN_MASK);

  GPIO_DETECT_RISING(DWM1000_INT_PORT_BASE, DWM1000_INT_PIN_MASK);

  GPIO_ENABLE_INTERRUPT(DWM1000_INT_PORT_BASE, DWM1000_INT_PIN_MASK);
  ioc_set_over(DWM1000_INT_PORT, DWM1000_INT_PIN, IOC_OVERRIDE_PUE);
  NVIC_EnableIRQ(DWM1000_GPIOx_VECTOR);
  gpio_register_callback(dw1000_int_handler, DWM1000_INT_PORT,
                         DWM1000_INT_PIN);
}/*---------------------------------------------------------------------------*/
void
dw1000_arch_gpio8_enable_irq(void)
{
  GPIO_ENABLE_INTERRUPT(DWM1000_INT_PORT_BASE, DWM1000_INT_PIN_MASK);
  ioc_set_over(DWM1000_INT_PORT, DWM1000_INT_PIN, IOC_OVERRIDE_PUE);
  NVIC_EnableIRQ(DWM1000_GPIOx_VECTOR);
}
/*---------------------------------------------------------------------------*/
void
dw1000_arch_gpio8_disable_irq(void)
{
  GPIO_DISABLE_INTERRUPT(DWM1000_INT_PORT_BASE, DWM1000_INT_PIN_MASK);
  NVIC_DisableIRQ(RF_TX_RX_IRQn);                         /* disable RF interrupts */

}
/*---------------------------------------------------------------------------*/
int
dw1000_arch_gpio8_read_pin(void)
{
  return GPIO_READ_PIN(DWM1000_INT_PORT_BASE, DWM1000_INT_PIN_MASK);
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Initialize the architecture specific part of the DW1000
 **/
void dw1000_arch_init()
{
  PRINTF("dw1000_arch_init\n");

  /* Initialize CSn */
  spix_cs_init(DWM1000_SPI_CSN_PORT, DWM1000_SPI_CSN_PIN);

  /* Initialize SPI */
  PRINTF("DWM1000_SPI_INSTANCE %d\n", (int) DWM1000_SPI_INSTANCE);

  /* Configure SPI (CPOL = 1, CPHA = 0) */
  spix_init(DWM1000_SPI_INSTANCE);

  /* Change the SPI configuration to CPOL = 0, CPHA = 1 
    clock_polarity = 0 : low when IDLE 
    clock_phase = 0 data send on the second edge of the clock*/
  spix_set_mode(DWM1000_SPI_INSTANCE, SSI_CR0_FRF_MOTOROLA, 0, 0, 8);

  /* Leave CSn as default */
  dw1000_arch_spi_deselect();

  /* Ensure MISO is low */
  // BUSYWAIT_UNTIL(
  //   (GPIO_READ_PIN(DWM1000_SPI_MISO_PORT_BASE, DWM1000_SPI_MISO_PIN_MASK) == 0),
  //   RTIMER_SECOND / 10);

  if(DW1000_ARCH_CONF_DMA) {
    printf("dma enabled\n");
  }
}
/**
 * \brief     Wait a delay in microsecond.
 *
 * \param ms  The delay in microsecond.
 **/
void dw1000_us_delay(int us){
  clock_delay_usec(us);
}
/**
 * Change the SPI frequency to freq.
 * If freq is bigger than the maximum SPI frequency value of the embedeed 
 * system set this maximum value.
 **/
void dw1000_arch_spi_set_clock_freq(uint32_t freq){
  spix_set_clock_freq(DWM1000_SPI_INSTANCE, freq);
}


