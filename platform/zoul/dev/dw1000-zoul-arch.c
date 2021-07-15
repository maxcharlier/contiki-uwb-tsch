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
#include "dw1000-const.h"

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
#include "assert.h"
#include "dev/ioc.h"
#include "dev/watchdog.h"

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
#define DWM1000_INT_PORT_BASE       GPIO_PORT_TO_BASE(DWM1000_INT_PORT)
#define DWM1000_INT_PIN_MASK        GPIO_PIN_MASK(DWM1000_INT_PIN)
#define DWM1000_WAKEUP_PORT_BASE    GPIO_PORT_TO_BASE(DWM1000_WAKEUP_PORT)
#define DWM1000_WAKEUP_PIN_MASK     GPIO_PIN_MASK(DWM1000_WAKEUP_PIN)
#define DWM1000_RESET_PORT_BASE    GPIO_PORT_TO_BASE(DWM1000_RESET_PORT)
#define DWM1000_RESET_PIN_MASK     GPIO_PIN_MASK(DWM1000_RESET_PIN)
/*---------------------------------------------------------------------------*/
#define DW1000_SPI_RX_FLUSH(n)    do { \
    for(int i = 0; i < n; i++) { \
      /* RX FIFO is not empty */  \
      SPIX_WAITFOREORx(DWM1000_SPI_INSTANCE);  \
      /* read the receive FIFO to clear it*/ \
      SPIX_BUF(DWM1000_SPI_INSTANCE); \
      } \
  } while(0)

#define DW1000_SELECT()      SPIX_CS_CLR(DWM1000_SPI_CSN_PORT, \
                                      DWM1000_SPI_CSN_PIN)
#define DW1000_DESELECT()    SPIX_CS_SET(DWM1000_SPI_CSN_PORT, \
                                      DWM1000_SPI_CSN_PIN)

/*---------------------------------------------------------------------------*/
/* DMA configuration */
#ifndef DW1000_ARCH_CONF_DMA
#define DW1000_ARCH_CONF_DMA      1
#endif
#define SPI_FIFO_SIZE             8
#define UDMA_SIZE_THRESHOLD       SPI_FIFO_SIZE + SPI_FIFO_SIZE/2

#define SSI_FIFO_SIZE             8
// #undef DW1000_ARCH_CONF_DMA
// #define DW1000_ARCH_CONF_DMA 0
/* Channel choose according to the Table 10-1. μDMA Channel Assignments of the 
  CC2538 User Manual */
#define DW1000_CONF_TX_DMA_SPI_ENC   UDMA_CH11_SSI1TX

/* Configure the channel with 8 bit source and destination size,
 * 8 source increment and 0 bit destination increment, one would need to pass */
#define DW1000_TX_CRTL_WORD (UDMA_CHCTL_XFERMODE_BASIC  \
  | UDMA_CHCTL_SRCINC_8 | UDMA_CHCTL_SRCSIZE_8 \
  | UDMA_CHCTL_DSTINC_NONE | UDMA_CHCTL_DSTSIZE_8)

#define DW1000_TX_FOR_RX_CRTL_WORD (UDMA_CHCTL_XFERMODE_BASIC  \
  | UDMA_CHCTL_SRCINC_NONE | UDMA_CHCTL_SRCSIZE_8 \
  | UDMA_CHCTL_DSTINC_NONE | UDMA_CHCTL_DSTSIZE_8)

#define DW1000_CONF_RX_DMA_SPI_ENC   UDMA_CH10_SSI1RX

/* Configure the channel with 8 bit source and destination size,
 * 0 source increment and 8 bit destination increment, one would need to pass */
#define DW1000_RX_CRTL_WORD (UDMA_CHCTL_XFERMODE_BASIC  \
  | UDMA_CHCTL_SRCINC_NONE | UDMA_CHCTL_SRCSIZE_8 \
  | UDMA_CHCTL_DSTINC_8 | UDMA_CHCTL_DSTSIZE_8)

#define DW1000_RX_FOR_TX_CRTL_WORD (UDMA_CHCTL_XFERMODE_BASIC  \
  | UDMA_CHCTL_SRCINC_NONE | UDMA_CHCTL_SRCSIZE_8 \
  | UDMA_CHCTL_DSTINC_NONE | UDMA_CHCTL_DSTSIZE_8)


/* SSI receive/transmit data register (R/W) 
  See "SSI_DR" in the CC2538 user manual */
#define CC2538_DW1000_SPI_FIFO_REG (SSI_BASE(DWM1000_SPI_INSTANCE) + SSI_DR)
#define CC2538_DW1000_SPI_DMA_REG  (SSI_BASE(DWM1000_SPI_INSTANCE) + SSI_DMACTL)
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
/* Dummy buffer for the SPI transaction */
uint8_t spi_dummy_buffer = 0;
/*---------------------------------------------------------------------------*/
void
dw1000_int_handler(uint8_t port, uint8_t pin)
{
  /* To keep the gpio_register_callback happy */
  dw1000_driver_interrupt();
}
/*---------------------------------------------------------------------------*/
/**
 * \brief                 Reads or writes a value a buffer to the SPI FIFO using
 *  using DMA. Only useful for "big" buffer (bigger than 8 bytes) because the
 *  configuration of the DMA register and the activation of DMA take some times.
 *
 *  The read or the write buffer can not be use at the same time.
 *  
 *
 * \param[in] read_buf    A pointer to a buffer used to get value from the SPI.
 * \param[in] write_buf   A pointer to a buffer used to send value to the SPI.
 * \param[in] len         The size of the buffer.
 * \param[in] spi_cmd_len The number of previous SPI command send. 
 *                        Used to flush the SPI RX FIFO.
 */
void
dw1000_arch_spi_dma(uint8_t *read_buf, const uint8_t *write_buf, uint16_t len,
                    uint8_t spi_cmd_len)
{
  /* Enable uDMA for the SSI module */
  REG(CC2538_DW1000_SPI_DMA_REG) |= SSI_DMACTL_TXDMAE_M | 
                                    SSI_DMACTL_RXDMAE_M;

  if(write_buf == NULL){
    /* We will do a DMA receive only, we need to write dummy byte (0) in 
      the SPI TX FIFO */
    spi_dummy_buffer = 0;
    for(int i = 0; i < SSI_FIFO_SIZE; i++) {
      SPIX_BUF(DWM1000_SPI_INSTANCE) = 0;
    }

    DW1000_SPI_RX_FLUSH(spi_cmd_len);

    udma_set_channel_src(DW1000_CONF_TX_DMA_SPI_CHAN,
                       (uint32_t)(&spi_dummy_buffer));

    udma_set_channel_control_word(DW1000_CONF_TX_DMA_SPI_CHAN,
                      DW1000_TX_FOR_RX_CRTL_WORD | 
                      udma_xfer_size(len - SSI_FIFO_SIZE) |
                      UDMA_CHCTL_ARBSIZE_4);
  }
  else
  {
    for(int i = 0; i < SSI_FIFO_SIZE; i++) {
      SPIX_BUF(DWM1000_SPI_INSTANCE) = write_buf[i];
    }
    
    DW1000_SPI_RX_FLUSH(spi_cmd_len);

    udma_set_channel_src(DW1000_CONF_TX_DMA_SPI_CHAN,
                      (uint32_t)(write_buf) + len - 1);

    udma_set_channel_control_word(DW1000_CONF_TX_DMA_SPI_CHAN,
                      DW1000_TX_CRTL_WORD | 
                      udma_xfer_size(len - SSI_FIFO_SIZE) |
                      UDMA_CHCTL_ARBSIZE_4);
  }
  if(read_buf == NULL){
    /* We don't need the rereaded value: we put these value in a dummy buffer.
      Thanks to the DMA reception, we don't need to flush the SPI buffer 
      at the end of the SPI transfer. */
    udma_set_channel_dst(DW1000_CONF_RX_DMA_SPI_CHAN,
                       (uint32_t)(&spi_dummy_buffer));

    udma_set_channel_control_word(DW1000_CONF_RX_DMA_SPI_CHAN,
                      DW1000_RX_FOR_TX_CRTL_WORD | udma_xfer_size(len) |
                      UDMA_CHCTL_ARBSIZE_4);
  }
  else
  {
    udma_set_channel_dst(DW1000_CONF_RX_DMA_SPI_CHAN,
                      (uint32_t)(read_buf) + len - 1);

    udma_set_channel_control_word(DW1000_CONF_RX_DMA_SPI_CHAN,
                      DW1000_RX_CRTL_WORD | udma_xfer_size(len) |
                      UDMA_CHCTL_ARBSIZE_4);
  }
  /* Enable the UDMA channel's */
  udma_channel_enable(DW1000_CONF_RX_DMA_SPI_CHAN);
  udma_channel_enable(DW1000_CONF_TX_DMA_SPI_CHAN);

  /* Wait for the transfer to complete. */
  do{watchdog_periodic();} while(udma_channel_get_mode(DW1000_CONF_RX_DMA_SPI_CHAN)
         != UDMA_CHCTL_XFERMODE_STOP);

  /* Disable uDMA for the SSI FIFO */
  REG(CC2538_DW1000_SPI_DMA_REG) &= ~(SSI_DMACTL_RXDMAE_M | SSI_DMACTL_TXDMAE_M);

}
/*---------------------------------------------------------------------------*/
void dw_read_subreg(uint32_t reg_addr, uint16_t subreg_addr, 
                    uint16_t subreg_len, uint8_t * read_buf)
{
  uint8_t spi_cmd_len = 1, i;
  if(subreg_len == 0)
  {
    return;
  }

  /* We write the SPI commands on the SPI TX FIFO and we flush the SPI RX FIFO 
    later (there are a delay between the reception of the byte on the physical 
    SPI and the availability of the byte on the RX FIFO). */

  DW1000_SELECT();

  /* write bit = 0, sub-reg present bit = 1 */
  SPIX_BUF(DWM1000_SPI_INSTANCE) = ((subreg_addr > 0?0x40:0x00) | 
                        (reg_addr & 0x3F));
  if (subreg_addr > 0) {
    if (subreg_addr > 0x7F) {
      /* extended address bit = 1 */
      SPIX_BUF(DWM1000_SPI_INSTANCE) = (0x80 | (subreg_addr & 0x7F));
      SPIX_BUF(DWM1000_SPI_INSTANCE) = ((subreg_addr >> 7) & 0xFF);
      spi_cmd_len = 3;
    } else {
      /* extended address bit = 0 */
      SPIX_BUF(DWM1000_SPI_INSTANCE) = (subreg_addr & 0x7F);
      spi_cmd_len = 2;
    }
  }

  if(DW1000_ARCH_CONF_DMA && subreg_len > SSI_FIFO_SIZE)
  { /* We will do a uDMA transfer */
    dw1000_arch_spi_dma(read_buf, NULL, subreg_len, spi_cmd_len);
  }
  else if(subreg_len > spi_cmd_len)
  {
    for(i = 0; i < spi_cmd_len ; i++) {
      SPIX_BUF(DWM1000_SPI_INSTANCE) = 0;
      
      SPIX_WAITFOREORx(DWM1000_SPI_INSTANCE); /* RX FIFO is not empty */
      /* read the receive FIFO to clear it*/
      SPIX_BUF(DWM1000_SPI_INSTANCE);
    }

    for(i = 0; i < subreg_len-spi_cmd_len; i++) {
      SPIX_BUF(DWM1000_SPI_INSTANCE) = 0;
      
      SPIX_WAITFOREORx(DWM1000_SPI_INSTANCE); /* RX FIFO is not empty */
      read_buf[i] = SPIX_BUF(DWM1000_SPI_INSTANCE);
    }

    for(i = subreg_len-spi_cmd_len; i < subreg_len; i++) {
      SPIX_WAITFOREORx(DWM1000_SPI_INSTANCE); /* RX FIFO is not empty */
      /* read the receive FIFO to clear it*/
      read_buf[i] = SPIX_BUF(DWM1000_SPI_INSTANCE);
    }
  }
  else{
    for(i = 0; i < subreg_len ; i++) {
      SPIX_WAITFORTxREADY(DWM1000_SPI_INSTANCE); /* TX FIFO is not full */
      SPIX_BUF(DWM1000_SPI_INSTANCE) = 0;
    }

    DW1000_SPI_RX_FLUSH(spi_cmd_len);

    for(i = 0; i < subreg_len; i++) {
      SPIX_WAITFOREORx(DWM1000_SPI_INSTANCE); /* RX FIFO is not empty */
      /* read the receive FIFO to clear it*/
      read_buf[i] = SPIX_BUF(DWM1000_SPI_INSTANCE);
    }
  }
  DW1000_DESELECT();
}
/*---------------------------------------------------------------------------*/
void dw_write_subreg(uint32_t reg_addr, uint16_t subreg_addr, 
                      uint16_t subreg_len, const uint8_t *write_buf)
{
  uint8_t spi_cmd_len = 1, i;

  if(subreg_len == 0)
  {
    return;
  }

  /* We write the SPI commands on the SPI TX FIFO and we flush the SPI RX FIFO 
    later (there are a delay between the reception of the byte on the physical 
    SPI and the availability of the byte on the RX FIFO). */

  DW1000_SELECT(); 

  /* write bit = 1, sub-reg present bit = 1 */
  SPIX_BUF(DWM1000_SPI_INSTANCE) = (0x80 | (subreg_addr > 0 ?0x40:0x00) |
                       (reg_addr & 0x3F));
  if (subreg_addr > 0) {
    if (subreg_addr > 0x7F) {
      /* extended address bit = 1 */
      SPIX_BUF(DWM1000_SPI_INSTANCE) = (0x80 | (subreg_addr & 0x7F));
      SPIX_BUF(DWM1000_SPI_INSTANCE) = ((subreg_addr >> 7) & 0xFF);
      spi_cmd_len = 3;
    } else {
      /* extended address bit = 0 */
      SPIX_BUF(DWM1000_SPI_INSTANCE) = (subreg_addr & 0x7F);
      spi_cmd_len = 2;
    }
  }
  
  if(DW1000_ARCH_CONF_DMA && subreg_len > SSI_FIFO_SIZE)
  { /* We will do a uDMA transfer */
    dw1000_arch_spi_dma(NULL, write_buf, subreg_len, spi_cmd_len);
  }
  else if(subreg_len + spi_cmd_len > SPI_FIFO_SIZE)
  {
    /* We fill the SPI TX FIFO, Flush the half of the TX FIFO 
      and send the last bytes */
    for(i = 0; i < subreg_len-spi_cmd_len; i++) {
      SPIX_BUF(DWM1000_SPI_INSTANCE) = write_buf[i];
    }

    DW1000_SPI_RX_FLUSH(SPI_FIFO_SIZE/2);

    for(i = subreg_len-spi_cmd_len; i < subreg_len; i++) {
      SPIX_BUF(DWM1000_SPI_INSTANCE) = write_buf[i];
    }

    DW1000_SPI_RX_FLUSH(subreg_len+spi_cmd_len-(SPI_FIFO_SIZE/2));
  }
  else{
    for(i = 0; i < subreg_len; i++) {
      SPIX_BUF(DWM1000_SPI_INSTANCE) = write_buf[i];
    }

    DW1000_SPI_RX_FLUSH(subreg_len+spi_cmd_len);
  }
  DW1000_DESELECT();
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
/** Initialize the RESET PIN in INPUT and open drain */
void
dw1000_arch_init_reset_pin(void)
{
  GPIO_SOFTWARE_CONTROL(DWM1000_RESET_PORT_BASE, DWM1000_RESET_PIN_MASK);
  GPIO_SET_INPUT(DWM1000_RESET_PORT_BASE, DWM1000_RESET_PIN_MASK);
  ioc_set_over(DWM1000_RESET_PORT, DWM1000_RESET_PIN, IOC_OVERRIDE_OE);}
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

  /* Change the SPI configuration to CPOL = 0, CPHA = 0 
    clock_polarity = 0 : low when IDLE 
    clock_phase = 0 data is valid on the clock leading edge*/
  spix_set_mode(DWM1000_SPI_INSTANCE, SSI_CR0_FRF_MOTOROLA, 0, 0, 8);

  /* Leave CSn as default */
  DW1000_DESELECT();

  /* Ensure MISO is low */
  // BUSYWAIT_UNTIL(
  //   (GPIO_READ_PIN(DWM1000_SPI_MISO_PORT_BASE, DWM1000_SPI_MISO_PIN_MASK) == 0),
  //   RTIMER_SECOND / 10);

  if(DW1000_ARCH_CONF_DMA) {
    /* Set the channel to the assignment 1 according to the Table 10-1. 
      "μDMA Channel Assignments" */
    udma_set_channel_assignment(DW1000_CONF_RX_DMA_SPI_CHAN, 
                              DW1000_CONF_RX_DMA_SPI_ENC);
    udma_set_channel_assignment(DW1000_CONF_TX_DMA_SPI_CHAN, 
                              DW1000_CONF_TX_DMA_SPI_ENC);

    /* Set the channel's end DST : The SPI FIFO register. */
    udma_set_channel_src(DW1000_CONF_RX_DMA_SPI_CHAN, 
                              CC2538_DW1000_SPI_FIFO_REG);
    udma_set_channel_dst(DW1000_CONF_TX_DMA_SPI_CHAN, 
                              CC2538_DW1000_SPI_FIFO_REG);

    /* Enable peripheral triggers */
    udma_channel_mask_clr(DW1000_CONF_RX_DMA_SPI_CHAN);
    udma_channel_mask_clr(DW1000_CONF_TX_DMA_SPI_CHAN);
  }
  ioc_init(); /* Needed to disable pull up  in deepsleep*/

  /* init the wake up port in OUTPUT, LOW, with PULL DOWN resistor */
  GPIO_SET_OUTPUT(DWM1000_WAKEUP_PORT_BASE, DWM1000_WAKEUP_PIN_MASK);
  ioc_set_over(DWM1000_WAKEUP_PORT, DWM1000_WAKEUP_PIN, IOC_OVERRIDE_PDE);
  GPIO_CLR_PIN(DWM1000_WAKEUP_PORT_BASE, DWM1000_WAKEUP_PIN_MASK);

  /* Performe a wake up in case of the node was in deepsleept before being restarted */
  NETSTACK_RADIO.set_value(RADIO_SLEEP_STATE, RADIO_REQUEST_WAKEUP);
  dw1000_us_delay(4000);      
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
 * If freq is bigger than the maximum SPI frequency value of the embedded 
 * system set this maximum value.
 **/
void dw1000_arch_spi_set_clock_freq(uint32_t freq){
  spix_set_clock_freq(DWM1000_SPI_INSTANCE, freq);
}
/**
 * Configure the embedeed system to allow the transceiver to go in 
 * DEEPSLEEP state. 
 * Disable SPICLK and SPIMISO to avoid leakeage current.
 **/
void dw1000_arch_init_deepsleep(void){

  /* disable SPI clock */
  // spix_disable(DWM1000_SPI_INSTANCE);

  GPIO_SOFTWARE_CONTROL(DWM1000_SPI_CLK_PORT_BASE, DWM1000_SPI_CLK_PIN_MASK);
  GPIO_SOFTWARE_CONTROL(DWM1000_SPI_MISO_PORT_BASE, DWM1000_SPI_MISO_PIN_MASK);
  // GPIO_SOFTWARE_CONTROL(DWM1000_SPI_MOSI_PORT_BASE, DWM1000_SPI_MOSI_PIN_MASK);

  /* disable Pull up resistance */
  ioc_set_over(DWM1000_CLK_PORT, DWM1000_CLK_PIN, IOC_OVERRIDE_DIS);
  ioc_set_over(DWM1000_MISO_PORT, DWM1000_MISO_PIN, IOC_OVERRIDE_DIS);
  // ioc_set_over(DWM1000_MOSI_PORT, DWM1000_MOSI_PIN, IOC_OVERRIDE_DIS);
  ioc_set_over(DWM1000_INT_PORT, DWM1000_INT_PIN, IOC_OVERRIDE_DIS);
  /* enable pull up resistance */
  ioc_set_over(DWM1000_SPI_CSN_PORT, DWM1000_SPI_CSN_PIN, IOC_OVERRIDE_PUE);

  /* set in input */
  GPIO_SET_INPUT(DWM1000_SPI_CLK_PORT_BASE, DWM1000_SPI_CLK_PIN_MASK);
  GPIO_SET_INPUT(DWM1000_SPI_MISO_PORT_BASE, DWM1000_SPI_MISO_PIN_MASK);
  // GPIO_SET_INPUT(DWM1000_SPI_MOSI_PORT_BASE, DWM1000_SPI_MOSI_PIN_MASK);
  // GPIO_SET_INPUT(DWM1000_INT_PORT_BASE, DWM1000_INT_PIN_MASK);


  /* drive CSn HIGH */
  DW1000_DESELECT();
}
/**
 * Used in DEEPSLEEP state. 
 * You need to drive HIGH the port WAKEUP or LOW the CSN port for min 500µs 
 * to wakeup the transceiver.
 * You can check if thew transceiver have wakeup by reading the GPIO8 state 
 * GPIO8 is drive HIGH when the transceiver comes in the IDLE state 
 * It take approx 3ms to go out of the deepsleep state.
 **/
void dw1000_arch_wake_up(dw1000_pin_state state){
  if(state == DW1000_PIN_ENABLE){
    // DW1000_SELECT();
    /*  wake up port in OUTPUT, HIGH, with PULL UP resistor */
    ioc_set_over(DWM1000_WAKEUP_PORT, DWM1000_WAKEUP_PIN, IOC_OVERRIDE_PUE);
    GPIO_SET_PIN(DWM1000_WAKEUP_PORT_BASE, DWM1000_WAKEUP_PIN_MASK);
  } else{ 
    // DW1000_DESELECT();
    /*  wake up port in OUTPUT, LOW, with PULL DOWN resistor */
    ioc_set_over(DWM1000_WAKEUP_PORT, DWM1000_WAKEUP_PIN, IOC_OVERRIDE_PDE);
    GPIO_CLR_PIN(DWM1000_WAKEUP_PORT_BASE, DWM1000_WAKEUP_PIN_MASK);
  }
}
/**
 * Configure the embedded system to interact with the transceiver. 
 * Enable SPICLK and SPIMISO.
 **/
void dw1000_arch_restore_idle_state(void){

  /* Enable Pull up resistance (default state) */
  ioc_set_over(DWM1000_CLK_PORT, DWM1000_CLK_PIN, IOC_OVERRIDE_PUE);
  ioc_set_over(DWM1000_MISO_PORT, DWM1000_MISO_PIN, IOC_OVERRIDE_PUE);
  // ioc_set_over(DWM1000_MOSI_PORT, DWM1000_MOSI_PIN, IOC_OVERRIDE_PUE);
  ioc_set_over(DWM1000_INT_PORT, DWM1000_INT_PIN, IOC_OVERRIDE_PUE);

  /* Disable pull up resistance */
  ioc_set_over(DWM1000_SPI_CSN_PORT, DWM1000_SPI_CSN_PIN, IOC_OVERRIDE_DIS);


  /* set pin in output */
  GPIO_SET_OUTPUT(DWM1000_SPI_CLK_PORT_BASE, DWM1000_SPI_CLK_PIN_MASK);
  GPIO_SET_OUTPUT(DWM1000_SPI_MISO_PORT_BASE, DWM1000_SPI_MISO_PIN_MASK);
  // GPIO_SET_OUTPUT(DWM1000_SPI_MOSI_PORT_BASE, DWM1000_SPI_MOSI_PIN_MASK);
  
  GPIO_PERIPHERAL_CONTROL(DWM1000_SPI_CLK_PORT_BASE, DWM1000_SPI_CLK_PIN_MASK);
  GPIO_PERIPHERAL_CONTROL(DWM1000_SPI_MISO_PORT_BASE, DWM1000_SPI_MISO_PIN_MASK);
  // GPIO_PERIPHERAL_CONTROL(DWM1000_SPI_MOSI_PORT_BASE, DWM1000_SPI_MOSI_PIN_MASK);

  /* drive CSn Low */
  // DW1000_DESELECT();

  /* re-enable SPI clock */
  // spix_enable(DWM1000_SPI_INSTANCE);
  // dw1000_arch_init();
}


/*---------------------------------------------------------------------------*/
/**
 * \brief Read the 4096 bytes of the Accumulator CIR memory.
 * */
void dw_read_CIR(uint8_t * read_buf)
{

  // printf("dw_read_CIR \n");
  uint8_t reg_addr = DW_REG_ACC_MEM;
  uint16_t subreg_len = DW_LEN_ACC_MEM, i;

  DW1000_SELECT();

  /* We write the SPI commands on the SPI TX FIFO and we flush the SPI RX FIFO 
    later (there are a delay between the reception of the byte on the physical 
    SPI and the availability of the byte on the RX FIFO). */
  SPIX_BUF(DWM1000_SPI_INSTANCE) = reg_addr & 0x3F;
  DW1000_SPI_RX_FLUSH(1);

  for(i = 0; i < subreg_len; i++) {
    SPIX_BUF(DWM1000_SPI_INSTANCE) = 0;
    
    SPIX_WAITFOREORx(DWM1000_SPI_INSTANCE); /* RX FIFO is not empty */
    read_buf[i] = SPIX_BUF(DWM1000_SPI_INSTANCE);

    watchdog_periodic(); /* avoid watchdog timer to be reach */
  }

  DW1000_DESELECT();
}