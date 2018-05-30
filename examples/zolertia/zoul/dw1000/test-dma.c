#include "contiki.h"
#include "net/rime/rime.h"
#include "dev/serial-line.h"
#include <stdio.h>
#include <stdlib.h>
#include "lib/random.h"
#include "net/netstack.h"

#include "reg.h"
#include "spi-arch.h"
#include "dev/ioc.h"
#include "dev/sys-ctrl.h"
#include "dev/spi.h"
#include "dev/ssi.h"
#include "dev/gpio.h" 
#include "dev/udma.h"
#include "watchdog.h"

#include "dw1000.h"
#include "dw1000-arch.h"

// #define DEBUG 1

// #undef DW1000_CONF_TX_DMA_SPI_CHAN
// #define DW1000_CONF_TX_DMA_SPI_CHAN 6

/* Channel choose according to the Table 10-1. μDMA Channel Assignments of the 
  CC2538 User Manual */
#define DW1000_CONF_TX_DMA_SPI_ENC   UDMA_CH11_SSI1TX

/* Configure the channel with 8 bit source and destination size,
 * 8 source increment and 0 bit destination increment, one would need to pass */
#define DW1000_TX_CRTL_WORD (UDMA_CHCTL_XFERMODE_BASIC  \
  | UDMA_CHCTL_SRCINC_8 | UDMA_CHCTL_SRCSIZE_8 \
  | UDMA_CHCTL_DSTINC_NONE | UDMA_CHCTL_DSTSIZE_8)

#define WRITE_BUFFER        DW_REG_TX_BUFFER 

#define DW1000_CONF_RX_DMA_SPI_ENC   UDMA_CH10_SSI1RX
/* SSI receive/transmit data register (R/W) 
  See "SSI_DR" in the CC2538 user manual */
#define CC2538_DW1000_SPI_FIFO_REG    (SSI_BASE(DWM1000_SPI_INSTANCE) + SSI_DR)
#define CC2538_DW1000_SPI_DMA_REG     (SSI_BASE(DWM1000_SPI_INSTANCE) + SSI_DMACTL)
/* Configure the channel with 8 bit source and destination size,
 * 0 source increment and 8 bit destination increment, one would need to pass */
#define DW1000_RX_CRTL_WORD (UDMA_CHCTL_XFERMODE_AUTO  \
  | UDMA_CHCTL_SRCINC_NONE | UDMA_CHCTL_SRCSIZE_8 \
  | UDMA_CHCTL_DSTINC_NONE | UDMA_CHCTL_DSTSIZE_8)



#define DMA_CHAN_5 5
#define DMA_CHAN_5_ENC 0
#define DMA_CHAN_5_CRTL_WORD (UDMA_CHCTL_ARBSIZE_512 | UDMA_CHCTL_XFERMODE_BASIC \
  | UDMA_CHCTL_DSTINC_8 | UDMA_CHCTL_DSTSIZE_8\
  | UDMA_CHCTL_SRCINC_8 | UDMA_CHCTL_SRCSIZE_8)
/*---------------------------------------------------------------------------*/
PROCESS(dw1000_dma, "Frame master");

AUTOSTART_PROCESSES(&dw1000_dma);
/*---------------------------------------------------------------------------*/

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while(0)
#endif

#define TIME_PORT     GPIO_A_NUM
#define TIME_PIN      2

void init_gpio(void);
void gpio_up(void);
void gpio_down(void);


PROCESS_THREAD(dw1000_dma, ev, data)
{
  PROCESS_BEGIN();

  // init_gpio();

  random_init(2018);

#ifndef RADIO_DRIVER_UWB
  /* initialize SPI 1 */
  dw1000_arch_init();
#endif

  spix_cs_init(TIME_PORT, TIME_PIN);

  printf("Node addr 0x%02X%02X\n", 
                      linkaddr_node_addr.u8[1], 
                      linkaddr_node_addr.u8[0]);
  printf("MACRO value\n");
  printf("CC2538_DW1000_SPI_DMA_REG 0x%08X\n", CC2538_DW1000_SPI_DMA_REG);
  printf("SSI_DMACTL_TXDMAE_M 0x%08X\n", SSI_DMACTL_TXDMAE_M);
  printf("UDMA_CHCTL_XFERMODE_STOP 0x%08X\n", UDMA_CHCTL_XFERMODE_STOP);
  printf("CC2538_DW1000_SPI_FIFO_REG 0x%08X\n", CC2538_DW1000_SPI_FIFO_REG);
  printf("DW1000_CONF_TX_DMA_SPI_CHAN 0x%08X\n", DW1000_CONF_TX_DMA_SPI_CHAN);
  printf("DW1000_CONF_TX_DMA_SPI_ENC 0x%08X\n", DW1000_CONF_TX_DMA_SPI_ENC);
  printf("DW1000_TX_CRTL_WORD 0x%08X\n", DW1000_TX_CRTL_WORD);

  printf("     [0, DMA, n] write n random byte in the RX buffer using DMA (MDA= 1) or simple SPI (DMA = 0)\n");
  printf("     [1, n] read n byte in the RX buffer\n");

  // dw1000_arch_spi_set_clock_freq(DW_SPI_CLOCK_FREQ_INIT_STATE);

  /* According 8.3.3 DMA Flash Write, "High priority should also be ensured for
    the DMA channel, so it is not interrupted in the write process. If 
    interrupted for more than 20 μs, the write operation may time out, 
    and the write bit, FCTL.WRITE, is set to 0." */
  udma_channel_prio_set_high(DW1000_CONF_TX_DMA_SPI_CHAN);

  /* Set the channel to the assignment 1 according to the Table 10-1. 
    "μDMA Channel Assignments */
  udma_set_channel_assignment(DW1000_CONF_TX_DMA_SPI_CHAN, 
                            DW1000_CONF_TX_DMA_SPI_ENC);

  /*
   * Set the channel's end DST : The SPI FIFO register.
   */
  udma_set_channel_dst(DW1000_CONF_TX_DMA_SPI_CHAN, 
                            CC2538_DW1000_SPI_FIFO_REG);
  for(;;) {
    PROCESS_YIELD();
    if(ev == serial_line_event_message) {
      /* we convert the input string data to tow int using strlol see 
      https://www.tutorialspoint.com/c_standard_library/c_function_strtol.htm */
      char * str;
      uint16_t mode = strtol(data, &str, 16);

      printf("Received line: %s\n", (char *)data);

      if(mode == 0x0 ){ 
        uint16_t dma_enable = strtol(str, &str, 16);
        uint16_t nb_bytes = strtol(str, &str, 16);
        /* generate the random bytes tab */
        uint8_t random_bytes[1025];
        uint8_t rx_trash_buffer;
        uint16_t write_spi_len = nb_bytes + 1;

        /* write SPI instruction */
        random_bytes[0] = 0x80 | (WRITE_BUFFER & 0x3F);

        printf("Random tab %u: ", nb_bytes);
        for(int i = 1; i < nb_bytes+1; i++){
          random_bytes[i] = i;
        }
        for(int i = 1; i < nb_bytes+1; i++){
          printf("%u ", random_bytes[i]);
          watchdog_periodic();
        }
        printf("\n");

        if(dma_enable){
          printf("Copies DMA\n");

          gpio_down();
          /* Disable the TX uDMA channel before his configuration */
          // udma_channel_disable(DW1000_CONF_TX_DMA_SPI_CHAN);
          // udma_channel_disable(DW1000_CONF_RX_DMA_SPI_CHAN);


          /* Enable uDMA for the transmit SSI FIFO */
          REG(CC2538_DW1000_SPI_DMA_REG) |= SSI_DMACTL_TXDMAE_M;

          // spix_enable(DWM1000_SPI_INSTANCE);


          /* Disable peripheral triggers for the channel and enable software usage of
          the channel.*/
          // udma_channel_mask_set(DW1000_CONF_TX_DMA_SPI_CHAN);


          /* Set the transfer source's end address */
          udma_set_channel_src(DW1000_CONF_TX_DMA_SPI_CHAN,
                               (uint32_t)(&random_bytes[0]) + write_spi_len - 1);




          /*
           * Use the primary DMA structure (the secondary structure doesn't exist in contiki).
           */
          // udma_channel_use_primary(DW1000_CONF_TX_DMA_SPI_CHAN);
          // printf("write buf start %08X, end %08X\n", (uint32_t)(write_buf), (uint32_t)(write_buf) + len - 1);


          /* Configure the control word 
            In burst mode the arbitration size should be the same as 
            the number of byte to transfer see 10.3.4.2 Burst Request*/
          udma_set_channel_control_word(DW1000_CONF_TX_DMA_SPI_CHAN,
                                  DW1000_TX_CRTL_WORD | udma_xfer_size(write_spi_len) |
                                  // udma_arb_size(write_spi_len));
                                  UDMA_CHCTL_ARBSIZE_4);

          // udma_channel_use_single(DW1000_CONF_TX_DMA_SPI_CHAN);

          /* SPI support burst of 4 bytes according to the Table 10-2. 
            Request Type Support */
          // udma_channel_use_burst(DW1000_CONF_TX_DMA_SPI_CHAN);
          // printf("REG(CC2538_DW1000_SPI_FIFO_REG) 0X%08X\n", REG(CC2538_DW1000_SPI_FIFO_REG));


          // printf("Control word : 0X%08lX, length %lu\n",
          //     udma_channel_get_control_word(DW1000_CONF_TX_DMA_SPI_CHAN),
          //     ((udma_channel_get_control_word(DW1000_CONF_TX_DMA_SPI_CHAN)>>4) & (0XFF)) +1);
          
          // SPIX_FLUSH(DWM1000_SPI_INSTANCE);

          /* Enabled the RF TX uDMA channel */
          udma_channel_enable(DW1000_CONF_TX_DMA_SPI_CHAN);

          dw1000_arch_spi_select(); 

          /* Trigger the uDMA transfer */
          udma_channel_sw_request(DW1000_CONF_TX_DMA_SPI_CHAN);

          // printf("Control word : 0X%08lX, length %lu\n",
          //     udma_channel_get_control_word(DW1000_CONF_RX_DMA_SPI_CHAN),
          //     ((udma_channel_get_control_word(DW1000_CONF_RX_DMA_SPI_CHAN)>>4) & (0XFF)) +1);

          // uint32_t result = udma_channel_get_mode(DW1000_CONF_TX_DMA_SPI_CHAN);
          // printf("max channel %d\n", UDMA_CONF_MAX_CHANNEL);
          // printf("udma_channel_get_mode(DW1000_CONF_TX_DMA_SPI_CHAN) %08X\n", result);
          // printf("REG(CC2538_DW1000_SPI_FIFO_REG) 0X%08X\n", REG(CC2538_DW1000_SPI_FIFO_REG));

          /* Wait for the transfer to complete. */
          while(udma_channel_get_mode(DW1000_CONF_TX_DMA_SPI_CHAN)
                 != UDMA_CHCTL_XFERMODE_STOP);

          // /* Wait for the transfer to complete. */
          // for(int i=0; i< write_spi_len; i++){
          //   while(udma_channel_get_mode(DW1000_CONF_TX_DMA_SPI_CHAN)
          //          != UDMA_CHCTL_XFERMODE_STOP);
          //   udma_channel_sw_request(DW1000_CONF_TX_DMA_SPI_CHAN);
          // }

          // printf("Control word : 0X%08lX, length %lu\n",
          //     udma_channel_get_control_word(DW1000_CONF_TX_DMA_SPI_CHAN),
          //     ((udma_channel_get_control_word(DW1000_CONF_TX_DMA_SPI_CHAN)>>4) & (0XFF)) +1);

          // clock_delay_usec(100);

          SPIX_FLUSH(DWM1000_SPI_INSTANCE);

          /* Disable uDMA for the transmit SSI FIFO */
          REG(CC2538_DW1000_SPI_DMA_REG) &= ~(SSI_DMACTL_TXDMAE_M);


          dw1000_arch_spi_deselect();       
        } else
        {
          printf("Copies SPI\n");

          gpio_down();
          dw_write_reg(WRITE_BUFFER, nb_bytes, (uint8_t *) &random_bytes[1]);
        }
        gpio_up();
      }
      else if(mode == 0x01){ 
        uint16_t nb_bytes = strtol(str, &str, 16);
        /* generate the random bytes tab */
        uint8_t random_bytes[1023];

        gpio_down();
        dw_read_reg(WRITE_BUFFER, nb_bytes, (uint8_t *) &random_bytes[0]);
        gpio_up();

        printf("WRITE_BUFFER %u : ", nb_bytes);
        for(int i = 0; i < nb_bytes; i++){
          printf("%u ", random_bytes[i]);
          watchdog_periodic();
        }
        printf("\n");
      }
      else if(mode == 0x02){ 
        uint16_t nb_bytes = strtol(str, &str, 16);
        /* generate the random bytes tab */
        uint8_t random_bytes[1023];
        printf("Change the value of the WRITE_BUFFER with random value.\n");
        printf("WRITE_BUFFER %u: ", nb_bytes);
        for(int i = 0; i < nb_bytes; i++){
          random_bytes[i] = rand();
        }
        for(int i = 0; i < nb_bytes; i++){
          printf("%u ", random_bytes[i]);
          watchdog_periodic();
        }
        printf("\n");

        gpio_down();
        dw_write_reg(WRITE_BUFFER, nb_bytes, (uint8_t *) &random_bytes[0]);
        gpio_up();

      }
      else if(mode == 0x3){ 
        printf("0X2\n");
        uint16_t nb_bytes = strtol(str, &str, 16);
        /* generate the random bytes tab */
        uint8_t random_bytes[512];
        uint8_t random_bytes_dest[512];

        printf("Random tab %u: ", nb_bytes);
        for(int i = 0; i < nb_bytes; i++){
          random_bytes[i] = random_rand();
        }
        for(int i = 0; i < nb_bytes; i++){
          printf("%u ", random_bytes[i]);
          watchdog_periodic();
        }
        printf("\n");

        gpio_down();
        /* Disable peripheral triggers for the channel and enable software usage of
        the channel.*/
        udma_channel_mask_set(DMA_CHAN_5);

        /* set the encoding param for the channel */
        // udma_set_channel_assignment(DMA_CHAN_5, DMA_CHAN_5_ENC);

        /* Set the transfer source's end address */
        udma_set_channel_src(DMA_CHAN_5, (uint32_t)(&random_bytes[0]) + nb_bytes - 1);
        printf("SRC ADDR 0X%08lX\n", (uint32_t)(&random_bytes[0]) + nb_bytes - 1);
        printf("SRC ADDR %ld\n", (uint32_t)(&random_bytes[0]) + nb_bytes - 1);
        /*
        * Set the channel's end DST
        */
        udma_set_channel_dst(DMA_CHAN_5, (uint32_t)(&random_bytes_dest[0]) + nb_bytes - 1);
        printf("DST ADDR 0X%08lX\n", (uint32_t)(&random_bytes_dest[0]) + nb_bytes - 1);
        printf("DST ADDR %ld\n", (uint32_t)(&random_bytes_dest[0]) + nb_bytes - 1);

        /*
        * Use the primary DMA structure (the secondary structure doesn't exist in contiki).
        */
        // udma_channel_use_primary(DMA_CHAN_5);
        // printf("write buf start %08X, end %08X\n", (uint32_t)(write_buf), (uint32_t)(write_buf) + len - 1);


        /* Configure the control word 
        In burst mode the arbitration size should be the same as 
        the number of byte to transfer see 10.3.4.2 Burst Request*/
        udma_set_channel_control_word(DMA_CHAN_5,
                            DMA_CHAN_5_CRTL_WORD | udma_xfer_size(nb_bytes));
                            // UDMA_CHCTL_ARBSIZE_1024);
                            // udma_arb_size(len));

        // udma_channel_use_burst(DMA_CHAN_5);
        // printf("REG(CC2538_DW1000_SPI_FIFO_REG) 0X%08X\n", REG(CC2538_DW1000_SPI_FIFO_REG));

        /* Enabled the RF TX uDMA channel */
        udma_channel_enable(DMA_CHAN_5);


        /* Trigger the uDMA transfer */
        udma_channel_sw_request(DMA_CHAN_5);

        uint32_t result = udma_channel_get_mode(DMA_CHAN_5);
        // printf("max channel %d\n", UDMA_CONF_MAX_CHANNEL);
        // printf("udma_channel_get_mode(DMA_CHAN_5) %08X\n", result);
        // printf("REG(CC2538_DW1000_SPI_FIFO_REG) 0X%08X\n", REG(CC2538_DW1000_SPI_FIFO_REG));
        /* Wait for the transfer to complete. */
        while(udma_channel_get_mode(DMA_CHAN_5)
             != UDMA_CHCTL_XFERMODE_STOP);

        gpio_up();

        printf("udma_channel_get_mode(DMA_CHAN_5) %08lX\n", result);

        printf("Random tab dest %u: ", nb_bytes);
        for(int i = 0; i < nb_bytes; i++){
          printf("%u ", random_bytes_dest[i]);
          watchdog_periodic();
        }
        printf("\n");
      }
    }
  }
  printf("end\n");
  PROCESS_END();
}


void 
init_gpio(void)
{
  GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(TIME_PORT),
                        GPIO_PIN_MASK(TIME_PORT));
  ioc_set_over(TIME_PORT, TIME_PIN, IOC_OVERRIDE_DIS);
  GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(TIME_PORT), GPIO_PIN_MASK(TIME_PIN));
  GPIO_CLR_PIN(GPIO_PORT_TO_BASE(TIME_PORT), GPIO_PIN_MASK(TIME_PIN));
}

void
gpio_up(void)
{
  GPIO_SET_PIN(GPIO_PORT_TO_BASE(TIME_PORT), GPIO_PIN_MASK(TIME_PIN));
}
void
gpio_down(void)
{
  GPIO_CLR_PIN(GPIO_PORT_TO_BASE(TIME_PORT), GPIO_PIN_MASK(TIME_PIN));
}