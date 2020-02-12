/*
 * Copyright (c) 2017, Charlier Maximilien, UMons University.
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
 *         Interface for the driver of the Decawave DW1000 on Contiki.
 *          Based on the CC2420 driver.
 *
 * \author
 *         Charlier Maximilien  <maximilien-charlier@outlook.com>
 */

#include <string.h>

#include "contiki.h"

#if defined(__AVR__)
#include <avr/io.h>
#endif
#include "dw1000.h"
#include "dw1000-driver.h"
#include "dw1000-arch.h"
#include "dw1000-util.h"
#include "dw1000-const.h"

#include "net/packetbuf.h"
#include "net/rime/rimestats.h"
#include "net/netstack.h"

#include "watchdog.h"
#include "assert.h"


#include <stdio.h>



#ifndef DW1000_CONF_CHECKSUM
#define DW1000_CONF_CHECKSUM    1
#endif /* DW1000_CONF_CHECKSUM */

#ifndef DW1000_CONF_AUTOACK
#define DW1000_CONF_AUTOACK     1
#endif /* DW1000_CONF_AUTOACK */

#define FOOTER_LEN                2

#ifndef DW1000_CHANNEL
#define DW1000_CHANNEL          5
#endif /* DW1000_CHANNEL */

#ifndef DW1000_DATA_RATE
#define DW1000_DATA_RATE         DW_DATA_RATE_6800_KBPS
#endif /* DW1000_DATA_RATE */

#ifndef DW1000_PREAMBLE
#define DW1000_PREAMBLE          DW_PREAMBLE_LENGTH_128
#endif /* DW1000_PREAMBLE */

#ifndef DW1000_PRF
#define DW1000_PRF               DW_PRF_64_MHZ
#endif /* DW1000_PRF */

#ifndef DW1000_TSCH
#define DW1000_TSCH     1
#endif /* DW1000_TSCH */

#ifndef RADIO_DELAY_MEASUREMENT
#define RADIO_DELAY_MEASUREMENT 0
#endif /* RADIO_DELAY_MEASUREMENT */

/* You should disable the ranging bias in case of antenna delay calibration */
#define DW1000_ENABLE_RANGING_BIAS_CORRECTION 1
#if DW1000_ENABLE_RANGING_BIAS_CORRECTION
  #include "dw1000-ranging-bias.h"
#endif /* DW1000_ENABLE_RANGING_BIAS_CORRECTION */

/* the delay induced by the SPI communication */
#define DW1000_SPI_DELAY        50l 


#if DW1000_IEEE802154_EXTENDED
#define DW1000_MAX_PACKET_LEN 265
#else
#define DW1000_MAX_PACKET_LEN 127
#endif

/* #define DEBUG_RANGING 1 */

// #define DEBUG 1
#define DEBUG_VERBOSE 0
#if DEBUG_VERBOSE
#define DEBUG 1
#endif
#ifndef DEBUG
#define DEBUG 0
#endif
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while(0)
#endif

#define DEBUG_LED 1

// #define DEBUG_RANGING_STATE
#ifdef DEBUG_RANGING_STATE
#define RANGING_STATE(...) printf(__VA_ARGS__)
#else
#define RANGING_STATE(...) do {} while(0)
#endif


/* Just for the debugging of TSCH */
#define DEBUG_GPIO_TSCH 1
#ifdef DEBUG_GPIO_TSCH
  #include "dev/gpio.h"

  #define DWM1000_LISTEN_PORT           GPIO_D_NUM
  #define DWM1000_LISTEN_PIN            2
  #define LISTEN_CLR() do { \
      GPIO_CLR_PIN(GPIO_PORT_TO_BASE(DWM1000_LISTEN_PORT), GPIO_PIN_MASK(DWM1000_LISTEN_PIN)); \
  } while(0)
  #define LISTEN_SET() do { \
      GPIO_SET_PIN(GPIO_PORT_TO_BASE(DWM1000_LISTEN_PORT), GPIO_PIN_MASK(DWM1000_LISTEN_PIN)); \
  } while(0)

  #define DWM1000_SLEEP_PORT           GPIO_A_NUM
  #define DWM1000_SLEEP_PIN            2
  #define SLEEP_CLR() do { \
      GPIO_CLR_PIN(GPIO_PORT_TO_BASE(DWM1000_SLEEP_PORT), GPIO_PIN_MASK(DWM1000_SLEEP_PIN)); \
  } while(0)
  #define SLEEP_SET() do { \
      GPIO_SET_PIN(GPIO_PORT_TO_BASE(DWM1000_SLEEP_PORT), GPIO_PIN_MASK(DWM1000_SLEEP_PIN)); \
  } while(0)
  #define DWM1000_SEND_PORT             GPIO_D_NUM
  #define DWM1000_SEND_PIN              0
  #define SEND_CLR() do { \
      GPIO_CLR_PIN(GPIO_PORT_TO_BASE(DWM1000_SEND_PORT), GPIO_PIN_MASK(DWM1000_SEND_PIN)); \
  } while(0)
  #define SEND_SET() do { \
      GPIO_SET_PIN(GPIO_PORT_TO_BASE(DWM1000_SEND_PORT), GPIO_PIN_MASK(DWM1000_SEND_PIN)); \
  } while(0)
  #define INIT_GPIO_DEBUG() do {\
    GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(DWM1000_SLEEP_PORT), GPIO_PIN_MASK(DWM1000_SLEEP_PIN)); \
    GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(DWM1000_SLEEP_PORT), GPIO_PIN_MASK(DWM1000_SLEEP_PIN)); \
    GPIO_CLR_PIN(GPIO_PORT_TO_BASE(DWM1000_SLEEP_PORT), GPIO_PIN_MASK(DWM1000_SLEEP_PIN)); \
    GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(DWM1000_LISTEN_PORT), GPIO_PIN_MASK(DWM1000_LISTEN_PIN)); \
    GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(DWM1000_LISTEN_PORT), GPIO_PIN_MASK(DWM1000_LISTEN_PIN)); \
    GPIO_CLR_PIN(GPIO_PORT_TO_BASE(DWM1000_LISTEN_PORT), GPIO_PIN_MASK(DWM1000_LISTEN_PIN)); \
    GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(DWM1000_SEND_PORT), GPIO_PIN_MASK(DWM1000_SEND_PIN)); \
    GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(DWM1000_SEND_PORT), GPIO_PIN_MASK(DWM1000_SEND_PIN)); \
    GPIO_CLR_PIN(GPIO_PORT_TO_BASE(DWM1000_SEND_PORT), GPIO_PIN_MASK(DWM1000_SEND_PIN)); \
  } while (0)
#else
  #define LISTEN_CLR() do {} while(0)
  #define LISTEN_SET() do {} while(0)
  #define SLEEP_CLR() do {} while(0)
  #define SLEEP_SET() do {} while(0)
  #define SEND_CLR() do {} while(0)
  #define SEND_SET() do {} while(0)
  #define INIT_GPIO_DEBUG() do {} while(0)
#endif /* DEBUG_GPIO_TSCH */
/* END : Just for the debugging of TSCH */

/* Time to read the current time on the DW1000 using SPI in microsecond */
#ifndef DW1000_SFD_READOUT_OFFSET
  #define DW1000_SFD_READOUT_OFFSET 18 
#endif
// #define DOUBLE_BUFFERING

/* Used to fix an error with an possible interruption before
   the driver initialization */
static int dw1000_driver_init_down = 0;

/* Are we currently in poll mode? Disabled by default */
static uint8_t volatile poll_mode = 0;

#if DW1000_TSCH
  /* Define the current TSCH channel used */
  static uint8_t volatile tsch_channel = 0;
#endif /* DW1000_TSCH */

/* Used by the driver to know if the transmit() fonction 
have to send directly the message or if it is a delayed transmission */
static int volatile dw1000_is_delayed_tx = 0;


/* store the current DW1000 configuration */
static dw1000_base_conf_t dw1000_conf;
static dw1000_frame_quality last_packet_quality;
static uint8_t sleep_mode = RADIO_IDLE;

/**
 * \brief Define a loop to wait until the success of "cond" or the expiration 
 *        of the max_time.
 * 
 * \param cond      A boolean condition
 * \param max_time  A time in micro second
 */
#define BUSYWAIT_UNTIL(cond, max_time)  \
          BUSYWAIT_UPDATE_UNTIL("", cond, max_time)

/**
 * \brief Define a loop to wait until the success of "cond" or the expiration 
 *        of the max_time.
 *
 * \param update    The update command (typically for update a register).
 * \param cond      A boolean condition.
 * \param max_time  A time in micro second.
 */
#define BUSYWAIT_UPDATE_UNTIL(update, cond, max_time) \
  do { \
    rtimer_clock_t timeout = RTIMER_NOW() \
                             + microsecond_to_clock_tik(max_time); \
    watchdog_periodic(); \
    do { \
      update; \
    } while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), timeout)); \
  } while(0)


volatile uint8_t dw1000_driver_sfd_counter = 0;
volatile uint16_t dw1000_driver_sfd_start_time = 0;
volatile uint16_t dw1000_driver_sfd_end_time = 0;

static volatile uint16_t last_packet_timestamp = 0;

/* start private function */
void dw1000_schedule_tx(uint16_t delay_us);
void dw1000_schedule_rx(uint16_t delay_us);
void dw1000_update_frame_quality(void);
dw1000_preamble_code_t
dw1000_get_preamble_code(dw1000_channel_t channel, dw1000_prf_t prf);
void dw1000_set_tsch_channel(uint8_t channel);
void dw1000_rx_timeout(uint16_t timeout);
/* end private function */

static int dw1000_driver_prepare(const void *data, unsigned short payload_len);
static int dw1000_driver_transmit(unsigned short payload_len);
static int dw1000_driver_send(const void *data, unsigned short payload_len);
static int dw1000_driver_read(void *buf, unsigned short bufsize);

static int dw1000_driver_cca(void);

static void dw1000_on(void);
static void dw1000_off(void);

static int dw1000_driver_receiving_packet(void);
static int dw1000_driver_pending_packet(void);

static radio_result_t dw1000_driver_get_value(radio_param_t param,
                                              radio_value_t *value);
static radio_result_t dw1000_driver_set_value(radio_param_t param,
                                              radio_value_t value);

static radio_result_t dw1000_driver_get_object(radio_param_t param,
                                               void *dest, size_t size);
static radio_result_t dw1000_driver_set_object(radio_param_t param,
                                               const void *src, size_t size);
static uint32_t get_sfd_timestamp(uint32_t reg_addr);
static void set_frame_filtering(uint8_t enable);
static void set_auto_ack(uint8_t enable);
static void set_poll_mode(uint8_t enable);

/*---------------------------------------------------------------------------*/
PROCESS(dw1000_driver_process, "DW1000 driver");
/*---------------------------------------------------------------------------*/

signed char dw1000_driver_last_rssi;
uint8_t dw1000_driver_last_correlation;
/*---------------------------------------------------------------------------*/

const struct radio_driver dw1000_driver =
{
  dw1000_driver_init,
  dw1000_driver_prepare,
  dw1000_driver_transmit,
  dw1000_driver_send,
  dw1000_driver_read,
  dw1000_driver_cca,
  dw1000_driver_receiving_packet,
  dw1000_driver_pending_packet,
  dw1000_driver_on,
  dw1000_driver_off,
  dw1000_driver_get_value,
  dw1000_driver_set_value,
  dw1000_driver_get_object,
  dw1000_driver_set_object
};
/*---------------------------------------------------------------------------*/

static uint8_t receive_on = 0;

/*---------------------------------------------------------------------------*/
static uint8_t locked = 0;
static uint8_t lock_on = 0;
static uint8_t lock_off = 0;


/*---------------------------------------------------------------------------*/

/**
 * \brief Initialize SPI configuration for interacting with the DW1000
 *          and configure the DW1000 with the good channel.
 */
int
dw1000_driver_init(void)
{
  PRINTF("dw1000_driver_init\r\n");

  dw1000_arch_init();
  dw1000_arch_spi_set_clock_freq(DW_SPI_CLOCK_FREQ_INIT_STATE);

  /* Check if SPI communication works by reading device ID */
  assert(0xDECA0130 == dw_read_reg_32(DW_REG_DEV_ID, DW_LEN_DEV_ID));

  /* Simple reset of device. */
  dw_soft_reset(); /* Need to be call with a SPI speed < 3MHz */
  
  /* clear all interrupt */
  dw_clear_pending_interrupt(0x07FFFFFFFFULL);

  /* load the program to compute the timestamps */
  dw_load_lde_code(); /* Need to be call with a SPI speed < 3MHz */

  dw1000_arch_spi_set_clock_freq(DW_SPI_CLOCK_FREQ_IDLE_STATE);

#if DW1000_IEEE802154_EXTENDED
  PRINTF("DW1000 set to use IEEE 802.15.4-2011 UWB non-standard mode, ");
  PRINTF("extended frame max 265 bytes.\r\n");
  dw_enable_extended_frame();
#else
  PRINTF("DW1000 set to use IEEE 802.15.4-2011 UWB standard mode.\r\n");
  dw_disable_extended_frame();
#endif

#if DW1000_TSCH
  /* we configure the bitrate and the preamble length */
  dw1000_driver_config(DW_CHANNEL_1, DW1000_DATA_RATE, DW1000_PREAMBLE, 
                        DW1000_PRF);
  /* we change the channel according the default configuration */
  dw1000_driver_set_value(RADIO_PARAM_CHANNEL, (radio_value_t) DW1000_CHANNEL);
  printf("TSCH Channel %d, ", DW1000_CHANNEL);
#else
  dw1000_driver_config(DW1000_CHANNEL, DW1000_DATA_RATE, DW1000_PREAMBLE, 
                        DW1000_PRF);
#endif /* DW1000_TSCH */

  printf("Channel %d, Data rate %d kb/s, Preamble %d, PRF %d MHz\n", 
                (unsigned int) dw1000_conf.channel, 
                (unsigned int) dw1000_conf.data_rate, 
                (unsigned int) dw1000_conf.preamble_length, 
                (dw1000_conf.prf == 1) ? 16U : 64U);

  dw_disable_rx_timeout();
  dw_disable_receive_abort_on_RSD_error();

  /* Contiki should set correct value. */
  dw_set_pan_id(0xffff);
  dw_set_short_addr(0x0000);

#ifdef DOUBLE_BUFFERING
  dw_enable_double_buffering();
#else
  // dw_enable_automatic_receiver_Re_Enable();
  dw_disable_automatic_receiver_Re_Enable();
#endif /* DOUBLE_BUFFERING */

#if DEBUG_LED
  dw_enable_gpio_led(); /* /!\ Increase the power consumption. */
#else
  dw_disable_gpio_led();
#endif

  enable_error_counter(); /* /!\ Increase the power consumption. */

  set_poll_mode(poll_mode);

  set_auto_ack(DW1000_CONF_AUTOACK); /* configure auto ACK */

  /* If LDO tuning available then load this value */ 
  dw_load_ldotune();

  process_start(&dw1000_driver_process, NULL);

  dw1000_driver_init_down = 1;
  dw1000_arch_gpio8_setup_irq();
  INIT_GPIO_DEBUG();

  return 1;
}
/**
 * \brief Copy data to the TX buffer.
 */
static int
dw1000_driver_prepare(const void *payload,
                      unsigned short payload_len)
{
  PRINTF("dw1000_driver_prepare\r\n");

  // rtimer_clock_t t0 = RTIMER_NOW();

  uint16_t data_len = payload_len;

  RIMESTATS_ADD(lltx);

  if(!DW1000_CONF_CHECKSUM) {
    dw_suppress_auto_FCS_tx();
  } else {
    data_len += FOOTER_LEN; /* add the FCS size */
  }
  /* preparing DW1000 for sending */

  dw_set_tx_frame_length(data_len);

  /* dw_disable_delayed_tx_rx();  default value is the same, not useful */

  if(payload_len > 0) {
    /* Copy data to DW1000 */
    dw_write_reg(DW_REG_TX_BUFFER, payload_len, (uint8_t *)payload);
  } 

#if DEBUG_VERBOSE
  uint8_t tempRead[8];
  dw_read_reg(DW_REG_PANADR, DW_LEN_PANADR, tempRead);
  print_u8_Array_inHex("Reading PAN ID:", tempRead, DW_LEN_PANADR);

  uint8_t frame[DW1000_MAX_PACKET_LEN + 1];
  dw_read_reg(DW_REG_TX_BUFFER, payload_len, (uint8_t *)frame);
  print_frame(payload_len, frame);
  PRINTF("payload_len %i\r\n", payload_len);
  PRINTF("Data len %i\r\n", (unsigned int)data_len);
#endif
  // RELEASE_LOCK();
  PRINTF("DW1000 prepare data lenght %u \n", (unsigned int)data_len);

  // rtimer_clock_t t1 = RTIMER_NOW();
  // printf("Prepare time %ld (ms)\n", RTIMERTICKS_TO_US(t1-t0) );
  
  return RADIO_TX_OK;
}
/**
 * \brief   Transmit sends an already prepared packet.
 *          It takes an unsigned short int that indicates the number
 *          of bytes to be transmitted and returns an int that indicates
 *          whether the transmission was successful or not.
 *
 * \return  The state of the transmission at the end of this.
 * \retval RADIO_TX_OK          Indicates that the transmission succeeded.
 * \retval RADIO_TX_ERR         Indicates that an error of some description
 *                              has occurred.
 * \retval RADIO_TX_COLLISION   Indicates that a collision has occurred.
 *                              This is only used by contikimac (for radios
 *                              that block in TX and do hardware ACK detection)
 *                              and nullrdc.
 * \retval RADIO_TX_NOACK       Indicates that no acknowledgment has been
 *                              received.
 */
static int
dw1000_driver_transmit(unsigned short payload_len)
{

#if RADIO_DELAY_MEASUREMENT
  uint64_t transmission_call, transmission_sfd;
  transmission_call = dw_read_reg_64(DW_REG_SYS_TIME, DW_LEN_SYS_TIME);
  transmission_sfd = 0;
#endif /* RADIO_DELAY_MEASUREMENT */

  // rtimer_clock_t t0 = RTIMER_NOW();
  PRINTF("dw1000_driver_transmit \r\n");

  uint8_t rx_state = receive_on;
  int tx_return = RADIO_TX_ERR;
#if DEBUG
  uint8_t count_idle = 0, count_txtrt = 0;
#endif /* DEBUG */

  /* abort reception and disable interrupt to be able to send a frame */
  if(rx_state) {
    dw_idle();
    receive_on = 1;
  }
  ENERGEST_ON(ENERGEST_TYPE_TRANSMIT);
  
  if(dw1000_is_delayed_tx){
    /* wait the effective start of the transmission */
    uint64_t sys_time = 0ULL, dx_time = 0ULL;
    /* We wait for a maximum of 1ms to be improve in case of lower bitrate */
    BUSYWAIT_UPDATE_UNTIL(
                    watchdog_periodic();\
                    dw_read_reg(DW_REG_SYS_TIME, DW_LEN_SYS_TIME, (uint8_t *)&sys_time);\
                    dw_read_reg(DW_REG_DX_TIME, DW_LEN_DX_TIME, (uint8_t *)&dx_time);,
                    sys_time >= dx_time, 
                    1000);
  }
  else{
    /* No wait for response, no delayed transmission */
    dw_init_tx(0, 0);
  }
  SEND_SET();

#if DEBUG
  /* wait the effective start of the transmission */
  uint8_t sys_ctrl_lo;
  BUSYWAIT_UPDATE_UNTIL(dw_read_subreg(DW_REG_SYS_CTRL, 0x0, 1, &sys_ctrl_lo);
                  watchdog_periodic();, 
                  ((sys_ctrl_lo & DW_TXSTRT_MASK) == 0),
                  microsecond_to_clock_tik(100));
#endif /* DEBUG */

  if(DW1000_CONF_CHECKSUM) {
    payload_len += FOOTER_LEN; /* add the FCS size */
  }
  
  /* wait the end of the transmission */
  /* only reads low-byte of DW1000's SYS_STATUS (bit 7 is TXFRS) */
  uint8_t sys_status_lo = 0x0;
  uint8_t count_send = 0; 
  BUSYWAIT_UPDATE_UNTIL(dw_read_subreg(DW_REG_SYS_STATUS, 0x0, 1, &sys_status_lo); 
                  count_send++; watchdog_periodic();,
                  ((sys_status_lo & DW_TXFRS_MASK) != 0),
                  theorical_transmission_approx(dw1000_conf.preamble_length, 
                  dw1000_conf.data_rate, dw1000_conf.prf, payload_len)+ 
                  DW1000_SPI_DELAY);

  PRINTF("Number of loop waiting IDLE: %d\n", count_idle);
  PRINTF("Number of loop waiting Tx on: %d\n", count_txtrt);
  PRINTF("Number of loop waiting Transmit Frame Sent: %d\n", count_send);
  
  if((sys_status_lo & DW_TXFRS_MASK) != 0) {
#if RADIO_DELAY_MEASUREMENT
  dw_read_subreg(DW_REG_TX_TIME, DW_SUBREG_TX_RAWST, DW_SUBLEN_TX_RAWST, 
                  (uint8_t *) &transmission_sfd);
  printf("RADIO_DELAY_BEFORE_TX RTIMER %lu\n", \
            RADIO_TO_RTIMER(transmission_sfd-transmission_call));
  printf("RADIO_DELAY_BEFORE_TX US %lu\n", \
            RTIMERTICKS_TO_US(RADIO_TO_RTIMER(transmission_sfd-transmission_call)));
  printf("RADIO_DELAY_BEFORE_TX US %lu (based on dw1000 time)\n", \
            RADIO_TO_US(transmission_sfd-transmission_call));
  printf("RADIO_DELAY_BEFORE_TX DW1000 %llu\n", transmission_sfd - transmission_call);
#endif /* RADIO_DELAY_MEASUREMENT */

    tx_return = RADIO_TX_OK;
  }else{
    // printf("Is delayed ? %d\n", dw1000_is_delayed_tx);
    // print_sys_status(dw_read_reg_64(DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS));

    // dw_idle(); /* error: abort the transmission */
    dw1000_driver_init();
  }

 
  /* re-enable the RX state
      clean the interrupt
      and change the RX buffer*/
  if(rx_state){
    dw1000_on();
  }
  else{
    dw1000_driver_clear_pending_interrupt();
  }

  ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);

  SEND_CLR();

  dw1000_is_delayed_tx = 0; /* disable delayed transmition for the next call */

#if DEBUG_VERBOSE
  print_sys_status(dw_read_reg_64(DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS));
#endif
#if DEBUG 
  if(tx_return == RADIO_TX_OK) {
    PRINTF("TX RADIO_TX_OK \r\n");
  }
  else if(tx_return == RADIO_TX_NOACK) {
    PRINTF("TX RADIO_TX_NOACK \r\n");
  }
  else if(tx_return == RADIO_TX_ERR) {
    PRINTF("TX RADIO_TX_ERR \r\n");
  }
  else{
    printf("TX result %d\n", tx_return);
  }
#endif
  // rtimer_clock_t t1 = RTIMER_NOW();
  // printf("tx time %ld (ms)\n", RTIMERTICKS_TO_US(t1-t0) );
  return tx_return;
}
/**
 * \brief     prepare and transmit a packet.
 *            This takes a pointer to the payload and an
 *            unsigned short int that indicate the number of bytes.
 *
 * \return    The same return codes as dw1000_driver_transmit().
 */
static int
dw1000_driver_send(const void *data, unsigned short payload_len)
{
  PRINTF("dw1000_driver_send\r\n");
  dw1000_driver_prepare(data, payload_len);
  return dw1000_driver_transmit(payload_len);
}

/**
 * \brief     Clean intterrupt flag and receive status in poll mode.
 */
void driver_flush_receive_buffer(void){
  if(poll_mode){
    /* See Figure 14: Flow chart for using double RX buffering
     * Of the manual */
#ifdef DOUBLE_BUFFERING
    uint32_t sys_status = dw_read_reg_32( DW_REG_SYS_STATUS, 4);
    if((sys_status & DW_RXOVRR_MASK) > 0){ /* overrun may be occurs */

      dw_idle();
      dw_trxsoft_reset();
      dw_change_rx_buffer();
      
      PRINTF("dw1000_process: RX Overrun case 2\n");
      if(receive_on)
        dw1000_on();
    }
    else{
      if(dw_good_rx_buffer_pointer()){
        dw_clear_receive_status();
      }
      dw_change_rx_buffer();
    }
#else
    if(receive_on)
      dw1000_on();
    dw_clear_receive_status();
#endif /* DOUBLE_BUFFERING */
  }/* end if(poll_mode) */
}

/**
 * \brief     Reads a pending packet from the radio.
 *
 * \return    The length of the read packet.
 */
static int
dw1000_driver_read(void *buf, unsigned short bufsize)
{
  PRINTF("dw1000_driver_read\r\n");
  if(poll_mode){
#ifdef DOUBLE_BUFFERING
    uint32_t sys_status = dw_read_reg_32(DW_REG_SYS_STATUS, 4);
    if((sys_status & DW_RXOVRR_MASK) > 0){  /* Overrun */
      PRINTF("dw1000_read > dw_overrun\r\n");
      dw_idle();
      dw_trxsoft_reset();
      dw_change_rx_buffer();
      if(receive_on) {
        dw1000_on();
      }
    }
#endif     /* DOUBLE_BUFFERING */
  } /* end if(poll_mode) */

#if DW1000_IEEE802154_EXTENDED
  int len = dw_get_rx_extended_len();
#else
  int len = dw_get_rx_len();
#endif

  if(len > DW1000_MAX_PACKET_LEN) {
    RIMESTATS_ADD(toolong);
    driver_flush_receive_buffer();
    return 0;
  }

  if(len <= FOOTER_LEN) {
    RIMESTATS_ADD(tooshort);
    driver_flush_receive_buffer();
    return 0;
  }

  if(len - FOOTER_LEN > bufsize) {
    RIMESTATS_ADD(toolong);
    driver_flush_receive_buffer();
    return 0;
  }

  if(DW1000_CONF_CHECKSUM) {
    len -= FOOTER_LEN;
  }

  /* Store rx data in buf */
  dw_read_reg(DW_REG_RX_BUFFER, len, (uint8_t *)buf);

  RIMESTATS_ADD(llrx);

#if DEBUG_VERBOSE
  print_frame(len, buf);
#endif

  driver_flush_receive_buffer();

  return len;
}

/**
 * \brief  Performs a Channel Clear Assessment to see if there is other
 *        RF activity on the channel to avoid collisions.
 *
 * \returns The result of the CCA
 * \retval 0  indicates that the channel is busy.
 * \retval 1  indicates that the channel is clear.
 */
static int
dw1000_driver_cca(void)
{
  PRINTF("dw1000_driver_cca\r\n");
  /*

     dw_idle();
     dw1000_driver_disable_interrupt();
     dw1000_driver_clear_pending_interrupt();
     dw_db_init_rx(); */

     /*
   * Check if a preamble has been detected.
   */
  /*
     cc2420 RSSI time 128us ~1/7812s
     dw1000 127bytes frame at 6800kps > 287us > 1/3484
     BUSYWAIT_UNTIL((dw_read_reg_64(DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS) & 
                      DW_RXPRD_MASK) == 0, RTIMER_SECOND / 3000);
     dw1000_driver_enable_interrupt();
     return (dw_read_reg_64(DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS) & 
              DW_RXPRD_MASK) == 0;
   */
  return 1;
}
/**
 * \brief     This function checks whether we are currently receiving a
 *            packet. This is used by several MAC layers to detect and
 *            indicate a collision before a TX.
 *
 * \retval 0  indicates that we are not receiving a packet.
 * \retval 1  indicates that we are receiving a packet.
 */
static int
dw1000_driver_receiving_packet(void)
{
  PRINTF("dw1000_driver_receiving_packet\r\n");
  uint64_t sys_status = dw_read_reg_64(DW_REG_SYS_STATUS, 
                              DW_LEN_SYS_STATUS);
  /* return if we currently receiving a message and we don't have finish to
  receive it */
  return ((sys_status & (DW_RXPRD_MASK     /* Receiver preamble detected */
                      | DW_RXSFDD_MASK    /* Receiver SFD detected */
                      | DW_RXPHD_MASK)) > 0)   /* Receiver PHY Header Detect */
        && !((sys_status & ((DW_RXFCG_MASK /* Receiver CRC good */
                      | DW_RXFCE_MASK /*  Receiver FCS Error */
                      | DW_LDEERR_MASK /* Leading edge detection processing error. */
                      | DW_RXDFR_MASK /* Receiver Data Frame Ready. */
                      | DW_RXFCE_MASK)) > 0)); /* Receiver PHY Header Error */
                      /* | DW_RXRFSL_MASK  Receiver Reed Solomon Frame Sync Loss. (keep receiving because the CRC of the frame could be good*/
}

/**
 * \brief     Checks to see if we have a pending packet. Some drivers check
 *            a flag set by an interrupt routine.
 *
 * \retval 0  indicates that there is no pending packet.
 * \retval 1  indicates that there is a pending packet.
 */
static int
dw1000_driver_pending_packet(void)
{
  PRINTF("dw1000_driver_pending_packet\r\n");
  /* return true if we have have the flag "data frame ready" 
  return (dw_read_reg_64(DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS) & DW_RXDFR_MASK) > 0; */
  /* Return true if we have receive a frame and the CRC is good */
  return (dw_read_reg_64(DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS) & DW_RXFCG_MASK) > 0;
}
/**
 * \brief     Turns "on" the radio. This function should put the radio into
 *            Receive mode and is called very frequently as the MAC layer
 *            turns on and off the radio rapidly.
 *
 * \return    an int that indicates whether the turn on succeeded.
 * \retval 0  indicates that the radio did not turn on.
 * \retval 1  indicates that the radio did turn on.
 */
int
dw1000_driver_on(void)
{
#if RADIO_DELAY_MEASUREMENT
  uint64_t receive_call, receive_begin;
  receive_call = dw_read_reg_64(DW_REG_SYS_TIME, DW_LEN_SYS_TIME);
#endif /* RADIO_DELAY_MEASUREMENT */
  
  PRINTF("dw1000_driver_on\r\n");
  if(receive_on) {
    return 1;
  }
  if(locked) {
    lock_on = 1;
    return 1;
  }

  dw1000_on();

  LISTEN_SET();
  // uint32_t rx_time = RTIMER_NOW();
  // printf("rx time %u\n", rx_time);

#if RADIO_DELAY_MEASUREMENT
  receive_begin = dw_read_reg_64(DW_REG_SYS_TIME, DW_LEN_SYS_TIME);
  printf("RADIO_DELAY_BEFORE_RX RTIMER %lu\n", \
            RADIO_TO_RTIMER(receive_begin-receive_call));
  printf("RADIO_DELAY_BEFORE_RX US %lu\n", \
            RTIMERTICKS_TO_US(RADIO_TO_RTIMER(receive_begin-receive_call)));
  printf("RADIO_DELAY_BEFORE_RX US (based on DW100 time)%lu\n", \
            RADIO_TO_US(receive_begin-receive_call));
  printf("RADIO_DELAY_BEFORE_RX DW1000 %llu\n", receive_begin-receive_call);
#endif /* RADIO_DELAY_MEASUREMENT */

  return 1;
}
/**
 * \brief Turn the radio on.
 * The receiver has a delay of 16μs after issuing the enable receiver command,
 *  after which it will start receiving preamble symbols. 
 */
void
dw1000_on(void)
{
#ifdef DOUBLE_BUFFERING
  if(!dw_good_rx_buffer_pointer()) { /* check HSRBP == ICRBP */
    /* Host Side Receive Buffer Pointer Toggle to 1. */
    dw_change_rx_buffer();
  }
#endif /* DOUBLE_BUFFERING */

  dw_init_rx();

  if(!poll_mode){
    dw1000_driver_enable_interrupt();
  }
  
  /* The receiver has a delay of 16μs after issuing the enable receiver command,
   *  after which it will start receiving preamble symbols. */ 
  /* dw1000_us_delay(16); */

  ENERGEST_ON(ENERGEST_TYPE_LISTEN);
  receive_on = 1;
}
/**
 * \brief     Turns "off" the radio. This should put the radio into Idle
 *            or a low-power state.
 * \return    An int that indicates whether the turn off succeeded.
 * \retval 0  Indicates that the radio did not turn off.
 * \retval 1  Indicates that the radio did turn off.
 */
int
dw1000_driver_off(void)
{
  PRINTF("dw1000_driver_off\r\n");
  /* Don't do anything if we are already turned off. */
  if(receive_on == 0) {
    return 1;
  }

  /* If we are called when the driver is locked, we indicate that the
     radio should be turned off when the lock is unlocked. */
  if(locked) {
    /*    printf("Off when locked (%d)\r\n", locked);*/
    lock_off = 1;
    return 0;
  }

  /* If we are currently receiving a packet (indicated by SFD == 1),
     we don't actually switch the radio off now, but signal that the
     driver should switch off the radio once the packet has been
     received and processed, by setting the 'lock_off' variable. */
  // uint64_t status = dw_read_reg_64(DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS);
  // if(((status & DW_RXSFDD_MASK) > 0) 
  //   && ((status & (DW_RXDFR_MASK | DW_RXPHE_MASK)) == 0)) {
  //   lock_off = 1;
  //   return 0;
  // } else {
    LISTEN_CLR();
    dw1000_off();
  // }
  return 1;
}
/**
 * \brief Turn the radio off.
 */
static void
dw1000_off(void)
{
  receive_on = 0;

  ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
  /* we need to disable interrupt before the call of dw_idle() 
      because an ISR can append... */
  if(!poll_mode){
    dw1000_driver_disable_interrupt();  
    dw1000_driver_clear_pending_interrupt();
  }

#ifdef DOUBLE_BUFFERING
  dw_trxoff_db_mode();
#else
  dw_idle();
#endif /* DOUBLE_BUFFERING */
#if DEBUG
  /* wait the effective TRX OFF */
  uint8_t sys_ctrl_lo;
  BUSYWAIT_UPDATE_UNTIL(dw_read_subreg(DW_REG_SYS_CTRL, 0x0, 1, &sys_ctrl_lo);
                watchdog_periodic(); /* count_idle++; */, 
                ((sys_ctrl_lo & DW_TRXOFF_MASK) == 0),
                microsecond_to_clock_tik(50));
#endif /* DEBUG */

  if(!poll_mode){
    dw1000_driver_enable_interrupt(); 
  } 
}
/**
 * \brief   Get a radio parameter value.
 *
 * \param param Defined the type of parameter.
 * \param value The radio parameter value.
 *
 * \return     Return the result of the getter.
 * \retval RADIO_RESULT_OK             The value was set correctly.
 * \retval RADIO_RESULT_INVALID_VALUE  The value was invalid.
 * \retval RADIO_RESULT_NOT_SUPPORTED  The value was not supported.
 */
static radio_result_t
dw1000_driver_get_value(radio_param_t param,
                        radio_value_t *value)
{
  PRINTF("dw1000_driver_get_value\r\n");
  if(!value) {
    return RADIO_RESULT_INVALID_VALUE;
  }

  switch(param) {
  case RADIO_PARAM_POWER_MODE:
    *value = receive_on ? RADIO_POWER_MODE_ON : RADIO_POWER_MODE_OFF;
    return RADIO_RESULT_OK;
  case RADIO_PARAM_CHANNEL:
#if DW1000_TSCH
    *value = (radio_value_t) tsch_channel;
#else
    *value = (radio_value_t) dw1000_conf.channel;
#endif /* DW1000_TSCH */
    return RADIO_RESULT_OK;
  case RADIO_PARAM_PAN_ID:
    *value = (radio_value_t) dw_get_pan_id();
    return RADIO_RESULT_OK;
  case RADIO_PARAM_16BIT_ADDR:
    *value = (radio_value_t) dw_get_short_addr();
    return RADIO_RESULT_OK;
  case RADIO_PARAM_RX_MODE:
    *value = 0;
    if(dw_is_frame_filtering_on()){
      *value |= RADIO_RX_MODE_ADDRESS_FILTER;
    }
    if(dw_is_automatic_ack()) {
      *value |= RADIO_RX_MODE_AUTOACK;
    }
    if(poll_mode){
      *value |= RADIO_RX_MODE_ADDRESS_FILTER;
    }
    return RADIO_RESULT_OK;
  case RADIO_PARAM_TX_MODE:
    /* radio not support CCA */
    *value = 0;
    return RADIO_RESULT_OK;

  case RADIO_PARAM_TXPOWER:
  case RADIO_PARAM_CCA_THRESHOLD:
  case RADIO_PARAM_RSSI:
    /* Return the RSSI value in dBm */
  case RADIO_PARAM_LAST_RSSI:
    /* RSSI of the last packet received */
  case RADIO_PARAM_LAST_LINK_QUALITY:
    /* LQI of the last packet received */
    return RADIO_RESULT_NOT_SUPPORTED;

  case RADIO_CONST_CHANNEL_MIN:
#if DW1000_TSCH
    *value = 0;
#else
    *value = 1;
#endif /* DW1000_TSCH */
    return RADIO_RESULT_OK;
  case RADIO_CONST_CHANNEL_MAX:
#if DW1000_TSCH
    *value = 5;
#else
    *value = 7;
#endif /* DW1000_TSCH */
    return RADIO_RESULT_OK;
  case RADIO_CONST_TXPOWER_MIN:
    return RADIO_RESULT_NOT_SUPPORTED;
  case RADIO_CONST_TXPOWER_MAX:
    return RADIO_RESULT_NOT_SUPPORTED;
  case RADIO_SLEEP_STATE:
    *value =  sleep_mode;
    return RADIO_RESULT_OK;
  default:
    return RADIO_RESULT_NOT_SUPPORTED;
  }
}
/**
 * \brief      Set a radio parameter value.
 *
 * \param param Defined the type of parameter.
 * \param value The radio parameter value.
 *
 * \return     Return the result of the setter.
 * \retval RADIO_RESULT_OK             The value was set correctly.
 * \retval RADIO_RESULT_INVALID_VALUE  The value was invalid.
 * \retval RADIO_RESULT_NOT_SUPPORTED  The value was not supported.
 */
static radio_result_t
dw1000_driver_set_value(radio_param_t param, radio_value_t value)
{
  PRINTF("dw1000_driver_set_value\r\n");
  switch(param) {
  case RADIO_PARAM_POWER_MODE:
    if(value == RADIO_POWER_MODE_ON) {
      dw1000_driver_on();
      return RADIO_RESULT_OK;
    }
    if(value == RADIO_POWER_MODE_OFF) {
      dw1000_driver_off();
      return RADIO_RESULT_OK;
    }
    return RADIO_RESULT_INVALID_VALUE;
  case RADIO_PARAM_CHANNEL:
#if DW1000_TSCH
    if(value < 0 || value > 7) {
      /* channel 6 is not supported by the DW1000*/
      return RADIO_RESULT_INVALID_VALUE;
    }
    uint8_t receive_state = receive_on;
    if(receive_state){
      dw1000_driver_off();
    }
    dw1000_set_tsch_channel(value);
    if(receive_state){
      dw1000_driver_on();
    }
    return RADIO_RESULT_OK;
#else /* !DW1000_TSCH */
    if(value < 1 || value > 7 || value == 6) {
      /* channel 6 is not supported */
      return RADIO_RESULT_INVALID_VALUE;
    }
    /* only change the channel not others parameters */
    dw1000_driver_config(value, dw1000_conf.data_rate, \
                        dw1000_conf.preamble_length, dw1000_conf.prf);

    return RADIO_RESULT_OK;
#endif
  case RADIO_PARAM_RX_MODE:
    if(value & ~(RADIO_RX_MODE_ADDRESS_FILTER |
         RADIO_RX_MODE_AUTOACK | RADIO_RX_MODE_POLL_MODE)) {
       return RADIO_RESULT_INVALID_VALUE;
       }
       set_frame_filtering((value & RADIO_RX_MODE_ADDRESS_FILTER) != 0);
       set_auto_ack((value & RADIO_RX_MODE_AUTOACK) != 0);
       set_poll_mode((value & RADIO_RX_MODE_POLL_MODE) != 0);
    return RADIO_RESULT_OK;
  case RADIO_PARAM_TX_MODE:
    /* we can not send on CCA */
    if(value & RADIO_TX_MODE_SEND_ON_CCA){
      return RADIO_RESULT_NOT_SUPPORTED;
    }
    return RADIO_RESULT_OK;
  case RADIO_PARAM_TXPOWER:
    return RADIO_RESULT_NOT_SUPPORTED;
  case RADIO_PARAM_CCA_THRESHOLD:
    return RADIO_RESULT_NOT_SUPPORTED;

  case RADIO_PARAM_PAN_ID:
    dw_set_pan_id(value & 0xFFFF);
    return RADIO_RESULT_OK;
  
  case RADIO_PARAM_16BIT_ADDR:
    dw_set_short_addr(value & 0xFFFF);
    return RADIO_RESULT_OK;
  
  case RADIO_SLEEP_STATE:
    SLEEP_SET();
    if(value == RADIO_SLEEP) {
            dw_read_reg_32(DW_REG_DEV_ID, DW_LEN_DEV_ID);
      // printf("put radio in sleep\n");
        assert(0xDECA0130 == dw_read_reg_32(DW_REG_DEV_ID, DW_LEN_DEV_ID));
        // dw_conf_print();
      dw1000_arch_spi_set_clock_freq(DW_SPI_CLOCK_FREQ_INIT_STATE);
      set_in_deep_sleep();
      dw1000_arch_init_deepsleep();
    }
    else if(value == RADIO_REQUEST_WAKEUP) {
      dw1000_arch_wake_up(DW1000_PIN_ENABLE);
      dw1000_us_delay(550);
      dw1000_arch_wake_up(DW1000_PIN_DISABLE);
      // printf("radio wake-up\n");
    }
    else if(value == RADIO_IDLE) {
      dw1000_arch_restore_idle_state();
      // printf("radio-idle\n");
              // dw_conf_print();
      /* Check if SPI communication works by reading device ID */
      // assert(0xDECA0130 == dw_read_reg_32(DW_REG_DEV_ID, DW_LEN_DEV_ID));

      set_poll_mode(poll_mode);

      dw1000_arch_spi_set_clock_freq(DW_SPI_CLOCK_FREQ_IDLE_STATE);
      dw_read_reg_32(DW_REG_DEV_ID, DW_LEN_DEV_ID);
      dw_clear_pending_interrupt(DW_MCPLOCK_MASK|DW_MSLP2INIT_MASK);

      /* restore AGC_TUNE 2 value */
      const uint32_t agc_tune2_val = 0X2502A907UL;  /* Always use this */
      dw_write_subreg(DW_REG_AGC_CTRL, DW_SUBREG_AGC_TUNE2, DW_SUBLEN_AGC_TUNE2,
                      (uint8_t *) &agc_tune2_val);
      
      #if DEBUG_LED
      dw_enable_gpio_led_from_deepsleep(); /* /!\ Increase the power consumption. */
      #endif /* DEBUG_LED */
    }
    else{
      return RADIO_RESULT_INVALID_VALUE;
    }
    SLEEP_CLR();
    sleep_mode = value;
    return RADIO_RESULT_OK;

  default:
    return RADIO_RESULT_NOT_SUPPORTED;
  }
}
/**
 * \brief   Get a radio parameter object. The argument 'dest' must point
 *          to a memory area of at least 'size' bytes, and this memory area
 *          will contain the parameter object if the function succeeds.
 *
 * \param param Defined the type of parameter.
 * \param dest  Defined a point to a memory area.
 * \param size  Defined the size of the reading area.
 *
 * \return     Return the result of the getter.
 * \retval RADIO_RESULT_OK             The value was set correctly.
 * \retval RADIO_RESULT_INVALID_VALUE  The value was invalid.
 * \retval RADIO_RESULT_NOT_SUPPORTED  The value was not supported.
 */
static radio_result_t
dw1000_driver_get_object(radio_param_t param, void *dest, size_t size)
{  
  PRINTF("dw1000_driver_get_object\r\n");

  if(param == RADIO_PARAM_64BIT_ADDR) {
    if(size != 8 || !dest) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    /* The extended addr (64 bit) is store in the Extended Unique ID register */
    dw_read_reg(DW_REG_EID, DW_LEN_EID, (uint8_t *) dest);

    return RADIO_RESULT_OK;
  }

  if(param == RADIO_PARAM_PAN_ID) {
    if(size != 2 || !dest) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    *(uint16_t *) dest = dw_get_pan_id();

    return RADIO_RESULT_OK;
  }

  if(param == RADIO_PARAM_LAST_PACKET_TIMESTAMP) {
    if(size != sizeof(rtimer_clock_t) || !dest) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    *(rtimer_clock_t *)dest = get_sfd_timestamp(DW_REG_RX_TIME);
    return RADIO_RESULT_OK;
  }

  if(param == RADIO_PARAM_LAST_TX_PACKET_TIMESTAMP) {
    if(size != sizeof(rtimer_clock_t) || !dest) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    *(rtimer_clock_t *)dest = get_sfd_timestamp(DW_REG_TX_TIME);
    return RADIO_RESULT_OK;
  }

  if(param == RADIO_LOC_LAST_RX_TIMESPTAMP) {
    if(size != sizeof(uint64_t) || !dest) {
      return RADIO_RESULT_INVALID_VALUE;
    }

    dw_read_subreg(DW_REG_RX_TIME, DW_SUBREG_RX_STAMP, DW_SUBLEN_RX_STAMP, 
                    (uint8_t *) dest);
    return RADIO_RESULT_OK;
  }
  if(param == RADIO_LOC_LAST_TX_TIMESPTAMP) {
    if(size != sizeof(uint64_t) || !dest) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    dw_read_subreg(DW_REG_TX_TIME, DW_SUBREG_TX_STAMP, DW_SUBLEN_TX_STAMP, 
                    (uint8_t *) dest);
    return RADIO_RESULT_OK;
  }
  if(param == RADIO_LOC_RX_ANTENNA_DELAY) {
    if(size != sizeof(uint16_t) || !dest) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    *(uint16_t *) dest = dw_get_rx_antenna_delay();
    return RADIO_RESULT_OK;
  }
  if(param == RADIO_LOC_TX_ANTENNA_DELAY) {
    if(size != sizeof(uint16_t) || !dest) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    *(uint16_t *) dest = dw_get_tx_antenna_delay();
    return RADIO_RESULT_OK;
  }
  if(param == RADIO_RX_TIMEOUT_US) {
    if(size != sizeof(uint16_t) || !dest) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    if(dw_is_rx_timeout()){
      *(uint8_t *) dest = dw_get_rx_timeout();
    }
    *(uint8_t *) dest = 0;
    return RADIO_RESULT_OK;
  }


  return RADIO_RESULT_NOT_SUPPORTED;
}
/**
 * \brief   Set a radio parameter object. The memory area referred to by
 *          the argument 'src' will not be accessed after the function
 *          returns.
 *
 * \param param Defined the type of parameter.
 * \param dest  Defined a point to a memory area.
 * \param size  Defined the size of the writing area.
 *
 * \return     Return the result of the setter.
 * \retval RADIO_RESULT_OK             The value was set correctly.
 * \retval RADIO_RESULT_INVALID_VALUE  The value was invalid.
 * \retval RADIO_RESULT_NOT_SUPPORTED  The value was not supported.
 */
static radio_result_t
dw1000_driver_set_object(radio_param_t param,
                         const void *src, size_t size)
{
  int i;
  uint64_t ext_addr;
  PRINTF("dw1000_driver_set_object\r\n");

  if(param == RADIO_PARAM_64BIT_ADDR) {
    if(size != 8 || !src) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    ext_addr = 0;
    for(i = 0; i < 8; i++) {

      ext_addr |= ((uint64_t) ((uint8_t *)src)[7-i]) << (8 * i);
    }

    dw_set_extended_addr(ext_addr);

    return RADIO_RESULT_OK;
  }
  else if(param == RADIO_LOC_TX_ANTENNA_DELAY){
    if(size != 2 || !src) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    uint16_t delay = ((uint8_t *)src)[0] | ((uint8_t *)src)[1] << 8; 
    dw_set_tx_antenna_delay(delay);
  }
  else if(param == RADIO_LOC_RX_ANTENNA_DELAY){
    if(size != 2 || !src) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    uint16_t delay = ((uint8_t *)src)[0] | ((uint8_t *)src)[1] << 8; 
    dw_set_rx_antenna_delay(delay);
  }
  else if(param == RADIO_LOC_TX_DELAYED_US){
    if(size != 2 || !src) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    uint16_t schedule = ((uint8_t *)src)[0] | ((uint8_t *)src)[1] << 8; 
    // printf("schedule %d \n", schedule);
    dw1000_schedule_tx(schedule);
  }
  else if(param == RADIO_LOC_RX_DELAYED_US){
    if(size != 2 || !src) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    uint16_t schedule = ((uint8_t *)src)[0] | ((uint8_t *)src)[1] << 8; 
    dw1000_schedule_rx(schedule);
  }
  else if(param == RADIO_RX_TIMEOUT_US){
    if(size != 2 || !src) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    uint16_t timeout = ((uint8_t *)src)[0] | ((uint8_t *)src)[1] << 8;
    dw1000_rx_timeout(timeout);
  }
  return RADIO_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
/**
 *
 * \brief Set a timeout value for the reception duration. 
 * Usefull when delayed reception is used to avoid listening too long 
 * if we don't receive a frame. 
 * The delay is set in micro second. A value of 0 will disable this feature.
 **/
void dw1000_rx_timeout(uint16_t timeout){
  if(timeout == 0){
    dw_disable_rx_timeout();
  }
  else
  {
    dw_set_rx_timeout(timeout);
    dw_enable_rx_timeout();
  }
}
/**
 * \brief Enable interrupt for Frame with good CRC reception or Overrun
 */
void
dw1000_driver_enable_interrupt(void)
{
  dw1000_driver_clear_pending_interrupt();
  dw_enable_interrupt(DW_MRXFCE_MASK | DW_MRXDFR_MASK | DW_RXOVRR_MASK);
}
/**
 * \brief Disable interrupt
 */
void
dw1000_driver_disable_interrupt(void)
{ 
  dw_enable_interrupt(0x0UL);
}
/**
 * \brief Change the reception mode.
 * \input     enable
 *            If enable true: change the reception mode to pooling mode;
 *            If false change the reception to interrupt mode (enable interrupt).
 */
static void
set_poll_mode(uint8_t enable)
{  poll_mode = enable;

  if(enable) {
    dw1000_driver_disable_interrupt();
    dw1000_arch_gpio8_enable_irq();
  } else {
    dw1000_driver_enable_interrupt();
    dw1000_arch_gpio8_disable_irq();
  }
}
/**
 * \brief Enable or disable automatic acknowledgment.
 */
static void
set_auto_ack(uint8_t enable)
{
  if(enable) {
    dw_enable_automatic_ack();
    dw_config_switching_tx_to_rx_ACK(); /* Configure the Automatic ACK Turnaround Time */
    dw_sfd_init(); /* Do a fake send to initialize the SFD. 
                    Required if we don't have send message before the first ACK. */
  } else {
    dw_disable_automatic_ack();
  }
}
/**
 * \brief Enable or disable the frame filtering mode.
 */
static void
set_frame_filtering(uint8_t enable)
{
  if(enable) {
    dw_turn_frame_filtering_on(); /* enable frame filtering */
  } else {
    dw_turn_frame_filtering_off();
  }
}
/**
 * \brief Clear pending interruption.
 */
void
dw1000_driver_clear_pending_interrupt(void)
{
  // dw_clear_pending_interrupt(0x00000007FFFFFFFFULL);
  dw_clear_pending_interrupt(DW_MTXFRB_MASK
         | DW_MTXPRS_MASK
         | DW_MRXFCE_MASK
         | DW_MTXPHS_MASK
         | DW_MTXFRS_MASK
         /* receive */
         | DW_MRXFCG_MASK
         | DW_MRXDFR_MASK
         | DW_MLDEDONE_MASK);
}
/*
 * \brief Interrupt leaves frame intact in FIFO.
 */
int
dw1000_driver_interrupt(void)
{
  PRINTF("dw1000_driver_interrupt()\r\n");
  /*
     dw1000_driver_init_down is use for fix bug of IRQ call before DW1000 reset
   */
#if DEBUG_RANGING
     rtimer_clock_t t0 = RTIMER_NOW();
#endif /* DEBUG_RANGING */
  if(dw1000_driver_init_down) {
    /* we read only the first 32 bit of the register */
    uint32_t status = dw_read_reg_64(DW_REG_SYS_STATUS, 4);

#ifdef DOUBLE_BUFFERING
    if((status & DW_RXOVRR_MASK) > 0){  /* Overrun */
      PRINTF("dw1000_driver_interrupt() > dw_overrun\r\n");
      dw1000_driver_disable_interrupt();
      dw_idle();
      dw1000_driver_enable_interrupt();
      dw_trxsoft_reset();
      dw_change_rx_buffer();
      if(receive_on) {
        dw1000_on();
      }
    } else
#endif     /* DOUBLE_BUFFERING */
    if((status & DW_RXDFR_MASK) > 0){ /* frame ready */
      /* receiver Data Frame Ready. */
      process_poll(&dw1000_driver_process);
#if DEBUG_INTERRUPT
    uint32_t value = 0x0;
    /* RNG bit is the 15nd bit */
    dw_read_subreg(DW_REG_RX_FINFO, 0x0, 4, (uint8_t* ) &value);
    uint16_t payload_len =dw_get_rx_extended_len();
    printf("rx_info %04X%04X, frame length %d, sys_status %04X%04X\n", (unsigned int) (value >> 16), 
    (unsigned int) value, payload_len,  (unsigned int) (status >> 16), 
    (unsigned int) status);
    print_sys_status(status);

    uint8_t frame[payload_len];
    dw_read_reg(DW_REG_TX_BUFFER, payload_len, (uint8_t *)frame);
    print_frame(payload_len, frame);
    printf("ranging value %02X\n", (unsigned int) ranging_value);
#endif /* DEBUG_INTERRUPT */
  }
  /* error catch by RX re-enable function. */
}

return 1;
}
/*---------------------------------------------------------------------------*/
/* We receive a data frame (not a ranging frame) or we have an error */
PROCESS_THREAD(dw1000_driver_process, ev, data){
  int len = 0;
  PROCESS_BEGIN();

  PRINTF("dw1000_process: started\r\n");
  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

    /* We are not interested by the 5nd bytes */
    uint32_t status = dw_read_reg_32( DW_REG_SYS_STATUS, 4);
    while (
      /* if overrun or good message */
#ifdef DOUBLE_BUFFERING
      ((status & DW_RXOVRR_MASK) > 0) ||
#endif /* DOUBLE_BUFFERING */
      ((status & DW_RXDFR_MASK) > 0)){

      PRINTF("dw1000_process: calling receiver callback\r\n");

#ifdef DOUBLE_BUFFERING
      if((status & DW_RXOVRR_MASK) > 0){
        dw_idle();
        dw_trxsoft_reset();
        dw_change_rx_buffer();

        PRINTF("dw1000_process: RX Overrun case 1\n");
        if(receive_on)
          dw1000_on();
      }
      else
#endif /* DOUBLE_BUFFERING */
      if((status & DW_RXFCG_MASK) > 0) { // no overrun and good CRC
        len = dw_get_rx_extended_len();
        if(len > 5){ /* check if we have receive an ACK*/

          /* not an ACK => process the message */
          packetbuf_clear();

          len = dw1000_driver_read(packetbuf_dataptr(), PACKETBUF_SIZE);
          /* packetbuf_set_attr(PACKETBUF_ATTR_TIMESTAMP, last_packet_timestamp); */

          dw1000_update_frame_quality();

          /* end of the interrupt */ 
          /* See Figure 14: Flow chart for using double RX buffering
           * Of the manual */
#ifdef DOUBLE_BUFFERING
          status = dw_read_reg_64( DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS);
          if((status & DW_RXOVRR_MASK) > 0){ /* overrun may be occurs */
                  
            dw_idle();
            dw_trxsoft_reset();
            dw_change_rx_buffer();
            
            PRINTF("dw1000_process: RX Overrun case 2\n");
            if(receive_on)
              dw1000_on();
          }
          else{
            if(dw_good_rx_buffer_pointer()){
              dw_db_mode_clear_pending_interrupt();
            }
            dw_change_rx_buffer();
#endif /* DOUBLE_BUFFERING */
            packetbuf_set_datalen(len);
        
            PRINTF("dw1000_process: length %i\r\n", len);
            
            NETSTACK_RDC.input();

            dw_clear_pending_interrupt(DW_MRXFCE_MASK
                     | DW_MRXFCG_MASK
                     | DW_MRXDFR_MASK
                     | DW_MLDEDONE_MASK);
            dw1000_on();
#ifdef DOUBLE_BUFFERING
          }
#endif /* DOUBLE_BUFFERING */
        }
        else{
          /* we have receive an ACK 
            We just swap the buffer */
#ifdef DOUBLE_BUFFERING
          if(dw_good_rx_buffer_pointer()){
            dw_db_mode_clear_pending_interrupt();
          }
          dw_change_rx_buffer();
#else
          dw_clear_pending_interrupt(DW_MRXFCE_MASK
                   | DW_MRXFCG_MASK
                   | DW_MRXDFR_MASK
                   | DW_MLDEDONE_MASK);
          dw_init_rx();
#endif /* DOUBLE_BUFFERING */
          PRINTF("interrupt ACK\n");
        }
      } else{ 
        /**
         * Bad CRC, drop the packet > no read
         * Change the buffer pointer.
         */
#ifdef DOUBLE_BUFFERING
        if(dw_good_rx_buffer_pointer()){
          dw_db_mode_clear_pending_interrupt();
        }
        dw_change_rx_buffer();
#else
        dw_clear_pending_interrupt(DW_MRXFCE_MASK
                 | DW_MRXFCG_MASK
                 | DW_MRXDFR_MASK
                 | DW_MLDEDONE_MASK);
        dw_init_rx();
#endif /* DOUBLE_BUFFERING */
        printf("dw1000_process: bad CRC\n\r"); 
      }
      PRINTF("dw interrupt message\n");
      /* Read status register for the next iteration */
      status = dw_read_reg_64( DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS);
      /* print_sys_status(status)); // can be use to debug */
    }
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * \brief   Configure the transceiver for a wished channel and data rate.
 *
 *          Pulse Repetition Frequency set to 16MHz
 *
 *          Preamble length set to the minimum for data transfer:
 *          DW_DATA_RATE_110_KBPS  > 1024
 *          DW_DATA_RATE_850_KBPS  > 256
 *          DW_DATA_RATE_6800_KBPS > 128
 *
 * \param channel   The wished channel.
 *                  Available channel: 1, 2, 3, 4, 5, 7.
 *                  Channel 4 and 7 have a bandwidth of 999MHz.
 *                  Other channels have bandwidth of 500MHz.
 * \param dw1000_data_rate_t   The wished data rate.
 *                  Available data rate:
 *                  DW_DATA_RATE_110_KBPS
 *                  DW_DATA_RATE_850_KBPS
 *                  DW_DATA_RATE_6800_KBPS
 * \param dw1000_preamble_length_t The wished preamble length.
 *                  Recommended value:
 *                  At 110 kbps: 1024 if data, 2048 if ranging;
 *                  At 850 kbps: 256 if data, 512 if ranging;
 *                  At 6800 kbps: 128 if data, 256 if ranging.
 * \param dw1000_prf_t The wiched pulse repetition frequency.
 *                  DW_PRF_16_MHZ is best for data.
 *                  DW_PRF_64_MHZ is best for ranging: improve the first 
 *                  path detection, but increase the power consumption.
 *
 */
void
dw1000_driver_config(dw1000_channel_t channel, dw1000_data_rate_t data_rate, 
                      dw1000_preamble_length_t preamble_length, 
                      dw1000_prf_t prf)
{
  dw1000_conf.prf = prf;
  dw1000_conf.sfd_type = DW_SFD_STANDARD;
  dw1000_conf.preamble_length = preamble_length;

  dw1000_conf.channel = channel;
  dw1000_conf.preamble_code = dw1000_get_preamble_code(channel, prf);

  if(data_rate == DW_DATA_RATE_110_KBPS) {
    dw1000_conf.data_rate = DW_DATA_RATE_110_KBPS;
  } else if(data_rate == DW_DATA_RATE_850_KBPS) {
    dw1000_conf.data_rate = DW_DATA_RATE_850_KBPS;
  } else { /* 6800 kbps */
    dw1000_conf.data_rate = DW_DATA_RATE_6800_KBPS;
  }

  /* PAC size choose, based on the Table 6: Recommended PAC size */
  if(preamble_length < 256){
    dw1000_conf.pac_size = DW_PAC_SIZE_8;
  } else if(preamble_length < 1024){
    dw1000_conf.pac_size = DW_PAC_SIZE_16;
  } else if(preamble_length == 1024){
    dw1000_conf.pac_size = DW_PAC_SIZE_32;
  }else{
    dw1000_conf.pac_size = DW_PAC_SIZE_64;
  }

  if(preamble_length >= 512){
    /* avoid RX state bug: use the sniff mode with a 50/50 approach*/
    dw_set_snif_mode(1, 3, dw1000_conf.pac_size*3);
  }
  if(preamble_length == 256){
    /* bug fix: improve ranging reliability at 6.8 Mbps */
    dw1000_conf.pac_size = DW_PAC_SIZE_8;
  }

  if(data_rate == DW_DATA_RATE_110_KBPS) {
    /* 64 symbols */
    dw1000_conf.sfd_type = DW_SFD_NON_STANDARD;  
  } else if(data_rate == DW_DATA_RATE_850_KBPS) {
    /* 16 symbols */
    dw1000_conf.sfd_type = DW_SFD_NON_STANDARD;
  } else { /* 6800 kbps */
    /* 8 symbols */
    dw1000_conf.sfd_type = DW_SFD_STANDARD;
  }

  dw_set_default_antenna_delay(prf);

  dw_conf(&dw1000_conf);

  set_auto_ack(dw_is_automatic_ack());
}

/**
 * \Brief Configure the transceiver according a given TSCH channel.
 **/
void
dw1000_set_tsch_channel(uint8_t channel){
  switch(channel){
    case 0:
      dw1000_conf.channel = DW_CHANNEL_1;
      dw1000_conf.prf = DW_PRF_16_MHZ;
      break;
    case 1:
      dw1000_conf.channel = DW_CHANNEL_2;
      dw1000_conf.prf = DW_PRF_16_MHZ;
      break;
    case 2:
      dw1000_conf.channel = DW_CHANNEL_3;
      dw1000_conf.prf = DW_PRF_16_MHZ;
      break;
    case 3:
      dw1000_conf.channel = DW_CHANNEL_5;
      dw1000_conf.prf = DW_PRF_16_MHZ;
      break;
    case 4:
      dw1000_conf.channel = DW_CHANNEL_1;
      dw1000_conf.prf = DW_PRF_64_MHZ;
      break;
    case 5:
      dw1000_conf.channel = DW_CHANNEL_2;
      dw1000_conf.prf = DW_PRF_64_MHZ;
      break;
    case 6:
      dw1000_conf.channel = DW_CHANNEL_3;
      dw1000_conf.prf = DW_PRF_64_MHZ;
      break;
    case 7:
      dw1000_conf.channel = DW_CHANNEL_5;
      dw1000_conf.prf = DW_PRF_64_MHZ;
      break;
  }
  tsch_channel = channel;
  dw1000_conf.preamble_code = dw1000_get_preamble_code(dw1000_conf.channel, dw1000_conf.prf);

  dw_set_prf(dw1000_conf.prf);
  dw_set_channel(dw1000_conf.channel);
  dw_set_default_tx_power(dw1000_conf.channel, dw1000_conf.prf);
  dw_set_preamble_code(dw1000_conf.preamble_code);

  dw_lde_repc_config(dw1000_conf.preamble_code, dw1000_conf.data_rate);
  dw_configure_lde(dw1000_conf.preamble_code);

  dw_set_pac_size(dw1000_conf.pac_size, dw1000_conf.prf);

  dw_set_default_antenna_delay(dw1000_conf.prf);
}
/**
 * \Brief For a given channel and PRF return a preamble code.
 *  Based on the APH010 of DecaWave (section 5).
 **/
dw1000_preamble_code_t
dw1000_get_preamble_code(dw1000_channel_t channel, dw1000_prf_t prf){
  dw1000_preamble_code_t preamble_code;
  /* we define the preamble code to get the smallest the channel interference 
    radius according to the APH010 of DecaWave (section 5). */
  if(channel == DW_CHANNEL_1) {
    preamble_code = DW_PREAMBLE_CODE_1;
    if(prf == DW_PRF_64_MHZ)
      preamble_code = DW_PREAMBLE_CODE_12;
  } else if(channel == DW_CHANNEL_2) {
    preamble_code = DW_PREAMBLE_CODE_3;
    if(prf == DW_PRF_64_MHZ)
      preamble_code = DW_PREAMBLE_CODE_9;
  } else if(channel == DW_CHANNEL_3) {
    preamble_code = DW_PREAMBLE_CODE_5;
    if(prf == DW_PRF_64_MHZ)
      preamble_code = DW_PREAMBLE_CODE_9;
  } else if(channel == DW_CHANNEL_4) {
    preamble_code = DW_PREAMBLE_CODE_7;
    if(prf == DW_PRF_64_MHZ)
      preamble_code = DW_PREAMBLE_CODE_17;
  } else if(channel == DW_CHANNEL_5) {
    preamble_code = DW_PREAMBLE_CODE_3;
    if(prf == DW_PRF_64_MHZ)
      preamble_code = DW_PREAMBLE_CODE_9;
  } else { /* channel 7 */
    preamble_code = DW_PREAMBLE_CODE_7;
    if(prf == DW_PRF_64_MHZ)
      preamble_code = DW_PREAMBLE_CODE_17;
  }
  return preamble_code;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Return the time of the last SFD detection.
 *
 * \param sfd_register Specified if we return the last received SFD
 *                  or the last transmitted SFD
 *        Possible values : DW_REG_RX_TIME or DW_REG_TX_TIME
 **/
uint32_t
get_sfd_timestamp(uint32_t reg_addr)
{
  uint64_t sys_time = 0, sfd_time = 0, sfd_delay;
  uint32_t current_mcu_time;

  current_mcu_time = RTIMER_NOW();
  
  // SEND_SET();
  /* we use the DW1000 clock, We don't clean the 9 low bit because 
    RADIO_TO_RTIMER do this job. */
  sys_time = dw_read_reg_64(DW_REG_SYS_TIME, DW_LEN_SYS_TIME);
  // SEND_CLR();

  if(reg_addr == DW_REG_RX_TIME){
    dw_read_subreg(DW_REG_RX_TIME, DW_SUBREG_RX_RAWST, DW_SUBLEN_RX_RAWST, 
                    (uint8_t *) &sfd_time);
  }
  else{
    dw_read_subreg(DW_REG_TX_TIME, DW_SUBREG_TX_RAWST, DW_SUBLEN_TX_RAWST, 
                    (uint8_t *) &sfd_time);
  }


  sys_time &= DW_TIMESTAMP_MAX_VALUE;
  sfd_time &= DW_TIMESTAMP_MAX_VALUE;

  if(sys_time > sfd_time){
    sfd_delay = sys_time - sfd_time;
  }
  else {
    /* we have an overflow. To compute the delay we have to take the maximum 
    timestamps value into account (because this is a 40 bits number and 
    we use 64 bits number). */
    sfd_delay = sys_time + DW_TIMESTAMP_MAX_VALUE - sfd_time;
  }

#if DEBUG_VERBOSE
  printf("get_sfd_timestamp(void)\n");
  printf("current_time %lu\n", current_mcu_time);
  printf("SYS_TIME %llu\n", sys_time);
  printf("RX_TIME %llu\n", sfd_time);
  printf("sfd_delay %llu\n", sfd_delay);
  printf("RTIMER_ARCH_SECOND / DW_TIMESTAMP_CLOCK %d %d\n", 
                            RTIMER_ARCH_SECOND,  DW_TIMESTAMP_CLOCK);
  printf("RADIO_TO_RTIMER(sfd_delay) %lu %lu\n", RADIO_TO_RTIMER(sfd_delay), 
    RTIMERTICKS_TO_US(RADIO_TO_RTIMER(sfd_delay)));
#endif /*  DEBUG_VERBOSE */

  return current_mcu_time - US_TO_RTIMERTICKS(RADIO_TO_US(sfd_delay-DW1000_SFD_READOUT_OFFSET));
}
/*===========================================================================*/
/* Ranging                                                                   */
/*===========================================================================*/

/**
 * \brief Based on the delay (in us) and the RX timestamps, this function schedule 
 *        the ranging reply message.
 */
void 
dw1000_schedule_tx(uint16_t delay_us)
{
  uint64_t schedule_time = dw_get_rx_timestamp();
  /* require \ref note in the section 3.3 Delayed Transmission of the manual. */
  schedule_time &= DW_TIMESTAMP_CLEAR_LOW_9; /* clear the low order nine bits */
  /* The 10nd bit have a "value" of 125Mhz */
  schedule_time = schedule_time + US_TO_RADIO(delay_us); 

  dw_set_dx_timestamp(schedule_time);
  // printf("schedule tx time %llu\n", (schedule_time-(dw_get_rx_timestamp()&DW_TIMESTAMP_CLEAR_LOW_9)));

  dw1000_is_delayed_tx = 1;
  dw_init_tx(0,1);
}

/**
 * \brief Configures the DW1000 to be ready to receive a ranging response 
 *          based on the lasted sended messages. The delay is in micro seconds.
 *        /!\ Private function.
 */
void
dw1000_schedule_rx(uint16_t delay_us)
{
  uint64_t schedule_time = dw_get_tx_timestamp();
  /* require \ref note in the section 3.3 Delayed Transmission of the manual. */
  schedule_time &= DW_TIMESTAMP_CLEAR_LOW_9; /* clear the low order nine bits */
  /* The 10nd bit have a "value" of 125Mhz */
  schedule_time= schedule_time + US_TO_RADIO(delay_us);

  // printf("schedule rx time %llu\n", (schedule_time-(dw_get_tx_timestamp()&DW_TIMESTAMP_CLEAR_LOW_9)));

  dw_set_dx_timestamp(schedule_time);
  dw_init_delayed_rx();
}


/**
 * \brief Update the quality information about the last packet received.
 */
void 
dw1000_update_frame_quality(void){
  dw_get_receive_quality(&last_packet_quality);
}

/**
 * \brief Return the quality of the last received packet.
 */
dw1000_frame_quality
dw1000_driver_get_packet_quality(void){
  return last_packet_quality;
}