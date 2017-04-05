/*
 * Copyright (c) 2016, UMons University.
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
#include "dw1000-util.h"

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

/** Set the ranging reply time (in µs).
 * For a configuration using a data-rate of 6.8 mbps and a preamble of 128 
 * symbols, we recommend a reply time of 600 µs. 
 * At 110 kbps and with a preamble of 1024 symbols, a replay time of 2300 µs
 * is enough 
 */
#ifndef DW1000_RANGING_REPLY_TIME
#define DW1000_RANGING_REPLY_TIME               0
#endif /* DW1000_RANGING_REPLY_TIME */

/* the delay induced by the SPI communication */
#define DW1000_SPI_DELAY        50l 
/* max delay between RX to TX and TX to RX time*/
#define IEEE802154_TURN_ARROUND_TIME 10l 

/* Time take by the receiver to make the ranging response and to schedule it */
/* this is platform dependent */
#define DW1000_REPLY_TIME_COMPUTATION 750

#if DW1000_IEEE802154_EXTENDED
#define DW1000_MAX_PACKET_LEN 265
#else
#define DW1000_MAX_PACKET_LEN 127
#endif

/* the length max of a ranging response message. 
 * Min value is 11: 9 for the header and 2 for the FCS */
#define DW1000_RANGING_MAX_LEN 11

#define DW1000_RANGING_FINAL_LEN 17
/* #define DEBUG_RANGING 1 */

#define DEBUG_VERBOSE 0
#if DEBUG_VERBOSE
#define DEBUG 1
#endif
#ifndef DEBUG
#define DEBUG 0
#endif
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while(0)
#endif

#define DEBUG_LED 1

// #define DOUBLE_BUFFERING

/* Used to fix an error with an possible interruption before
   the driver initialization */
static int dw1000_driver_init_down = 0;

/* Used to avoid multiple interrupt in ranging mode */
static int volatile dw1000_driver_in_ranging = 0;

/* define if transmission wait for an ACK. */
static int dw1000_driver_wait_ACK = 0;
static int dw1000_driver_wait_ACK_num = 0;

/* define if we are in the Single-sided Two-way Ranging protocol */
static uint8_t volatile dw1000_driver_sstwr = 0;

/* define if we are in the Symmetric double-sided two-way ranging protocol */
static uint8_t volatile dw1000_driver_sdstwr = 0;

/* Define the reply time (in micro second). 
 */
static uint32_t dw1000_driver_reply_time;

/* Define the real reply time
 * Note this value match with the clock of the DW1000 (125 MHz ~ 0.008µs) 
 */
static uint32_t dw1000_driver_schedule_reply_time;

static int32_t dw1000_driver_last_prop_time;

/* store the current DW1000 configuration */
static dw1000_base_conf_t dw1000_conf;
static dw1000_frame_quality last_packet_quality;

static uint8_t volatile pending;

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

volatile uint8_t dw1000_driver_sfd_counter;
volatile uint16_t dw1000_driver_sfd_start_time;
volatile uint16_t dw1000_driver_sfd_end_time;

static volatile uint16_t last_packet_timestamp;

/* start private function */
inline void dw1000_schedule_reply(void);
void dw1000_schedule_reply2(void);
void dw1000_compute_prop_time_sdstwr(void);
void dw1000_compute_prop_time_sstwr(void);
inline void  dw1000_update_frame_quality(void);
uint8_t ranging_send_ack_sheduled(uint8_t wait_for_resp, uint8_t wait_send);
uint16_t convert_payload_len(uint16_t payload_len);
/* end private function */

/* PLATFORM DEPENDENT
 * these following functions are defined in 
 * platform/[platform]/dev/dw1000-arc.c 
 */
void dw1000_arch_init(void);
/* end PLATFORM DEPENDENT */

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

/* functions defined in platform/[platform]/dev/dw1000-arc.c */
void dw1000_us_delay(int ms);
void dw_read_subreg(uint32_t reg_addr, uint16_t subreg_addr, 
                    uint16_t subreg_len, uint8_t *p_data);
void dw_write_subreg(uint32_t reg_addr, uint16_t subreg_addr, 
                    uint16_t subreg_len, const uint8_t *data);

/*---------------------------------------------------------------------------*/
PROCESS(dw1000_driver_process, "DW1000 driver");
PROCESS(dw1000_driver_process_sds_twr, "DW1000 driver SDS TWR");
PROCESS(dw1000_driver_process_ss_twr, "DW1000 driver SS TWR");
/*---------------------------------------------------------------------------*/

signed char dw1000_driver_last_rssi;
uint8_t dw1000_driver_last_correlation;

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

static uint8_t receive_on;

/*---------------------------------------------------------------------------*/
static uint8_t locked, lock_on, lock_off;


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

  /* Check if SPI communication works by reading device ID */
  assert(0xDECA0130 == dw_read_reg_32(DW_REG_DEV_ID, DW_LEN_DEV_ID));

  /* Simple reset of device. */
  dw_soft_reset(); /* Need to be call with a SPI speed < 3MHz */

  /* clear all interrupt */
  dw_clear_pending_interrupt(0x00000007FFFFFFFFULL);

  /* load the program to compute the timestamps */
  dw_load_lde_code(); /* Need to be call with a SPI speed < 3MHz */

#if DW1000_IEEE802154_EXTENDED
  PRINTF("DW1000 set to use IEEE 802.15.4-2011 UWB non-standard mode, ");
  PRINTF("extended frame max 265 bytes.\r\n");
  dw_enable_extended_frame();
#else
  PRINTF("DW1000 set to use IEEE 802.15.4-2011 UWB standard mode.\r\n");
  dw_disable_extended_frame();
#endif

  dw1000_driver_config(DW1000_CHANNEL, DW1000_DATA_RATE, DW1000_PREAMBLE, 
                        DW1000_PRF);
  printf("channel %d, data rate %d, preamble %d, prf %d\n", 
                (unsigned int) DW1000_CHANNEL, 
                (unsigned int) DW1000_DATA_RATE, 
                (unsigned int) DW1000_PREAMBLE, 
                (unsigned int) DW1000_PRF);

  dw_disable_rx_timeout();

  /* dw1000_driver_set_pan_addr is recall after by Contiki. */
  dw1000_driver_set_pan_addr(0xffff, 0x0000, NULL);

#ifdef DOUBLE_BUFFERING
  dw_enable_double_buffering();
#else
  dw_enable_automatic_receiver_Re_Enable();
#endif /* DOUBLE_BUFFERING */

#if DEBUG_LED
  dw_enable_gpio_led();
#else
  dw_disable_gpio_led();
#endif
  
  enable_error_counter(); /* /!\ Increase the power consumption. */

  dw1000_driver_set_reply_time(DW1000_RANGING_REPLY_TIME);

  /* because in some case the ranging request bit TR is TRUE */
  dw_disable_ranging_frame();

  process_start(&dw1000_driver_process, NULL);
  process_start(&dw1000_driver_process_ss_twr, NULL);
  process_start(&dw1000_driver_process_sds_twr, NULL);

  dw1000_driver_init_down = 1;
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

  uint16_t data_len = payload_len;
#if DW1000_CONF_AUTOACK
  dw1000_driver_wait_ACK = (((uint8_t *)payload)[0] & (1 << 5)) ? 1 : 0;
  if(dw1000_driver_wait_ACK) {
    dw1000_driver_wait_ACK_num = ((uint8_t *)payload)[2];
  }
#endif

  RIMESTATS_ADD(lltx);

  if(dw1000_driver_sstwr || dw1000_driver_sdstwr){
    PRINTF("Ranging request\n");
    /* we can not wait for an ACK if we are in a ranging protocol */
#if DW1000_CONF_AUTOACK
    if(dw1000_driver_wait_ACK){
      ((uint8_t *)payload)[0] &= ~(1UL << 5);
    }
#endif
#if !DW1000_IEEE802154_EXTENDED
    /* In standard mode we use the RNG bit in the PHR. */
    dw_enable_ranging_frame();
#endif
    payload_len = convert_payload_len(payload_len);
    data_len = payload_len;
  }

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

#if DW1000_IEEE802154_EXTENDED
  /* Just replace the 10nd bytes by a value of "0x0" */
  if(dw1000_driver_sstwr){
    uint8_t ranging_value = 0x0;
    /* 10nd bytes */
    dw_write_subreg(DW_REG_TX_BUFFER, 0x9, 1, &ranging_value);
  }
#endif
  if(dw1000_driver_sdstwr){
    uint8_t ranging_value = 0x01;
    /* 10nd bytes */
    dw_write_subreg(DW_REG_TX_BUFFER, 0x9, 1, &ranging_value);
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

  return 0;
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
  PRINTF("dw1000_driver_transmit ACK %d Ranging %d\r\n",
              dw1000_driver_wait_ACK,
              dw1000_driver_sstwr);

  int tx_return = RADIO_TX_ERR;
#if DEBUG
  uint8_t count_idle = 0, count_txtrt = 0;
#endif /* DEBUG */

  /* abort reception and disable interrupt to be able to send a frame */
  if(receive_on) {
    dw_idle();
  }

  if(dw1000_driver_wait_ACK | dw1000_driver_sstwr | dw1000_driver_sdstwr) {
    dw1000_driver_disable_interrupt();  
    dw1000_driver_clear_pending_interrupt();
  }

  ENERGEST_ON(ENERGEST_TYPE_TRANSMIT);

  /* if we are in ranging, the size change. */
  payload_len = convert_payload_len(payload_len);


  if(!(dw1000_driver_sstwr || dw1000_driver_sdstwr)){
    /* Initialize a no delayed transmission 
      and wait for an ACK if an ACK request is triggered */
    dw_init_tx(dw1000_driver_wait_ACK, 0); 
  } else{
    /* Initialize a no delayed transmission and wait for an ranging response 
      Re-enable the RX state after the transmission. */
    dw_init_tx(1, 0); 
  }

#if DEBUG
  uint8_t tr_value;
  /* TR bit is the 15nd bit */
  dw_read_subreg(DW_REG_TX_FCTRL, 0x1, 1, &tr_value); 
  printf("Ranging request send: %d\r\n", (tr_value & (DW_TR_MASK >> 8)) > 0);

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
                  (theorical_transmission_approx(dw1000_conf.preamble_length, 
                  dw1000_conf.data_rate, dw1000_conf.prf, payload_len) << 1 )+ 
                  DW1000_SPI_DELAY);

  PRINTF("Number of loop waiting IDLE: %d\n", count_idle);
  PRINTF("Number of loop waiting Tx on: %d\n", count_txtrt);
  PRINTF("Number of loop waiting Transmit Frame Sent: %d\n", count_send);
  
  if((sys_status_lo & DW_TXFRS_MASK) != 0) {
    tx_return = RADIO_TX_OK;
  }
  if(dw1000_driver_sstwr || dw1000_driver_sdstwr){
#if !DW1000_IEEE802154_EXTENDED
    /* In standard mode, we disable the RNG bit in the PHR for the next frame.*/
    dw_disable_ranging_frame();
#endif
  }

  /* start WAIT ACK */
  if(dw1000_driver_wait_ACK && tx_return == RADIO_TX_OK) {
    tx_return = RADIO_TX_NOACK;
    uint8_t count_ack = 0;
    sys_status_lo = 0x0; /* clear the value */

    /* wait the ACK */
    BUSYWAIT_UPDATE_UNTIL(dw_read_subreg(DW_REG_SYS_STATUS, 0x1, 1, 
                    &sys_status_lo); watchdog_periodic(); count_ack++,
                    ((sys_status_lo & ((DW_RXFCG_MASK >> 8) | 
                    (DW_RXFCE_MASK >> 8))) != 0),
                    (theorical_transmission_approx(dw1000_conf.preamble_length,
                    dw1000_conf.data_rate, dw1000_conf.prf, DW_ACK_LEN) << 1) + 
                    DW1000_SPI_DELAY + IEEE802154_TURN_ARROUND_TIME);

    PRINTF("Number of loop waiting ACK %d\n", count_ack);

    if((sys_status_lo & (DW_RXFCG_MASK >> 8)) != 0) {
      /* (length ACK== 5) and (Sequence Number, 3rd byte == ACK number) */
      if(dw_get_rx_len() == DW_ACK_LEN) {
        uint8_t ack_num;
        dw_read_subreg(DW_REG_RX_BUFFER, 0x2, 1, &ack_num);
        if(ack_num == dw1000_driver_wait_ACK_num) {
          tx_return = RADIO_TX_OK;
        }
      }
      dw1000_update_frame_quality();
    }
  } /* end WAIT ACK */

  /* we wait for a ranging response */
  if(dw1000_driver_sstwr && tx_return == RADIO_TX_OK){ /* start SS TWR */
    tx_return = RADIO_TX_ERR;
    uint8_t count = 0;
    sys_status_lo = 0x0; /* clear the value */

    /* wait for a ranging response */
    /* the length of the ranging response is 5 */
    BUSYWAIT_UPDATE_UNTIL(dw_read_subreg(DW_REG_SYS_STATUS, 0x1, 1, 
                    &sys_status_lo); watchdog_periodic(); count++,
                    (((sys_status_lo & (DW_RXDFR_MASK >> 8)) != 0) &&
                    ((sys_status_lo & ((DW_RXFCG_MASK >> 8) | 
                    (DW_RXFCE_MASK >> 8))) != 0)), 
                    theorical_transmission_approx(dw1000_conf.preamble_length,
                      dw1000_conf.data_rate, dw1000_conf.prf, DW_ACK_LEN) + 
                      DW1000_SPI_DELAY + IEEE802154_TURN_ARROUND_TIME + 
                      dw1000_driver_reply_time);
    PRINTF("Number of loop waiting the ranging response %d\n", count);

    if((sys_status_lo & (DW_RXFCG_MASK >> 8)) != 0) {
      tx_return = RADIO_TX_OK;

#if DEBUG
      PRINTF("length of the ranging response %d\n", dw_get_rx_len());
      PRINTF("waiting time %d\n", (unsigned long int ) 
              (theorical_transmission_approx(dw1000_conf.preamble_length,
              dw1000_conf.data_rate, dw1000_conf.prf, DW1000_RANGING_MAX_LEN) + 
              DW1000_SPI_DELAY + IEEE802154_TURN_ARROUND_TIME + 
              DW1000_REPLY_TIME_COMPUTATION));
#endif /* DEBUG */

      /* compute the propagation time with the clock offset correction */
      dw1000_compute_prop_time_sstwr();

      dw1000_update_frame_quality();
    }
  } /* end SS TWR */

  /* start SDS TWR */
  /* we wait for the ranging response in symmetric mode */
  if(dw1000_driver_sdstwr && tx_return == RADIO_TX_OK){
    tx_return = RADIO_TX_ERR;
    uint8_t count = 0;
    sys_status_lo = 0x0; /* clear the value */

    /* we wait for the first ranging response */
    BUSYWAIT_UPDATE_UNTIL(dw_read_subreg(DW_REG_SYS_STATUS, 0x1, 1, 
                    &sys_status_lo); watchdog_periodic(); count++,
                    (((sys_status_lo & (DW_RXDFR_MASK >> 8)) != 0) &&
                    ((sys_status_lo & ((DW_RXFCG_MASK >> 8) | 
                    (DW_RXFCE_MASK >> 8))) != 0)), 
                    theorical_transmission_approx(dw1000_conf.preamble_length,
                      dw1000_conf.data_rate, dw1000_conf.prf, DW_ACK_LEN) + 
                      DW1000_SPI_DELAY + IEEE802154_TURN_ARROUND_TIME + 
                      dw1000_driver_reply_time);

    PRINTF("Number of loop waiting the ranging response %d\n", count);
    /* we have receive the ranging response, now we compute the t_trop 
      and we send a ranging response*/
    if((sys_status_lo & (DW_RXFCG_MASK >> 8)) != 0) {

      dw_idle(); /* receiver off */
      
      uint64_t t_round_I = dw_get_rx_timestamp() - dw_get_tx_timestamp();
      /* use further to compute the real reply time */
      uint64_t rx_timestamp0 = dw_get_rx_timestamp();

      /* wait for the ranging response, with the t_prop on the receiver*/
      ranging_send_ack_sheduled(1, 0);

      /* clear the sys status */
      dw1000_driver_clear_pending_interrupt();

#ifdef DOUBLE_BUFFERING
      if(dw_good_rx_buffer_pointer()) {
        dw_db_mode_clear_pending_interrupt();
      }
      dw_change_rx_buffer();
#endif /* DOUBLE_BUFFERING */

      sys_status_lo = 0x0;
      /* wait the end of the transmission */
      BUSYWAIT_UPDATE_UNTIL(dw_read_subreg(DW_REG_SYS_STATUS, 0x0, 1, 
                &sys_status_lo); watchdog_periodic();,
                ((sys_status_lo & DW_TXFRS_MASK) != 0),
                theorical_transmission_approx(dw1000_conf.preamble_length, 
                dw1000_conf.data_rate, dw1000_conf.prf, DW_ACK_LEN) + 
                DW1000_SPI_DELAY + dw1000_driver_reply_time);

      /* clear the sys status */
      dw1000_driver_clear_pending_interrupt();

      /* ranging response send OK */
      if((sys_status_lo & DW_TXFRS_MASK) != 0) {
        /* compute the difference between the expected reply time and 
          the real reply time to correct further the propagation time 
          in the sender side.*/
        uint16_t t_reply_corrector_sender = dw_get_tx_timestamp() 
                                  - rx_timestamp0 
                                  - dw1000_driver_schedule_reply_time;

        PRINTF("dw_get_tx_timestamp %llu\n", dw_get_tx_timestamp());
        PRINTF("rx_timestamp0 %llu\n", rx_timestamp0);
        PRINTF("dw1000_driver_schedule_reply_time %lu\n", 
                                            dw1000_driver_schedule_reply_time);
        
        PRINTF("send ok\n");

        /* the length of the ranging response is 17 */
        count = 0;
        BUSYWAIT_UPDATE_UNTIL(dw_read_subreg(DW_REG_SYS_STATUS, 0x1, 1, 
                      &sys_status_lo); watchdog_periodic(); count++,
                      (((sys_status_lo & (DW_RXDFR_MASK >> 8)) != 0) &&
                      ((sys_status_lo & ((DW_RXFCG_MASK >> 8) | 
                      (DW_RXFCE_MASK >> 8))) != 0)), 
                      theorical_transmission_approx(dw1000_conf.preamble_length,
                        dw1000_conf.data_rate, dw1000_conf.prf, 
                        DW1000_RANGING_FINAL_LEN) + 
                        DW1000_SPI_DELAY + IEEE802154_TURN_ARROUND_TIME + 
                        dw1000_driver_reply_time);
        
#if DEBUG
        PRINTF("ko %d %02X\n", count, sys_status_lo);
        PRINTF("time interrupt %d\n", 
                    theorical_transmission_approx(dw1000_conf.preamble_length,
                          dw1000_conf.data_rate, dw1000_conf.prf, 15) + 
                          DW1000_SPI_DELAY + IEEE802154_TURN_ARROUND_TIME + 
                          dw1000_driver_reply_time);

        print_sys_status((uint64_t) (sys_status_lo) << 8);

        print_sys_status(dw_read_reg_64(DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS));
#endif /* DEBUG */


        /* we received the ranging response of the receiver and we made the 
            correction on the t_prop */
        if((sys_status_lo & (DW_RXFCG_MASK >> 8)) != 0) {
          /* check if the message have the good size */
          if(dw_get_rx_extended_len() == DW1000_RANGING_FINAL_LEN){
            int32_t t_propReceiver;
            dw_read_subreg(DW_REG_RX_BUFFER, 0x9, 4, 
                                                    (uint8_t*) &t_propReceiver);
            uint16_t t_reply_corrector_receiver;
            dw_read_subreg(DW_REG_RX_BUFFER, 0xD, 2, 
                                        (uint8_t*) &t_reply_corrector_receiver);

            // printf("t_round_I %llu\n", t_round_I);
            uint32_t t_reply_R = dw1000_driver_schedule_reply_time 
                                + t_reply_corrector_receiver;
            // printf("t_reply_corrector_receiver %u\n", t_reply_corrector_receiver);
            // printf("dw1000_driver_schedule_reply_time %lu\n", dw1000_driver_schedule_reply_time);

            // printf("t_propReceiver %ld\n", t_propReceiver);
            // printf("t_reply_corrector_sender %u\n", t_reply_corrector_sender);
            
            dw1000_driver_last_prop_time = t_round_I - t_reply_R;
            // printf("dw1000_driver_last_prop_time %ld\n", dw1000_driver_last_prop_time);
            dw1000_driver_last_prop_time = dw1000_driver_last_prop_time;
            // printf("dw1000_driver_last_prop_time %ld\n", dw1000_driver_last_prop_time);

            // printf("t prop %4X\n", (unsigned int) t_propReceiver);
            // printf("prop %u\n", (unsigned int) dw1000_driver_last_prop_time);
            dw1000_driver_last_prop_time += t_propReceiver
                                          - t_reply_corrector_sender;
            // printf("dw1000_driver_last_prop_time %ld\n", dw1000_driver_last_prop_time);
            // printf("prop %u\n",(unsigned int)  dw1000_driver_last_prop_time);
            dw1000_driver_last_prop_time = dw1000_driver_last_prop_time / 4;
            // printf("dw1000_driver_last_prop_time %ld\n", dw1000_driver_last_prop_time);
            // printf("prop %u\n", (unsigned int) dw1000_driver_last_prop_time);

#if DEBUG
            count = 0;
            BUSYWAIT_UPDATE_UNTIL(dw_read_subreg(DW_REG_SYS_STATUS, 0x1, 1, 
                          &sys_status_lo); watchdog_periodic(); count++,
                          ((sys_status_lo & (DW_LDEDONE_MASK >> 8)) != 0), 
                          30);
            PRINTF("Number of loop waiting the LDE done %d\n", count);
            PRINTF("length of the ranging response %d\n", dw_get_rx_len());
#endif /* DEBUG */

            dw1000_update_frame_quality();
            PRINTF("SS TWR end correctly\n");

            tx_return = RADIO_TX_OK;
          }
        }
      }
    }
  } /* end SDS TWR */


  if(dw1000_driver_wait_ACK | dw1000_driver_sstwr | dw1000_driver_sdstwr) {
    dw_idle();  /* bug fix of waiting an ACK which
                   avoid the next transmission */
    dw1000_driver_enable_interrupt();
  }

  /* disable the ranging bit in the PHY header for the next transmission. */
  if(dw1000_driver_sstwr || dw1000_driver_sdstwr){
    /* disable ranging request */
    dw1000_driver_sstwr = 0;
    dw1000_driver_sdstwr = 0;
  }

  /* re-enable the RX state
      clean the interrupt
      and change the RX buffer*/
  if(receive_on){
    dw1000_on();
  }
  else{
    dw1000_driver_clear_pending_interrupt();
  }

  ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);

#if DEBUG_VERBOSE
  print_sys_status(dw_read_reg_64(DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS));
  if(tx_return == RADIO_TX_OK) {
    PRINTF("TX RADIO_TX_OK \r\n");
  }
#endif
#if DEBUG
  if(tx_return == RADIO_TX_NOACK) {
    printf("TX RADIO_TX_NOACK \r\n");
  }
  if(tx_return == RADIO_TX_ERR) {
    printf("TX RADIO_TX_ERR \r\n");
  }
#endif
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
 * \brief     Reads a pending packet from the radio.
 *
 * \return    The length of the read packet.
 */
static int
dw1000_driver_read(void *buf, unsigned short bufsize)
{

  PRINTF("dw1000_driver_read\r\n");

#if DW1000_IEEE802154_EXTENDED
  int len = dw_get_rx_extended_len();
#else
  int len = dw_get_rx_len();
#endif

  if(len > DW1000_MAX_PACKET_LEN) {
    RIMESTATS_ADD(toolong);
    return 0;
  }

  if(len <= FOOTER_LEN) {
    RIMESTATS_ADD(tooshort);
    return 0;
  }

  if(len - FOOTER_LEN > bufsize) {
    RIMESTATS_ADD(toolong);
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

  pending--;
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
  uint64_t status = dw_read_reg_64(DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS);
  return dw_is_receive_status(status);
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
  PRINTF("dw1000_driver_receiving_packet\r\n");
  return pending > 0;
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
  PRINTF("dw1000_driver_on\r\n");
  if(receive_on) {
    return 1;
  }
  if(locked) {
    lock_on = 1;
    return 1;
  }

  dw1000_on();
  return 1;
}
/**
 * \brief Turn the radio on.
 */
void
dw1000_on(void)
{
  dw1000_driver_enable_interrupt();

#ifdef DOUBLE_BUFFERING
  if(!dw_good_rx_buffer_pointer()) { /* check HSRBP == ICRBP */
    /* Host Side Receive Buffer Pointer Toggle to 1. */
    dw_change_rx_buffer();
  }
#endif /* DOUBLE_BUFFERING */

  uint32_t sys_ctrl;
  dw_read_subreg(DW_REG_SYS_CTRL, 0x0, 4, (uint8_t *) &sys_ctrl);
  if(sys_ctrl != 0)
    printf("init rx Sys ctrl value %lu\n", sys_ctrl);
  
  dw_idle();
  uint8_t sys_ctrl_val;
  BUSYWAIT_UPDATE_UNTIL(dw_read_reg(DW_REG_SYS_CTRL, 1, &sys_ctrl_val); 
            watchdog_periodic();,
            ((sys_ctrl_val & ((1 << DW_TRXOFF) & DW_TRXOFF_MASK)) == 0),
            500);
  dw_init_rx();
  /* The receiver has a delay of 16μs after issuing the enable receiver command,
   *  after which it will start receiving preamble symbols. */
  dw1000_us_delay(16);

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
  uint64_t status = dw_read_reg_64(DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS);
  if(((status & DW_RXSFDD_MASK) > 0) 
    && ((status & (DW_RXDFR_MASK | DW_RXPHE_MASK)) == 0)) {
    lock_off = 1;
  } else {
    dw1000_off();
  }
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
    *value = DW1000_CHANNEL;
    return RADIO_RESULT_OK;
  case RADIO_PARAM_RX_MODE:
    *value = 0;
    /* radio not set to filtering frame */
    /*  *value |= RADIO_RX_MODE_ADDRESS_FILTER; */
    if(DW1000_CONF_AUTOACK) {
      *value |= RADIO_RX_MODE_AUTOACK;
    }
    /* Radio not support poll */
    /*   *value |= RADIO_RX_MODE_POLL_MODE; */
    return RADIO_RESULT_OK;
  case RADIO_PARAM_TX_MODE:
    /* radio not support CCA */
    *value = 0;
    return RADIO_RESULT_OK;
  case RADIO_PARAM_TXPOWER:
    return RADIO_RESULT_NOT_SUPPORTED;
  case RADIO_PARAM_CCA_THRESHOLD:
    return RADIO_RESULT_NOT_SUPPORTED;
  case RADIO_PARAM_RSSI:
    /* Return the RSSI value in dBm */
    return RADIO_RESULT_NOT_SUPPORTED;
  case RADIO_PARAM_LAST_RSSI:
    /* RSSI of the last packet received */
    return RADIO_RESULT_NOT_SUPPORTED;
  case RADIO_PARAM_LAST_LINK_QUALITY:
    /* LQI of the last packet received */
    return RADIO_RESULT_NOT_SUPPORTED;
  case RADIO_CONST_CHANNEL_MIN:
    *value = 1;
    return RADIO_RESULT_OK;
  case RADIO_CONST_CHANNEL_MAX:
    *value = 7;
    return RADIO_RESULT_OK;
  case RADIO_CONST_TXPOWER_MIN:
    return RADIO_RESULT_NOT_SUPPORTED;
  case RADIO_CONST_TXPOWER_MAX:
    return RADIO_RESULT_NOT_SUPPORTED;
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
    if(value < 1 || value > 7) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    /* TODO add support */
    /* dw1000_driver_config(value, dw1000_conf.data_rate); */
    return RADIO_RESULT_NOT_SUPPORTED;
  case RADIO_PARAM_RX_MODE:
    /* TODO add support */
    /* if(value & ~(RADIO_RX_MODE_ADDRESS_FILTER |
         RADIO_RX_MODE_AUTOACK | RADIO_RX_MODE_POLL_MODE)) {
       return RADIO_RESULT_INVALID_VALUE;
       }
       set_frame_filtering((value & RADIO_RX_MODE_ADDRESS_FILTER) != 0);
       set_auto_ack((value & RADIO_RX_MODE_AUTOACK) != 0);
       set_poll_mode((value & RADIO_RX_MODE_POLL_MODE) != 0);*/
    return RADIO_RESULT_NOT_SUPPORTED;
  case RADIO_PARAM_TX_MODE:
    return RADIO_RESULT_NOT_SUPPORTED;
  case RADIO_PARAM_TXPOWER:
    return RADIO_RESULT_NOT_SUPPORTED;
  case RADIO_PARAM_CCA_THRESHOLD:
    return RADIO_RESULT_NOT_SUPPORTED;
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
dw1000_driver_get_object(radio_param_t param,
                         void *dest, size_t size)
{
  PRINTF("dw1000_driver_get_object\r\n");
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
  PRINTF("dw1000_driver_set_object\r\n");
  return RADIO_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/

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
  if(dw1000_driver_init_down && !dw1000_driver_in_ranging) {
    /* we read only the first 32 bit of the register */
    uint32_t status = dw_read_reg_64(DW_REG_SYS_STATUS, 4);

#ifdef DOUBLE_BUFFERING
    if(status & DW_RXOVRR_MASK) {  /* Overrun */
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
    if(status & DW_RXDFR_MASK){ /* frame ready */
 
 #if DW1000_IEEE802154_EXTENDED
      uint8_t ranging_value; /* use to check if the frame is a ranging frame */
      /* read the 10nd bytes */
      dw_read_subreg(DW_REG_RX_BUFFER, 0x9, 1, &ranging_value);
      dw1000_driver_in_ranging = (dw_get_rx_extended_len() == 12);
#else
      dw1000_driver_in_ranging = dw_is_ranging_frame();
#endif
      if(dw1000_driver_in_ranging){

        /* We made a new packet for the response
           And we program the response */
        uint8_t ranging_mode = 0x0; /* use yo get the ranging mode */
        /* read the 10nd bytes */
        if(dw_get_rx_extended_len() == 12){
          dw_read_subreg(DW_REG_RX_BUFFER, 0x9, 1, &ranging_mode);
        }
        if (ranging_mode == 0x0){/* SS TWR */
          process_poll(&dw1000_driver_process_ss_twr);
        }
        else if (ranging_mode == 0x01){ /* SDS TWR */
          process_poll(&dw1000_driver_process_sds_twr);
        }
        else{
          /* not a correct mode we exit the ranging mode */
          dw1000_driver_in_ranging = 0;
        }
      }
      else{    
      /* receiver Data Frame Ready. */
      process_poll(&dw1000_driver_process);
      pending++;
      }
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
  int len;
  PROCESS_BEGIN();

  PRINTF("dw1000_process: started\r\n");
  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

    /* We are not interested by the 5nd bytes */
    uint32_t status = dw_read_reg_32( DW_REG_SYS_STATUS, 4);
    while (
      /* if overrun or good message */
#ifdef DOUBLE_BUFFERING
      (status & DW_RXOVRR_MASK) ||
#endif /* DOUBLE_BUFFERING */
      (status & DW_RXDFR_MASK)){

      PRINTF("dw1000_process: calling receiver callback\r\n");

#ifdef DOUBLE_BUFFERING
      if(status & DW_RXOVRR_MASK){
        dw_idle();
        dw_trxsoft_reset();
        dw_change_rx_buffer();

        PRINTF("dw1000_process: RX Overrun case 1\n");
        if(receive_on)
          dw1000_on();
      }
      else
#endif /* DOUBLE_BUFFERING */
      if(status & DW_RXFCG_MASK) { // no overrun and good CRC
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
          if(status & DW_RXOVRR_MASK){ /* overrun may be occurs */
                  
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
/* process used in case of Symmetric double-sided two-way ranging (SDS-TWR) */
PROCESS_THREAD(dw1000_driver_process_sds_twr, ev, data){
  PROCESS_BEGIN();

  PRINTF("dw1000_driver_process_sds_twr: started\r\n");
  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

    /* we can not send the response if we are in RX mode */
    dw_idle();

    dw1000_driver_clear_pending_interrupt();

    /* use further to compute the real reply time */
    uint64_t rx_timestamp0 = dw_get_rx_timestamp();

#if !DW1000_IEEE802154_EXTENDED
    if(dw_is_ranging_frame()){
      /* In standard mode, we disable the RNG bit in the PHR for the response.*/
      dw_disable_ranging_frame();
    }
#endif

    /* send the ranging response to compute the t prop 
      wait for response.
      don't wait the end of the send, we have to work...*/
    ranging_send_ack_sheduled(1, 0);

    /* we save the frame of the initiator to make the response later */
    uint8_t initiator_frame[9];
    dw_read_subreg(DW_REG_RX_BUFFER, 0x0, 9, initiator_frame);

    dw1000_update_frame_quality();

#ifdef DOUBLE_BUFFERING
    if(dw_good_rx_buffer_pointer()){
      dw_db_mode_clear_pending_interrupt();
    }
    dw_change_rx_buffer();
#endif /* ! DOUBLE_BUFFERING */

    uint32_t sys_status = 0x0;
    /* wait the end of the transmission */
    BUSYWAIT_UPDATE_UNTIL(dw_read_subreg(DW_REG_SYS_STATUS, 0x0, 4, 
              (uint8_t*) &sys_status); watchdog_periodic();,
              ((sys_status & (DW_TXFRS_MASK | DW_HPDWARN_MASK))  != 0),
              theorical_transmission_approx(dw1000_conf.preamble_length, 
              dw1000_conf.data_rate, dw1000_conf.prf, DW_ACK_LEN) + 
              DW1000_SPI_DELAY + dw1000_driver_reply_time);
    /* HPDWARN => we abort the send */
    if((sys_status & DW_HPDWARN_MASK) != 0){
      dw_idle();
      printf("process_sds_twr HPDWARN\n");
    }

    dw1000_driver_clear_pending_interrupt();

    /* ranging response send OK */
    if((sys_status & DW_TXFRS_MASK) != 0) {
      /* compute the difference between the expected reply time and 
            the real reply time to correct further the propagation time 
            in the sender side.*/
      uint16_t t_reply_corrector = dw_get_tx_timestamp() - rx_timestamp0 
                                    - dw1000_driver_schedule_reply_time;
      uint8_t sys_status_lo = 0x0;
      /* wait the ranging response */
      BUSYWAIT_UPDATE_UNTIL(dw_read_subreg(DW_REG_SYS_STATUS, 0x1, 1, 
            &sys_status_lo); watchdog_periodic();,
            (((sys_status_lo & (DW_RXDFR_MASK >> 8)) != 0) &&
            ((sys_status_lo & ((DW_RXFCG_MASK >> 8) | 
            (DW_RXFCE_MASK >> 8))) != 0)), 
            theorical_transmission_approx(dw1000_conf.preamble_length,
              dw1000_conf.data_rate, dw1000_conf.prf, DW_ACK_LEN) + 
              DW1000_SPI_DELAY + IEEE802154_TURN_ARROUND_TIME + 
              dw1000_driver_reply_time);

      /* clear the sys status */
      dw1000_driver_clear_pending_interrupt();

      /* we receive the ranging response */
      if((sys_status_lo & (DW_RXFCG_MASK >> 8)) != 0) {

        dw_idle();

        dw1000_compute_prop_time_sdstwr();

        dw_set_tx_frame_length(DW1000_RANGING_FINAL_LEN);

        /* Copy the response frame to the DW1000 */
        /* header CRTL, seq num, PAN ID */
        dw_write_reg(DW_REG_TX_BUFFER, 0x5, (uint8_t*) &initiator_frame[0]);
        /* src */
        dw_write_subreg(DW_REG_TX_BUFFER, 0x5, 2,  
                              (uint8_t*) &initiator_frame[7]);
        /* dst */
        dw_write_subreg(DW_REG_TX_BUFFER, 0x7, 2,  
                              (uint8_t*) &initiator_frame[5]);
        /* t prop */
        dw_write_subreg(DW_REG_TX_BUFFER, 0x9, 4,  
                              (uint8_t*) &dw1000_driver_last_prop_time);
        /* t_reply_corrector */
        dw_write_subreg(DW_REG_TX_BUFFER, 0xD, 2,  
                              (uint8_t*) &t_reply_corrector);
       
        /* no wait for resp and no delayed */
        dw_init_tx(0, 0);

        sys_status_lo = 0x0;
        BUSYWAIT_UPDATE_UNTIL(dw_read_subreg(DW_REG_SYS_STATUS, 0x0, 1, 
              &sys_status_lo); watchdog_periodic();,
              ((sys_status_lo & DW_TXFRS_MASK) != 0),
              (theorical_transmission_approx(dw1000_conf.preamble_length, 
              dw1000_conf.data_rate, dw1000_conf.prf, 
              DW1000_RANGING_FINAL_LEN) << 1 )+ 
              DW1000_SPI_DELAY);
      }
    }

    dw_idle();

#if DEBUG_RANGING
    printf("time of an interrupt: %d\n", clock_ticks_to_microsecond(t1-t0));

    uint64_t sys_status = 0x0;
    dw_read_reg(DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS, (uint8_t *) &sys_status);
    print_sys_status(sys_status);
#endif /* DEBUG_RANGING */

    /* we reenable the RX state */
    dw1000_on();

    dw1000_driver_in_ranging = 0; /* False */

    PRINTF("dw interrupt sds twr\n");
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/* process used in case of single sided two way ranging (SS-TWR) */
PROCESS_THREAD(dw1000_driver_process_ss_twr, ev, data){
  PROCESS_BEGIN();

  PRINTF("dw1000_driver_process_sds_twr: started\r\n");
  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL); 

    /* we can not send the response if we are in RX mode */
    dw_idle();

#if !DW1000_IEEE802154_EXTENDED
    if(dw_is_ranging_frame()){
      /* In standard mode, we disable the RNG bit in the PHR for the response.*/
      dw_disable_ranging_frame();
    }
#endif

    /* don't wait a response */
    ranging_send_ack_sheduled(0, 1);

    dw1000_update_frame_quality();

    /* we reenable the RX state */
    dw1000_on();
    
    PRINTF("dw interrupt ss twr\n");
    dw1000_driver_in_ranging = 0; /* False */
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

/**
 * \brief Set the pan id and the address (16 bits or IEEE eUID 64 bits).
 */
void
dw1000_driver_set_pan_addr(unsigned pan,
                           unsigned addr,
                           const uint8_t *ieee_addr)
{
  uint8_t i = 0;

  uint16_t pan_id = pan & 0xFFFF;
  dw_set_pan_id(pan_id);

  uint16_t short_addr = addr & 0xFFFF;
  dw_set_short_addr(short_addr);

  if(ieee_addr != NULL) {
    uint64_t euid = 0x0;
    for(i = 0; i < 8; i++) {
      euid |= ieee_addr[i] << (8 * i);
    }
    dw_set_extendedUniqueID(euid);
  }
}
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
 * \param dw1000_prf_t The withed pulse repetition frequency.
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
  dw1000_conf.sfd_type = DW_SFD_NON_STANDARD;
  dw1000_conf.preamble_length = preamble_length;

  if(channel == DW_CHANNEL_1) {
    dw1000_conf.channel = DW_CHANNEL_1;
    dw1000_conf.preamble_code = DW_PREAMBLE_CODE_1;
    if(prf == DW_PRF_64_MHZ)
      dw1000_conf.preamble_code = DW_PREAMBLE_CODE_9;
  } else if(channel == DW_CHANNEL_2) {
    dw1000_conf.channel = DW_CHANNEL_2;
    dw1000_conf.preamble_code = DW_PREAMBLE_CODE_3;
    if(prf == DW_PRF_64_MHZ)
      dw1000_conf.preamble_code = DW_PREAMBLE_CODE_9;
  } else if(channel == DW_CHANNEL_3) {
    dw1000_conf.channel = DW_CHANNEL_3;
    dw1000_conf.preamble_code = DW_PREAMBLE_CODE_5;
    if(prf == DW_PRF_64_MHZ)
      dw1000_conf.preamble_code = DW_PREAMBLE_CODE_9;
  } else if(channel == DW_CHANNEL_4) {
    dw1000_conf.channel = DW_CHANNEL_4;
    dw1000_conf.preamble_code = DW_PREAMBLE_CODE_7;
    if(prf == DW_PRF_64_MHZ)
      dw1000_conf.preamble_code = DW_PREAMBLE_CODE_17;
  } else if(channel == DW_CHANNEL_5) {
    dw1000_conf.channel = DW_CHANNEL_5;
    dw1000_conf.preamble_code = DW_PREAMBLE_CODE_3;
    if(prf == DW_PRF_64_MHZ)
      dw1000_conf.preamble_code = DW_PREAMBLE_CODE_9;
  } else { /* channel 7 */
    dw1000_conf.channel = DW_CHANNEL_7;
    dw1000_conf.preamble_code = DW_PREAMBLE_CODE_7;
    if(prf == DW_PRF_64_MHZ)
      dw1000_conf.preamble_code = DW_PREAMBLE_CODE_17;
  }

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

  dw_conf(&dw1000_conf);

#if DW1000_CONF_AUTOACK
  dw_enable_automatic_acknowledge();
  dw_config_switching_tx_to_rx_ACK();
  dw_sfd_init();
#endif
}

/*===========================================================================*/
/* Ranging                                                                   */
/*===========================================================================*/

/**
 * \brief Set the next transmission to be the first message of a Single-sided 
 *        Two-way Ranging.
 */
void
dw1000_driver_sstwr_request(void)
{
  dw1000_driver_sstwr = 1;
}
/**
 * \brief Set the next transmission to be the first message of a 
 *              Symmetrical Double-sided Two-way Ranging.
 */
void
dw1000_driver_sdstwr_request(void)
{
  dw1000_driver_sdstwr = 1;
}
/**
 * \brief Check if the driver is in ranging mode.
 * 
 * \return if the driver is in ranging mode.
 */
uint8_t 
dw1000_driver_is_ranging_request(void)
{
  return (dw1000_driver_sstwr == 1) || (dw1000_driver_sdstwr == 1);
}
/**
 * \brief Gets the reply time used in the Single-sided Two-way Ranging.
 * \return The reply time.
 */
uint32_t
dw1000_driver_get_reply_time(void)
{
  return dw1000_driver_reply_time;
}
/**
 * \brief The reply time used in the Single-sided Two-way Ranging protocol.
 *        This value is in microsecond. It must be the smallest possible to 
 *        limit the clock error and it must be long enough to perform the reply.
 *        This value must be change in function of the configuration 
 *        (preamble and data_rate).
 *
 *        You can use a value of "0" to compute a dynamic value based on the 
 *          clock speed of a Zolertia Z1.
 * \param reply_time The desired reply time in micro second.
 */
void
dw1000_driver_set_reply_time(uint32_t reply_time)
{
  if(reply_time == 0){
#if DW1000_IEEE802154_EXTENDED
    reply_time = theorical_transmission_approx(dw1000_conf.preamble_length,
                      dw1000_conf.data_rate, dw1000_conf.prf, 12) + 
    DW1000_REPLY_TIME_COMPUTATION;
#else
    reply_time = theorical_transmission_approx(dw1000_conf.preamble_length,
                      dw1000_conf.data_rate, dw1000_conf.prf, 11)
    + DW1000_REPLY_TIME_COMPUTATION;
#endif /* DW1000_IEEE802154_EXTENDED */
  }

  dw1000_driver_reply_time = reply_time;
  dw1000_driver_schedule_reply_time = ((uint64_t) 
                                  dw1000_driver_reply_time * 125) << 9;
  printf("DW10000 Reply Time %lu (ms) %lu (Decawave time)\n", 
                                          dw1000_driver_reply_time,
                                          dw1000_driver_schedule_reply_time);
}
/**
 * \brief Based on the reply time and the RX timestamps, this function schedule 
 *        the ranging reply message.
 */
inline void 
dw1000_schedule_reply(void)
{
  uint64_t schedule_time = dw_get_rx_timestamp();
  /* require \ref note in the section 3.3 Delayed Transmission of the manual. */
  schedule_time &= DX_TIMESTAMP_CLEAR_LOW_9; /* clear the low order nine bits */
  /* The 10nd bit have a "value" of 125Mhz */
  schedule_time += dw1000_driver_schedule_reply_time; 

  dw_set_dx_timestamp(schedule_time);
}
/**
 * \brief Based on the reply time and the RX timestamps, this function schedule 
 *        the ranging reply message.
 */
void 
dw1000_schedule_reply2(void)
{
  int32_t rx_tofs = 0L;
  uint32_t rx_tofsU = 0UL;
  uint64_t rx_ttcki = 0ULL;

  uint64_t schedule_time = dw_get_rx_timestamp();
  /* require \ref note in the section 3.3 Delayed Transmission of the manual. */
  schedule_time &= DX_TIMESTAMP_CLEAR_LOW_9; /* clear the low order nine bits */
  
  schedule_time += dw1000_driver_schedule_reply_time;

  /* we want to take into a count the clock offset
    Clock offset = RX TOFS / RX TTCKI */

  /* brief dummy : The value in RXTTCKI will take just one of two values 
      depending on the PRF: 0x01F00000 @ 16 MHz PRF, 
      and 0x01FC0000 @ 64 MHz PRF. */
  if(dw1000_conf.prf == DW_PRF_16_MHZ){
    rx_ttcki = 0x01F00000ULL;
  } else{ /* prf == DW_PRF_64_MHZ */
    rx_ttcki = 0x01FC0000ULL;
  }

  /* RX TOFS is a signed 19-bit number, the 19nd bit is the sign */
  dw_read_subreg(DW_REG_RX_TTCKO, 0x0, 3, (uint8_t *) &rx_tofs);
  rx_tofs &= DW_RXTOFS_MASK;
  /* convert a 19 signed bit number to a 32 bits signed number */
  if((rx_tofs & (0x1UL << 18)) != 0){ /* the 19nd bit is 1 => negative number */
    /* a signed int is represented by a two complement representation */
    rx_tofs |= ~DW_RXTOFS_MASK; /* all bit between 31 and 19 are set to 1 */
    rx_tofsU = -rx_tofs; /* only store the absolute value */
    schedule_time -= 
          ((dw1000_driver_schedule_reply_time * rx_tofsU) / rx_ttcki) >> 1;
  }
  else{
    schedule_time += 
          ((dw1000_driver_schedule_reply_time * rx_tofs) / rx_ttcki) >> 1;
  }

  dw_set_dx_timestamp(schedule_time);
}

/**
 * \brief Compute the propagation time based on the reply time, and the RX and 
 *        TX timestamps
 *        /!\ Private function.
 *
 *        ToF = (t_round - t_reply (uncorrected))
 */
void 
dw1000_compute_prop_time_sdstwr(void){
  /* Compute the round time */
  dw1000_driver_last_prop_time = dw_get_rx_timestamp() - dw_get_tx_timestamp(); 

  /* We shift the reply time to match with the time of the DW1000 */
  dw1000_driver_last_prop_time -= dw1000_driver_schedule_reply_time;

  /* we don't divide the prop time to get better resolution */
}

/**
 * \brief Compute the propagation time based on the reply time, and the RX and 
 *        TX timestamps
 *        /!\ Private function.
 *
 *        ToF = (1/2)*(t_round - (1 + F) * t_reply)
 *            = (1/2)*(t_round - t_reply - (F * t_reply)
 *        F is the clock offset between the TX and the RX transceiver.
 *
 *        The Receiver Time Tracking Offset is provide by analyzing the 
 *        correction made by the phase-lock-loop (PLL) to decode the
 *        signal, it provide an estimate of the difference between the 
 *        transmitting and the receiver clock.
 */
void 
dw1000_compute_prop_time_sstwr(void){
  int32_t rx_tofs = 0L;
  uint64_t rx_tofsU = 0UL;
  uint8_t  rx_tofs_negative = 0; /* false */
  uint64_t rx_ttcki = 0ULL;
  /* Compute the round time  t_round*/
  dw1000_driver_last_prop_time = dw_get_rx_timestamp() - 
                              dw_get_tx_timestamp();      

  PRINTF("t reply  %d Deca Time\n", (unsigned int) dw1000_driver_reply_time);
  PRINTF("t reply 2 %d ms\n", (unsigned int) t_reply);

  dw1000_driver_last_prop_time -= dw1000_driver_schedule_reply_time;

  /* we want to take into a count the clock offset
    Clock offset = RX TOFS / RX TTCKI */

  /* RX TOFS is a signed 19-bit number, the 19nd bit is the sign */
  dw_read_subreg(DW_REG_RX_TTCKO, 0x0, 3, (uint8_t *) &rx_tofs);
  rx_tofs &= DW_RXTOFS_MASK;
  /* convert a 19 signed bit number to a 32 bits signed number */
  if((rx_tofs & (0x1UL << 18)) != 0){ /* the 19nd bit is 1 => negative number */
    /* a signed int is represented in Ones' complement */
    rx_tofs |= ~DW_RXTOFS_MASK; /* all bit between 31 and 19 are set to 1 */
    rx_tofs_negative = 1;
    rx_tofsU = -rx_tofs; /* only store the absolute value */
  }
  else
    rx_tofsU = rx_tofs;
  /* brief dummy : The value in RXTTCKI will take just one of two values 
      depending on the PRF: 0x01F00000 @ 16 MHz PRF, 
      and 0x01FC0000 @ 64 MHz PRF. */
  if(dw1000_conf.prf == DW_PRF_16_MHZ){
    rx_ttcki = 0x01F00000ULL;
  } else{ /* prf == DW_PRF_64_MHZ */
    rx_ttcki = 0x01FC0000ULL;
  }

  PRINTF("clock offset %ld\n", (long int) dw_get_clock_offset());
  /* We are not able to use the formula Clock offset = RX TOFS / RX TTCKI 
      because of the restricted embedded system.
      We change "- (Clock offset * t_reply)" to "- RX TOFS * t_reply / RX TTCKI"
      We don't want to use signed number => we made tow cases in function of the
      sign of RX TOFS */
  if(rx_tofs_negative == 1){
    dw1000_driver_last_prop_time -= 
                  ((dw1000_driver_schedule_reply_time * rx_tofsU) / rx_ttcki);
  }else {
    dw1000_driver_last_prop_time += 
                  ((dw1000_driver_schedule_reply_time * rx_tofsU) / rx_ttcki);
  }
  /* dw1000_driver_last_prop_time divided by 2 */
  dw1000_driver_last_prop_time = dw1000_driver_last_prop_time / 2;

#if DEBUG_VERBOSE
  PRINTF("RX TTCKI %"PRIu64"\n", (long long unsigned int) rx_ttcki);
  /* work but slow compare to of a no SPI assignation */
  rx_ttcki = dw_read_reg_32(DW_REG_RX_TTCKI, DW_LEN_RX_TTCKI);
  PRINTF("RX TTCKI %"PRIu64"\n", (long long unsigned int) rx_ttcki);
  PRINTF("RX TTCKI tow previous value must be the same\n");
  PRINTF("RX TOFS %lu\n", (long unsigned int) rx_tofs);
  PRINTF("RX TOFS negative (1 = true, 0 false) %d\n", rx_tofs_negative);
  PRINTF("t_reply %"PRIu64"\n", (long long unsigned int) t_reply);

  PRINTF("rx timestamp %"PRIu64"\n", 
    (long long unsigned int) dw_get_rx_timestamp());
  PRINTF("tx timestamp %"PRIu64"\n", 
    (long long unsigned int) dw_get_tx_timestamp());
  PRINTF("propagation corrected %"PRIu64"\n", 
    (long long unsigned int) dw1000_driver_last_prop_time);
#endif /* DEBUG_VERBOSE */

}

/**
 * \brief Return the propagation time, based on the last two way ranging.
 * \return The propagation time. The unit of the least significant bit is 
 *          approximately 15.65 picoseconds. The actual unit may be calculated 
 *          as 1/ (128*499.2×10^6 ) seconds.
 */
uint64_t
dw1000_driver_get_propagation_time(void){
  uint64_t propagation = dw1000_driver_last_prop_time;
  dw1000_driver_last_prop_time = 0;
  return propagation;
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

/**
 * \brief Send an ACK has ranging response.
 *        This send was scheduled base on the reply time.
 *
 *  \return If the message was correctly sent.
 */
uint8_t ranging_send_ack_sheduled(uint8_t wait_for_resp, uint8_t wait_send){
  uint32_t sys_status = 0x0; /* clear the value */
  uint8_t ack_num;

  /* clear the sys status */
  dw1000_driver_clear_pending_interrupt();

  dw1000_schedule_reply();

  /* read the lasted received message to prepare 
      response frame */
  dw_read_subreg(DW_REG_RX_BUFFER, 0x2, 1, &ack_num);
  /* prepare an ACK */

  uint8_t response[3];
  make_ack(ack_num, 3, &response[0]);

  dw_set_tx_frame_length(DW_ACK_LEN);

  /* Copy data to DW1000 */
  dw_write_reg(DW_REG_TX_BUFFER, 3, (uint8_t*) &response[0]);

  /* wait for resp and delayed */
  dw_init_tx(wait_for_resp, 1);

  if(wait_send){
    /* wait the end of the transmission */
    BUSYWAIT_UPDATE_UNTIL(dw_read_subreg(DW_REG_SYS_STATUS, 0x0, 4, 
              (uint8_t*) &sys_status); watchdog_periodic();,
              ((sys_status & (DW_TXFRS_MASK | DW_HPDWARN_MASK))  != 0),
              theorical_transmission_approx(dw1000_conf.preamble_length, 
              dw1000_conf.data_rate, dw1000_conf.prf, DW_ACK_LEN) + 
              DW1000_SPI_DELAY + dw1000_driver_reply_time);
    if((sys_status & DW_HPDWARN_MASK) != 0){
      dw_idle();
      printf("ranging_send_ack_sheduled HPDWARN\n"  );
      return 0;
    }
    if((sys_status & DW_TXFRS_MASK) == 0){

      printf("ranging_send_ack_sheduled TXFRS error\n"  );
    }
    /* ranging response send OK */
    return ((sys_status & DW_TXFRS_MASK) != 0);
  }
  return 1;
}

/**
 * \brief Convert the size of the payload based on the mode.
 *        If ranging, the payload size is changed.
 *        /!\ Private function.
 * */
uint16_t convert_payload_len(uint16_t payload_len){
  if(dw1000_driver_sstwr){
#if DW1000_IEEE802154_EXTENDED
    /* In extended mode, there are not a RNG bit in the PHR.
      We use a dedicated frame of 10 bytes and we use the last byte 
      to indicates a ranging request.  */
    payload_len = 10;
#else
    /* In standard mode we use the RNG bit in the PHR. */
    payload_len = 9;
#endif
  }
  if(dw1000_driver_sdstwr){
    payload_len = 10;
  }
  return payload_len;
}