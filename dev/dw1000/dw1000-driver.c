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
 *         Interface for the driver of the Decawave dw1000 on Contiki.
 *          Based on the cc2420 driver.
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

#ifndef DW1000_CONF_CHECKSUM
  #define DW1000_CONF_CHECKSUM    1
#endif /* DW1000_CONF_CHECKSUM */

#ifndef DW1000_CONF_AUTOACK
  #define DW1000_CONF_AUTOACK     1
#endif /* DW1000_CONF_AUTOACK */

#define FOOTER_LEN              2

#ifndef DW1000_CHANNEL
  #define DW1000_CHANNEL              5
#endif /* DW1000_CHANNEL */

#ifndef DW1000_DATA_RATE
  #define DW1000_DATA_RATE            DW_DATA_RATE_6800_KBPS
#endif /* DW1000_DATA_RATE */

#ifndef DW1000_DATA_RATE
  #define DW1000_DATA_RATE        DW_DATA_RATE_6800_KBPS
#endif
#if DW1000_DATA_RATE == DW_DATA_RATE_6800_KBPS 
   #if DW1000_IEEE802154_EXTENDED
    #define DW1000_TX_TIMEOUT        RTIMER_SECOND/1250 //max 800us  
  #else
    #define DW1000_TX_TIMEOUT        RTIMER_SECOND/1600 //max 625us  
  #endif
  #define DW1000_ACK_TIMEOUT       RTIMER_SECOND/2185 //max 460us 
#elif DW1000_DATA_RATE == DW_DATA_RATE_850_KBPS 
   #if DW1000_IEEE802154_EXTENDED
    #define DW1000_TX_TIMEOUT        RTIMER_SECOND/322 //max 3100us  
  #else
    #define DW1000_TX_TIMEOUT        RTIMER_SECOND/525 //max 1900us  
  #endif 
  #define DW1000_ACK_TIMEOUT       RTIMER_SECOND/1561 //640us
#else // 110kbps
   #if DW1000_IEEE802154_EXTENDED
    #define DW1000_TX_TIMEOUT        RTIMER_SECOND/43 //max 23000us  
  #else
    #define DW1000_TX_TIMEOUT        RTIMER_SECOND/76 //max 13000us  
  #endif 
  #define DW1000_ACK_TIMEOUT      0                  // bug with ACK
  #define DW1000_CONF_AUTOACK     0
  #define DW1000_DATA_RATE  DW_DATA_RATE_110_KBPS
#endif

#if DW1000_IEEE802154_EXTENDED
  #define DW1000_MAX_PACKET_LEN 265
#else
  #define DW1000_MAX_PACKET_LEN 127
#endif


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
  #define PRINTF(...) do {} while (0)
#endif

#define DEBUG_LED 1

#define DOUBLE_BUFFERING

// Used to fix an error with an possible interruption before 
// the driver initialisation
static int dw1000_driver_init_down = 0;

//define if transmition wait for an ACK.
static int dw1000_driver_wait_ACK = 0; 
static int dw1000_driver_wait_ACK_num = 0; 

static uint8_t volatile pending;

#define BUSYWAIT_UNTIL(cond, max_time)                                  \
  do {                                                                  \
    rtimer_clock_t t0;                                                  \
    t0 = RTIMER_NOW();                                                  \
    while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time)));   \
  } while(0)

#define BUSYWAIT_UNTIL2(update, cond, max_time)                         \
  do {                                                                  \
    rtimer_clock_t t0;                                                  \
    t0 = RTIMER_NOW();                                                  \
      do {                                                              \
        update;                                                         \
      } while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time))); \
  } while(0)

volatile uint8_t dw1000_driver_sfd_counter;
volatile uint16_t dw1000_driver_sfd_start_time;
volatile uint16_t dw1000_driver_sfd_end_time;

static volatile uint16_t last_packet_timestamp;

/* PLATFORM DEPENDENT */
/* these following functions are defined in platform/[platform]/dev/dw1000-arc.c */
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
void dw_read_subreg( uint32_t reg_addr, uint32_t subreg_addr, uint32_t subreg_len, uint8_t * p_data);
void dw_write_subreg(uint32_t reg_addr, uint32_t subreg_addr, uint32_t subreg_len, uint8_t * data );

// Composite data
void dw_write_reg_multiple_data( uint32_t    reg_addr,
                 uint32_t    reg_len,
                 uint8_t  ** pp_data,
                 uint32_t *  len_p_data,
                 uint32_t    len_pp_data);

/*---------------------------------------------------------------------------*/
PROCESS(dw1000_driver_process, "DW1000 driver");
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

#define GET_LOCK() locked++
static void RELEASE_LOCK(){
  if(locked == 1) {
    if(lock_on) {
      dw1000_on();
      lock_on = 0;
    }
    if(lock_off) {
      dw1000_off();
      lock_off = 0;
    }
  }
  locked--;
}
/*---------------------------------------------------------------------------*/

/**
 * Initialise SPI configuration for interacting with the DW1000
 *          and configure the DW1000 with the good channel.
 */
int dw1000_driver_init(void){
  PRINTF("dw1000_driver_init\r\n");

  dw1000_arch_init();

  dw1000_init();

  #if DW1000_IEEE802154_EXTENDED
    PRINTF("DW1000 set to use IEEE 802.15.4-2011 UWB non-standard mode, extended frame max 265 bytes.\r\n");
    dw_enable_extended_frame();
  #else
    PRINTF("DW1000 set to use IEEE 802.15.4-2011 UWB standard mode.\r\n");
    dw_disable_extended_frame();
  #endif

  dw1000_driver_config(DW1000_CHANNEL, DW1000_DATA_RATE);

  // dw1000_driver_set_pan_addr is recall after by contiki.
  // dw1000_driver_set_pan_addr(0xffff, 0x0000, NULL); 

  dw1000_driver_enable_interrupt();

  #if DW1000_CONF_AUTOACK
    dw_enable_automatic_acknowledge();
    dw_config_switching_tx_to_rx_ACK(DW1000_DATA_RATE);
  #endif

  dw_disable_rx_timeout();

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

  process_start(&dw1000_driver_process, NULL);
  dw1000_driver_init_down = 1;
  return 1;
}

/**
 * \brief Copy data to the TX buffer.
 */
static int dw1000_driver_prepare(const void *payload, 
                          unsigned short payload_len)
{
  PRINTF("dw1000_driver_prepare\r\n");

  uint32_t data_len = payload_len;
  #if DW1000_CONF_AUTOACK
    dw1000_driver_wait_ACK = (((uint8_t *) payload)[0] & (1 << 5)) ? 1:0;
    if(dw1000_driver_wait_ACK)
      dw1000_driver_wait_ACK_num= ((uint8_t *) payload)[2];
  #endif

  GET_LOCK();
  RIMESTATS_ADD(lltx);
  if(!DW1000_CONF_CHECKSUM)
    dw_suppress_auto_FCS_tx();
  else
    data_len += FOOTER_LEN; // add the FCS size

  //preparing DW1000 for sending
  
  #if DW1000_IEEE802154_EXTENDED
    dw_set_tx_extended_frame_length(data_len);
  #else
    dw_set_tx_frame_length(data_len);
  #endif
  // dw_disable_delayed_tx_rx();       < default value is same, not usefull

  if (payload_len > 0)
  {
    // Copy data to dw1000
    dw_write_reg(DW_REG_TX_BUFFER, payload_len, (uint8_t *) payload);
  }

  #if DEBUG_VERBOSE
    uint8_t tempRead[8] ;
    dw_read_reg(DW_REG_PANADR, DW_LEN_PANADR, tempRead);
    print_u8_Array_inHex ( "Reading PAN ID:", tempRead , DW_LEN_PANADR );

    uint8_t frame[DW1000_MAX_PACKET_LEN+1];
    dw_read_reg(DW_REG_TX_BUFFER, payload_len, (uint8_t *) frame);
    print_frame(payload_len, frame);
    PRINTF("payload_len %i\r\n", payload_len);
    PRINTF("Data len %i\r\n", (unsigned int) data_len);
  #endif
  
  RELEASE_LOCK();

  return 0;
}

/**
 * Transmit sends an already prepared packet.
 * It takes an unsigned short int that indicates the number of bytes to be transmitted and returns an int that
 *              indicates whether the transmission was successful or not.
 *
 * \return
 *          RADIO_TX_OK: Indicates that the transmission succeeded.
 *          RADIO_TX_ERR: Indicates that an error of some description has occurred.
 *          RADIO_TX_COLLISION: Indicates that a collision has occurred.
 *              This is only used by contikimac (for radios that block in TX and do hardware ACK detection) and nullrdc.
 *          RADIO_TX_NOACK: Indicates that no acknowledgement has been received.
 */
static int dw1000_driver_transmit(unsigned short payload_len){
  PRINTF("dw1000_driver_transmit\r\n");

  int tx_return = RADIO_TX_ERR;
  GET_LOCK();

  if(receive_on){
    dw_idle();
  }
  if(dw1000_driver_wait_ACK)
    dw1000_driver_disable_interrupt();

  ENERGEST_ON(ENERGEST_TYPE_TRANSMIT);

  if(dw1000_driver_wait_ACK)
    dw_init_tx_ack();
  else
    dw_init_tx();

  /* only reads low-byte of DW1000's SYS_STATUS (bit 7 is TXFRS) */
  uint8_t sys_status_lo;
  BUSYWAIT_UNTIL2(dw_read_subreg(DW_REG_SYS_STATUS, 0, 1, &sys_status_lo),
      ((sys_status_lo & DW_TXFRS_MASK) != 0),
      DW1000_TX_TIMEOUT);

  if ((sys_status_lo & DW_TXFRS_MASK) != 0)
    tx_return= RADIO_TX_OK;

  //wait ACK
  if(dw1000_driver_wait_ACK && tx_return == RADIO_TX_OK){
    tx_return = RADIO_TX_NOACK;
    BUSYWAIT_UNTIL2(dw_read_subreg(DW_REG_SYS_STATUS, 1, 1, &sys_status_lo/*_2*/); watchdog_periodic(),
        ( ((sys_status_lo/*_2*/ & (DW_RXDFR_MASK >> 8)) != 0) &&
          ((sys_status_lo/*_2*/ & ((DW_RXFCG_MASK >> 8) | (DW_RXFCE_MASK >> 8))) != 0) ),
        DW1000_ACK_TIMEOUT);
    if ((sys_status_lo/*_2*/ & (DW_RXFCG_MASK >> 8)) != 0){
      // (len ACK== 5) and (Seq Num, 3rd byte == ACK num)
      if(dw_get_rx_len() == 5 ){
          uint8_t ack_num;
          dw_read_subreg(DW_REG_RX_BUFFER, 2, 1, &ack_num);
          if(ack_num == dw1000_driver_wait_ACK_num)
            tx_return = RADIO_TX_OK;
      }
      #ifdef DOUBLE_BUFFERING
        //double buffering! swap the buffer
        if(dw_good_rx_buffer_pointer()){
          dw_db_mode_clear_pending_interrupt();
        }
        dw_change_rx_buffer();
      #else
        dw_clear_pending_interrupt(DW_MRXFCE_MASK
           | DW_MRXFCG_MASK
           | DW_MRXDFR_MASK
           | DW_MLDEDONE_MASK);
      #endif /* DOUBLE_BUFFERING */
    }
  }

  if(dw1000_driver_wait_ACK){
    dw_idle();  // bug fix of waiting an ACK which 
                  //  avoid the next transmittion 
    dw1000_driver_enable_interrupt();
  }

  // reenable the rx
  if(receive_on){
    dw_db_init_rx();
  }
  RELEASE_LOCK();
  
  ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);


  #if DEBUG_VERBOSE
    print_sys_status(dw_read_reg_64(DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS));
    if(tx_return == RADIO_TX_OK)
      PRINTF("TX RADIO_TX_OK \r\n");
  #endif
  #if DEBUG
    if(tx_return == RADIO_TX_NOACK)
      PRINTF("TX RADIO_TX_NOACK \r\n");
    if(tx_return == RADIO_TX_ERR){
      PRINTF("TX RADIO_TX_ERR \r\n");
    }
  #endif

  return tx_return;
}

/**
 * Send() prepare and transmit a packet.
 * This takes a pointer to the payload and an 
 *      unsigned short int that indicate the number of bytes.
 *
 * \return The same return codes as dw1000_driver_transmit().
 */
static int dw1000_driver_send(const void *data, unsigned short payload_len){ 
  PRINTF("dw1000_driver_send\r\n");
  dw1000_driver_prepare(data, payload_len);
  return dw1000_driver_transmit(payload_len);
}

/**
 * Reads a pending packet from the radio.
 *
 * \return The length of the read packet.
 */
static int dw1000_driver_read(void *buf, unsigned short bufsize){   

  PRINTF("dw1000_driver_read\r\n");

  GET_LOCK();

  #if DW1000_IEEE802154_EXTENDED
    int len = dw_get_rx_extended_len();
  #else
    int len = dw_get_rx_len();
  #endif
  

  if(len > DW1000_MAX_PACKET_LEN) {
    RIMESTATS_ADD(toolong);
    RELEASE_LOCK();
    return 0;
  }

  if(len <= FOOTER_LEN) {
    RIMESTATS_ADD(tooshort);
    RELEASE_LOCK();
    return 0;
  }

  if(len - FOOTER_LEN > bufsize) {
    RIMESTATS_ADD(toolong);
    RELEASE_LOCK();
    return 0;
  }

  if(DW1000_CONF_CHECKSUM)
    len -= FOOTER_LEN;

  // Store rx data in buf
  dw_read_reg( DW_REG_RX_BUFFER, len, (uint8_t *) buf);
  
  RIMESTATS_ADD(llrx);

  #if DEBUG_VERBOSE
    print_frame(len, buf);
  #endif

  pending--;
  RELEASE_LOCK();
  return len;
}

/** Channel Clear performs a Channel Clear Assesment to see 
 *    if there is other RF activity on
 *    the channel to avoid collisions.
 *
 * \return  "0" indicates that the channel is busy.
 *          "1" indicates that the channel is clear.
 */
static int dw1000_driver_cca(void){
  PRINTF("dw1000_driver_cca\r\n");
  // GET_LOCK();

  // dw_idle();
  // dw1000_driver_disable_interrupt();
  // dw1000_driver_clear_pending_interrupt();
  // dw_db_init_rx();

  // /*
  // * Check if a preamble has been detected.
  // */
  // // cc2420 RSSI time 128us ~1/7812s
  // // dw1000 127bytes frame at 6800kps > 287us > 1/3484
  // BUSYWAIT_UNTIL((dw_read_reg_64(DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS) & DW_RXPRD_MASK) == 0, RTIMER_SECOND / 3000);
  // dw1000_driver_enable_interrupt();
  // RELEASE_LOCK();
  // return (dw_read_reg_64(DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS) & DW_RXPRD_MASK) == 0;
  return 1;
}

/**
 * This function checks whether we are currently receiving a packet. This is used by several MAC layers to detect and indicate a collision before a TX. Return codes are:
 *
 * \return  "0" indicates that we are not receiving a packet.
 *          "1" indicates that we are receiving a packet.
 */
static int dw1000_driver_receiving_packet(void){
  PRINTF("dw1000_driver_receiving_packet\r\n");
  uint64_t status = dw_read_reg_64( DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS);
  return dw_is_receive_status(status);
}


/**
 * Checks to see if we have a pending packet. Some drivers check a flag set by an interrupt routine. Return codes are:
 *
 * \return  "0" indicates that there is no pending packet.
 *          "1" indicates that there is a pending packet.
 */
static int dw1000_driver_pending_packet(void){
  PRINTF("dw1000_driver_receiving_packet\r\n");
  return pending > 0;
}



/**
 * Turns "on" the radio. This function should put the radio into Receive mode and is called very frequently as
 *      the MAC layer turns on and off the radio rapidly. This function takes no arguments and returns an int
 *      that indicates whether the turn on succeeded.
 *
 * \return  "0" indicates that the radio did not turn on.
 *          "1" indicates that the radio did turn on.
 */
int dw1000_driver_on(void){
  PRINTF("dw1000_driver_on\r\n");
  if(receive_on) {
    return 1;
  }
  if(locked) {
    lock_on = 1;
    return 1;
  }

  GET_LOCK();
  dw1000_on();
  RELEASE_LOCK();
  return 1;
}

/**
 * Turn the radio on.
 */
static void dw1000_on(void){
  dw1000_driver_clear_pending_interrupt();
  dw_db_init_rx();
  /* The receiver has a delay of 16μs after issuing the enable receiver command,
   *      after which it will start receiving preamble symbols. */
  dw1000_us_delay(16);

  ENERGEST_ON(ENERGEST_TYPE_LISTEN);
  receive_on = 1;
}

/**
 * Turns "off" the radio. This should put the radio into Idle or a low-power state.
 *  This function takes no arguments and returns an int that indicates whether the turn off succeeded.
 *
 * \return  "0" indicates that the radio did not turn off.
 *          "1" indicates that the radio did turn off.
 */
int dw1000_driver_off(void){
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

  GET_LOCK();
  /* If we are currently receiving a packet (indicated by SFD == 1),
     we don't actually switch the radio off now, but signal that the
     driver should switch off the radio once the packet has been
     received and processed, by setting the 'lock_off' variable. */
  uint64_t status = dw_read_reg_64(DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS);
  if(((status & DW_RXSFDD_MASK) > 0) && ((status & (DW_RXDFR_MASK | DW_RXPHE_MASK)) == 0)) {
    lock_off = 1;
  } else {
    dw1000_off();
  }
  RELEASE_LOCK();
  return 1;
}

/**
 * Turn the radio off.
 */
static void dw1000_off(void){
  receive_on = 0;

  /* Wait for transmission to end before turning radio off. */
  // BUSYWAIT_UNTIL((dw1000.state == DW_STATE_TRANSMITTING), RTIMER_SECOND / 10);

  ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
  dw_trxoff_db_mode();
}
/** Get a radio parameter value. */
static radio_result_t dw1000_driver_get_value(
                    radio_param_t param, 
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
    // radio not set to filterring frame
    //  *value |= RADIO_RX_MODE_ADDRESS_FILTER;
    if(DW1000_CONF_AUTOACK) {
      *value |= RADIO_RX_MODE_AUTOACK;
    }
    // Radio not support poll
    //   *value |= RADIO_RX_MODE_POLL_MODE;
    return RADIO_RESULT_OK;
  case RADIO_PARAM_TX_MODE:
    // radio not support CCA
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

/** Set a radio parameter value. */
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
    // todo
    // dw1000_driver_config(value, dw1000_conf.data_rate);
    return RADIO_RESULT_NOT_SUPPORTED;
  case RADIO_PARAM_RX_MODE:
    // TOTO add support 
    // if(value & ~(RADIO_RX_MODE_ADDRESS_FILTER |
    //     RADIO_RX_MODE_AUTOACK | RADIO_RX_MODE_POLL_MODE)) {
    //   return RADIO_RESULT_INVALID_VALUE;
    // }
    // set_frame_filtering((value & RADIO_RX_MODE_ADDRESS_FILTER) != 0);
    // set_auto_ack((value & RADIO_RX_MODE_AUTOACK) != 0);
    // set_poll_mode((value & RADIO_RX_MODE_POLL_MODE) != 0);
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
 * Get a radio parameter object. The argument 'dest' must point to a
 * memory area of at least 'size' bytes, and this memory area will
 * contain the parameter object if the function succeeds.
 */
static radio_result_t dw1000_driver_get_object(
              radio_param_t param, 
              void *dest, size_t size)
{
  PRINTF("dw1000_driver_get_object\r\n");
  return RADIO_RESULT_NOT_SUPPORTED;
}

/**
 * Set a radio parameter object. The memory area referred to by the
 * argument 'src' will not be accessed after the function returns.
 */
static radio_result_t dw1000_driver_set_object(
            radio_param_t param, 
            const void *src, size_t size)
{
  PRINTF("dw1000_driver_set_object\r\n");
  return RADIO_RESULT_NOT_SUPPORTED;
}

/*---------------------------------------------------------------------------*/

/**
 * Enable interrupt for Frame with good CRC reception and Overrun
 */
void dw1000_driver_enable_interrupt(void){
  dw1000_driver_clear_pending_interrupt();
  dw_enable_interrupt( DW_MRXDFR_MASK | DW_RXOVRR_MASK);
}

/**
 * Disable interrupt
 */
void dw1000_driver_disable_interrupt(void){
  dw_enable_interrupt( 0Ull );
}

/**
 * Clear pending interruption.
 */
void dw1000_driver_clear_pending_interrupt(void){
  dw_clear_pending_interrupt( 0x00000007FFFFFFFFULL );
}


/*
 * Interrupt leaves frame intact in FIFO.
 */
int dw1000_driver_interrupt(void){
  PRINTF("dw1000_driver_interrupt()\r\n");
  /*
    dw1000_driver_init_down is use for fix bug of IRQ call before dw1000 reset 
  */
  if(dw1000_driver_init_down){
    uint64_t status = dw_read_reg_64( DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS);
    // print_sys_status(status);

    #ifdef DOUBLE_BUFFERING
      if(status & DW_RXOVRR_MASK){ //Overrun
        PRINTF("dw1000_driver_interrupt() > dw_overrun\r\n");
        dw1000_driver_disable_interrupt();
        dw_idle();        
        dw1000_driver_enable_interrupt();
        dw_trxsoft_reset();
        dw_change_rx_buffer();
        if(receive_on)
          dw_db_init_rx();
      } else
    #endif // DOUBLE_BUFFERING
    if(status & DW_RXDFR_MASK){ // receiver Data Frame Ready.
      process_poll(&dw1000_driver_process);
      pending++;
    }
    // error catch by RX reenable function.
  }
  return 1;
}

/*---------------------------------------------------------------------------*/

PROCESS_THREAD(dw1000_driver_process, ev, data){
  int len;
  PROCESS_BEGIN();

  PRINTF("dw1000_process: started\r\n");
  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

    uint64_t status = dw_read_reg_64( DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS);
    while(
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

          PRINTF("dw1000_process: RX Overrun\n");
          if(receive_on)
            dw_db_init_rx();
        }
        else 
#endif /* DOUBLE_BUFFERING */
      if(status & DW_RXFCG_MASK) { // no overrun and good CRC

        packetbuf_clear();
        //packetbuf_set_attr(PACKETBUF_ATTR_TIMESTAMP, last_packet_timestamp);

        len = dw1000_driver_read(packetbuf_dataptr(), PACKETBUF_SIZE);

#ifndef DOUBLE_BUFFERING
          dw_clear_pending_interrupt(DW_MRXFCE_MASK
                   | DW_MRXFCG_MASK
                   | DW_MRXDFR_MASK
                   | DW_MLDEDONE_MASK);
          dw_init_rx();

          // end of the interrupt
          /* See Figure 14: Flow chart for using double RX buffering
           * Of the manual */
          status = dw_read_reg_64( DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS);
          if(status & DW_RXOVRR_MASK){ //overun may be occurs
            dw_idle();
            dw_trxsoft_reset();
            dw_change_rx_buffer();

            PRINTF("dw1000_process: RX Overrun\n");
            if(receive_on)
              dw_db_init_rx();
          }
          else{
            if(dw_good_rx_buffer_pointer()){
              dw_db_mode_clear_pending_interrupt();
            }
            dw_change_rx_buffer();

          packetbuf_set_datalen(len);

          PRINTF("dw1000_process: len %i\r\n", len);

          NETSTACK_RDC.input();
          }
#else /* DOUBLE_BUFFERING */
          packetbuf_set_datalen(len);

          PRINTF("dw1000_process: len %i\r\n", len);

          NETSTACK_RDC.input();
#endif /* DOUBLE_BUFFERING */
      } else{ 
        /**
        Bad CRC, drop the packet > no read
        Change the buffer pointer.
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
        #endif /* DOUBLE_BUFFERING */
            PRINTF("bad CRC\n\r");
      }

      // Read status register for next iteration
      status = dw_read_reg_64( DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS);
    }
  }
  PROCESS_END();
}


/*-----------------------------------------------------------------------------*/

/**
 * Set the pan id and the address (16bits or IEEE eUID 64bits)
 */
void dw1000_driver_set_pan_addr(unsigned pan,
                    unsigned addr,
                    const uint8_t *ieee_addr){
  uint16_t i = 0;

  GET_LOCK();

  uint16_t pan_id = pan & 0xFFFF;
  dw_set_pan_id(pan_id);

  uint16_t short_addr = addr & 0xFFFF;
  dw_set_short_addr(short_addr);

  if(ieee_addr != NULL) {
      uint64_t euid = 0x0;
      for (i = 0; i < 8; i++) {
          euid |= ieee_addr[i] << (8 * i);
      }
      dw_set_extendedUniqueID(euid);
  }
  RELEASE_LOCK();
}

/**
* Set the channel:
*  Available channel: 1, 2, 3, 4, 5, 7.
*      Channel 4 and 7 have a bandwidth of 999MHz
*      Other channels have bandwidth of 500MHz
*  Pulse Repetition Frequency set to 16MHz
*  Preambule length set to the minimum for data transfert:
*      DW_DATA_RATE_110_KBPS  > 1024
*      DW_DATA_RATE_850_KBPS  > 256
*      DW_DATA_RATE_6800_KBPS > 128
*  
*/
void dw1000_driver_config(dw1000_channel_t channel, dw1000_data_rate_t data_rate){

  dw1000_base_conf_t dw1000_conf;

  dw1000_conf.prf             = DW_PRF_16_MHZ;
  dw1000_conf.sfd_type        = DW_SFD_STANDARD;
  if(channel == DW_CHANNEL_1){
    dw1000_conf.channel         = DW_CHANNEL_1;
    dw1000_conf.preamble_code   = DW_PREAMBLE_CODE_1;
  }
  else if(channel == DW_CHANNEL_2){
    dw1000_conf.channel         = DW_CHANNEL_2;
    dw1000_conf.preamble_code   = DW_PREAMBLE_CODE_3;
  }
  else if(channel == DW_CHANNEL_3){
    dw1000_conf.channel         = DW_CHANNEL_3;
    dw1000_conf.preamble_code   = DW_PREAMBLE_CODE_5;
  }
  else if(channel == DW_CHANNEL_4){
    dw1000_conf.channel         = DW_CHANNEL_4;
    dw1000_conf.preamble_code   = DW_PREAMBLE_CODE_7;
  }
  else if(channel == DW_CHANNEL_5){
    dw1000_conf.channel         = DW_CHANNEL_5;
    dw1000_conf.preamble_code   = DW_PREAMBLE_CODE_3;
  }
  else { // channel 7
    dw1000_conf.channel         = DW_CHANNEL_7;
    dw1000_conf.preamble_code   = DW_PREAMBLE_CODE_7;
  }

  if(data_rate == DW_DATA_RATE_110_KBPS){
    dw1000_conf.data_rate       = DW_DATA_RATE_110_KBPS;
    dw1000_conf.preamble_length = DW_PREAMBLE_LENGTH_1024;
    dw1000_conf.pac_size        = DW_PAC_SIZE_32;
  }
  else if(data_rate == DW_DATA_RATE_850_KBPS){
    dw1000_conf.data_rate       = DW_DATA_RATE_850_KBPS;
    dw1000_conf.preamble_length = DW_PREAMBLE_LENGTH_256;
    dw1000_conf.pac_size        = DW_PAC_SIZE_16;
  }
  else{ //6800 kbps
    dw1000_conf.data_rate       = DW_DATA_RATE_6800_KBPS;
    dw1000_conf.preamble_length = DW_PREAMBLE_LENGTH_128;
    dw1000_conf.pac_size        = DW_PAC_SIZE_8;
  }

  dw_conf(&dw1000_conf);

  // BQU -- SFD initialization: This can be done by writing
  // to the system control register Register file: 0x0D – System
  // Control Register with both the transmission startbit TXSTRT
  // and the transceiver off bit TRXOFF set at the same time. 
  uint32_t sys_ctrl;
  dw_read_reg(DW_REG_SYS_CTRL, DW_LEN_SYS_CTRL, (uint8_t *)&sys_ctrl);
  sys_ctrl |= DW_TXSTRT_MASK | DW_TRXOFF_MASK;
  dw_write_reg(DW_REG_SYS_CTRL, DW_LEN_SYS_CTRL, (uint8_t *)&sys_ctrl);
  sys_ctrl &= ~( DW_TXSTRT_MASK );
  sys_ctrl |= ( DW_RXENAB_MASK );
  dw_write_reg(DW_REG_SYS_CTRL, DW_LEN_SYS_CTRL, (uint8_t *)&sys_ctrl);
}