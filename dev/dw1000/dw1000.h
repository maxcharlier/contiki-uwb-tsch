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
 *         Hardware abstraction library for the decawave
 *            dw1000 specific driver.
 *          Based on the work of Hasan Derhamy & Kim Albertsson
 * \author
 *         Charlier Maximilien  <maximilien-charlier@outlook.com>
 *         Hasan Derhamy        <hasan.derhamy@ltu.se>
 *         Kim Albertsson       <kim.albertsson@ltu.se>
 */

#ifndef DW1000_H_
#define DW1000_H_

/*===========================================================================*/
/*================================ Includes =================================*/

#include <stdint.h> /* For fixed size integer uint32_t etc. */
#include "dw1000-const.h"

/*===========================================================================*/
/*================================ Defines ==================================*/

/* frame filtering bits */
#define DW_CFG_FF_ALL_EN  0x000001FEUL /* Frame filtering options all 
                                          frames allowed */
#define DW_FFAR 6
#define DW_FFA4 7
#define DW_FFA5 8

/**
 * \brief Defines the max length of the host mirror of the receive buffer in
 * variable dw1000.
 * \todo Put in dw_driver_config.h
 */
#define DW_RX_BUFFER_MAX_LEN 128

/**
 * \brief Defines the len of an ACK 
 */
#define DW_ACK_LEN 5

/**
 * \brief Used to align delayed timestamps.
 * \details Whenever a variable is called dx_timestamp the low order nine bits
 * are ignored by the device, thus to get the correct manual timestamping
 * behaviour this needs to be accounted for.
 */
#define DX_TIMESTAMP_CLEAR_LOW_9 ((uint64_t) 0xFFFFFFFFFFFFFE00ULL)

/**
 * \def DW_ERROR(...)
 * \brief Conditional print of encountered errors. Prints erros if the
 * compiler is invoked with \code -D ERROR_REPORT \endcode
 */
#ifdef ERROR_REPORT
#define DW_ERROR(...) do { printf(__VA_ARGS__); } while(0)
#else
#define DW_ERROR(...)
#endif

/**
 * \brief Used to choose from which source ADC should sample.
 */
typedef enum {
  DW_ADC_SRC_LATEST = 0,
  DW_ADC_SRC_WAKEUP
} dw_adc_src_t;

/**
 * \brief DW1000 state representation.
 */
typedef enum {
  DW_STATE_UNINITIALIZED = 0, /* \brief DW1000 uninitialized state. */
  DW_STATE_INITIALIZING,      /* \brief DW1000 currently running 
                               *  initialization.*/
  DW_STATE_IDLE,              /* \brief DW1000 idle state.*/
  DW_STATE_SLEEP,             /* \brief DW1000 has enetered sleep mode.*/
  DW_STATE_DEEP_SLEEP,        /* \brief DW1000 has enetered deep sleep mode.*/
  DW_STATE_RECEIVING,         /* \brief Reception turned on.*/
  DW_STATE_TRANSMITTING,      /* \brief Transmission started.*/
  DW_STATE_ERROR              /* \brief DW1000 has encouterend an error.*/
} dw1000_state_t;

/**
 * \brief DW1000 error codes.
 * \todo Add the other error codes. Find in register SYS_EVENT.
 */
typedef enum {
  /**
   * \brief No error encountered.
   */
  DW_ERROR_NO_ERROR = 0,
  /**
   * \brief Reception timed out.
   */
  DW_ERROR_TIMEOUT
} dw1000_error_t;

/*===========================================================================*/
/*============================= Configuration ===============================*/

/**
 * \brief Specifies if a wireless transaction should be sync or async.
 * Default for statically allocated variables is async.
 */
typedef enum {
  DW_TRANCEIVE_ASYNC = 0, /* \brief Asynchronous communication. Default. */
  DW_TRANCEIVE_SYNC       /* \brief Synchronous communication. */
} dw1000_tranceive_t;

/**
 * \brief Specifies which pulse repetition frequency the preamble and data
 * sections should use.
 */
typedef enum {
  DW_PRF_16_MHZ = 1, /* 16 MHz Pulse repetition */
  DW_PRF_64_MHZ = 2  /* 64 MHz Pulse repetition */
} dw1000_prf_t;

/**
 * \brief Selects the centre frequency and bandwidth for communication. The
 * dw1000 supports a subset of the 16 channels defined in the IEEE 802.15.4
 * UWB PHY.
 */
typedef enum {
  DW_CHANNEL_1 = 1, /* C-Freq: 3494.4 MHz, BWD: 499.2  MHz */
  DW_CHANNEL_2 = 2, /* C-Freq: 3993.6 MHz, BWD: 499.2  MHz */
  DW_CHANNEL_3 = 3, /* C-Freq: 4492.8 MHz, BWD: 499.2  MHz */
  DW_CHANNEL_4 = 4, /* C-Freq: 3993.6 MHz, BWD: 1331.2 MHz */
  DW_CHANNEL_5 = 5, /* C-Freq: 6489.6 MHz, BWD: 499.2  MHz */
  /* DW_CHANNEL_6 = 6,  Not supported */
  DW_CHANNEL_7 = 7  /* C-Freq: 6489.6 MHz, BWD: 1081.6 MHz */
    /* DW_CHANNEL_8  = 8,  Not supported */
    /* DW_CHANNEL_9  = 9,  Not supported */
    /* DW_CHANNEL_10 = 10, Not supported */
    /* DW_CHANNEL_11 = 11, Not supported */
    /* DW_CHANNEL_12 = 12, Not supported */
    /* DW_CHANNEL_13 = 13, Not supported */
    /* DW_CHANNEL_14 = 14, Not supported */
    /* DW_CHANNEL_15 = 15, Not supported */
    /* DW_CHANNEL_16 = 16  Not supported */
} dw1000_channel_t;

/**
 * \brief The preamble is part of the synchronisation frame. This option
 * configures the number of symbols in the preamble.
 *
 * \details A short header provides increased speed traded for communication
 * range.
 *
 */
typedef enum {
  DW_PREAMBLE_LENGTH_64 = 64,     /* Uses 64 symbols */
  DW_PREAMBLE_LENGTH_128 = 128,   /* Uses 128 symbols */
  DW_PREAMBLE_LENGTH_256 = 256,   /* Uses 256 symbols */
  DW_PREAMBLE_LENGTH_512 = 512,   /* Uses 512 symbols */
  DW_PREAMBLE_LENGTH_1024 = 1024, /* Uses 1024 symbols */
  DW_PREAMBLE_LENGTH_1536 = 1536, /* Uses 1536 symbols */
  DW_PREAMBLE_LENGTH_2048 = 2048, /* Uses 2048 symbols */
  DW_PREAMBLE_LENGTH_4096 = 4096  /* Uses 4096 symbols */
} dw1000_preamble_length_t;

/**
 * \brief The preamble code determines the specific pulse sequence of 
 *        the preamble.
 */
typedef enum {
  DW_PREAMBLE_CODE_1 = 1,
  DW_PREAMBLE_CODE_2 = 2,
  DW_PREAMBLE_CODE_3 = 3,
  DW_PREAMBLE_CODE_4 = 4,
  DW_PREAMBLE_CODE_5 = 5,
  DW_PREAMBLE_CODE_6 = 5,
  DW_PREAMBLE_CODE_7 = 7,
  DW_PREAMBLE_CODE_8 = 8,
  DW_PREAMBLE_CODE_9 = 9,
  DW_PREAMBLE_CODE_10 = 10,
  DW_PREAMBLE_CODE_11 = 11,
  DW_PREAMBLE_CODE_12 = 12,
  /* DW_PREAMBLE_CODE_13 = 13, */
  /* DW_PREAMBLE_CODE_14 = 14, */
  /* DW_PREAMBLE_CODE_15 = 15, */
  /* DW_PREAMBLE_CODE_16 = 16, */
  DW_PREAMBLE_CODE_17 = 17,
  DW_PREAMBLE_CODE_18 = 18,
  DW_PREAMBLE_CODE_19 = 19,
  DW_PREAMBLE_CODE_20 = 20
} dw1000_preamble_code_t;

/**
 * \brief   LDE Replica Coefficient configuration
 *      ref: Table 48: Sub-Register 0x2E:2804 – LDE_REPC
 *			configurations for (850 kbps & 6.8 Mbps)
 *
 *			When operating at 110 kbps the unsigned values in
 *			Table 48 have to be divided by 8,
 */
typedef enum {
  /* DW_LDE_REPC_0  = 0x5998UL, */
  DW_LDE_REPC_1 = 0x5998UL,
  DW_LDE_REPC_2 = 0x5998UL,
  DW_LDE_REPC_3 = 0x51EAUL,
  DW_LDE_REPC_4 = 0x428EUL,
  DW_LDE_REPC_5 = 0x451EUL,
  DW_LDE_REPC_6 = 0x2E14UL,
  DW_LDE_REPC_7 = 0x8000UL,
  DW_LDE_REPC_8 = 0x51EAUL,
  DW_LDE_REPC_9 = 0x28F4UL,
  DW_LDE_REPC_10 = 0x3332UL,
  DW_LDE_REPC_11 = 0x3AE0UL,
  DW_LDE_REPC_12 = 0x3D70UL,
  /* DW_LDE_REPC_13 = 0x3AE0UL, */
  /* DW_LDE_REPC_14 = 0x35C2UL, */
  /* DW_LDE_REPC_15 = 0x2B84UL, */
  /* DW_LDE_REPC_16 = 0x35C2UL, */
  DW_LDE_REPC_17 = 0x3332UL,
  DW_LDE_REPC_18 = 0x35C2UL,
  DW_LDE_REPC_19 = 0x35C2UL,
  DW_LDE_REPC_20 = 0x47AEUL,
  /* DW_LDE_REPC_21 = 0x47AEUL, */
  /* DW_LDE_REPC_22 = 0x3850UL, */
  /* DW_LDE_REPC_23 = 0x30A2UL, */
  /* DW_LDE_REPC_24 = 0x3850UL */
} dw1000_LDE_replica_coeff_t;

/**
 * \brief   Preamble Acquisition Chunk, the chunk size in which the
 *			preamble is processed. A larger PAC size is generally better
 *			for acquisition but should be small compared to the preamble
 *			length.
 *
 * \warning A PAC size of 64 cannot detect a preamble of length 64.
 */
typedef enum {
  DW_PAC_SIZE_8 = 8,
  DW_PAC_SIZE_16 = 16,
  DW_PAC_SIZE_32 = 32,
  DW_PAC_SIZE_64 = 64
} dw1000_pac_size_t;

/**
 * \brief   Select SFD (Start of frame delimiter). The dw1000 can use
 *			the SFD defined in IEEE 802.15.4 and a Decawave specific
 *			SFD for improved performance on 100kbps. There is also the
 *			option to specify your own SFD.
 *
 * \details See the decawave dw1000 user manual for more information on how
 *			use custom SFD.
 *
 * \warning As of 2014-10-17 the user specified SFD has not been implemented.
 */
typedef enum {
  DW_SFD_STANDARD = 0,  /* SFD as specified in IEEE 802.15.4. */
  DW_SFD_NON_STANDARD,  /* Decawave SFD. */
  DW_SFD_USER_SPECIFIED /* Indicates that a custom SFD has been defined. */
} dw1000_sfd_type_t;

/**
 * \brief   DW1000 data transfer rate selection.
 *
 *      The device can use three different data rate for communication.
 *			110 kbps,  850 kbps and 6.8 Mbps where increasing data rate
 *			decreases effective range.
 *
 *      Default value, 6.8 Mbps.
 *
 *  \bug  110 kbps is not working properly. Packet drops for long data frames.
 *      In the ranging protocol, the blink, ranging-init, poll and response
 *      messages are transmitted and received properly. However the final
 *			message always fails.
 */
typedef enum {
  DW_DATA_RATE_110_KBPS = 110,
  DW_DATA_RATE_850_KBPS = 850,
  DW_DATA_RATE_6800_KBPS = 6800
} dw1000_data_rate_t;

/**
 * \brief   DW1000 base configuration. Configuration of tranception properties
 *      that seldom changes. Handles configuration for:
 *      - PRF
 *      - Channel
 *      - Preamble length
 *      - Preabmle code
 *      - PAC size
 *      - SFD type
 *      - Data rate
 *
 *      Often changing configuration data for rx and tx are handled by
 *  \ref  dw1000_rx_conf_t and \ref dw1000_tx_conf_t respectively.
 */
typedef struct {
  /**
   * \brief Select Pulse Repetition Frequency to use for tx/rx.
   */
  dw1000_prf_t prf;
  /**
   * \brief Select channel to use for rx and tx. Should be configured the
   *  same as the remote device.
   */
  dw1000_channel_t channel;
  /**
   * \brief Select length of preamble.
   */
  dw1000_preamble_length_t preamble_length;
  /**
   * \brief Select which preamble code to use. Should be set according to
   *  table below. Should be configured identically on both transmitting and
   *  receiving end.
   *
   *        Channel no. | 16 MHz PRF |  64 MHz PRF
   *        :---------: | :--------: | :-------------:
   *             1      |    1, 2    |  9, 10, 11, 12
   *             2      |    3, 4    |  9, 10, 11, 12
   *             3      |    5, 6    |  9, 10, 11, 12
   *             4      |    7, 8    | 17, 18, 19, 20
   *             5      |    3, 4    |  9, 10, 11, 12
   *             7      |    7, 8    | 17, 18, 19, 20
   */
  dw1000_preamble_code_t preamble_code;
  /**
   * \brief PAC size determines in how large chunks the preamble is processed
   *  and should be set according to the table below.
   *
   *        Tx preamble len | Rx PAC size
   *        :-------------: | :---------:
   *          64            |    8
   *          128           |    8
   *          256           |    16
   *          512           |    16
   *          1024          |    32
   *          1536          |    64
   *          2048          |    64
   *          4096          |    64
   */
  dw1000_pac_size_t pac_size;
  /**
   * \brief Select the sfd scheme to use. Non-standard configuration can
   *  improve performance.
   *
   *  Can take three valid values: DW_SFD_STANDARD, DW_SFD_NON_STANDARD and
   *  DW_SFD_USER_SPECIFIED. When configured in standard operation, SFD
   *  sequences defined in IEEE 802.15.4-2011 UWB are used. When configured
   *  for non-standard operation, SFD sequences defined by Decawave are used.
   *  The SFD sequence can also be manually configured. For more information
   *  on this, see DW1000 User Manual page 105 and onward.
   */
  dw1000_sfd_type_t sfd_type;
  /**
   * \brief Select bit rate for data transfer. The dw1000 supports three
   *  speeds: 110 kbps, 850 kbps and 6.8 Mbps.
   */
  dw1000_data_rate_t data_rate;
} dw1000_base_conf_t;

/**
 * \brief Specifies how to receive data with the dw1000.
 */
typedef struct {
  /**
   * \brief Listening will start at time specified by dx_timestamp.
   */
  uint32_t is_delayed;
  /**
   * \brief See \ref is_delayed.
   * \details Value to be programmed into DW1000 dx_timestamp register.
   */
  uint64_t dx_timestamp;

  /**
   * \brief Timeout interval in approximate milliseconds for receiver.
   * \details 1.026 us per tick to be exact.
   */
  uint16_t timeout;
} dw1000_rx_conf_t;

/**
 * \brief Specifies how to transmit data with the dw1000.
 */
typedef struct {
  uint16_t data_len; /* \brief Length of data segment to send. */

  /**
   * \brief Listening will start at time specified by dx_timestamp.
   */
  uint32_t is_delayed;
  /**
   * \brief See \ref is_delayed.
   * \details Value to be programmed into DW1000 dx_timestamp register.
   */
  uint64_t dx_timestamp;
} dw1000_tx_conf_t;

/*===========================================================================*/
/*============================== Base driver ================================*/

/**
 * \brief DW1000 base driver.
 */
typedef struct {
  /**
   * \brief The current state of the dw1000.
   * \note Not fully implemented
   */
  dw1000_state_t state;

  /**
   * \brief Shows last encountered error.
   * \note Not fully implemented
   */
  dw1000_error_t error_code;

  /**
   * \brief Holds the configuration of the device. Modifications are
   * committed to the dw1000 by using dw_conf().
   */
  dw1000_base_conf_t conf;

  /**
   * \brief The current ACK mode.
   */
  uint8_t auto_ack;
} dw1000_base_driver;

extern dw1000_base_driver dw1000;

/*===========================================================================*/
/*============================ Public Functions =============================*/
void dw1000_init(void);

/* Configuration */
void dw_conf(dw1000_base_conf_t *dw_conf);
void dw_conf_rx(dw1000_rx_conf_t *rx_conf);
void dw_conf_print(void);
void dw_turn_frame_filtering_off(void);
void dw_turn_frame_filtering_on(void);
void dw_enable_gpio_led(void);
void dw_disable_gpio_led(void);

/* RX / TX */
void dw_init_rx(void);

/* Utility */

/* Device independent */
uint64_t dw_ms_to_device_time(float t);

/* Device dependent */
uint16_t dw_get_pan_id(void);
uint16_t dw_get_short_addr(void);
uint64_t dw_get_extendedUniqueID(void);
uint32_t dw_get_device_id(void);
uint64_t dw_get_device_time(void);
void dw_set_pan_id(uint16_t pan_id);
void dw_set_short_addr(uint16_t short_addr);
void dw_set_extendedUniqueID(uint64_t euid);
void print_u8_Array_inHex(char *string, uint8_t *array, uint32_t arrayLength);
void dw_soft_reset(void);

/* Diagnostics */
float dw_get_noise_level(void);
float dw_get_fp_ampl(void);

/* RX/TX */
void dw_enable_automatic_receiver_Re_Enable(void);
void dw_disable_automatic_receiver_Re_Enable(void);
void dw_get_rx_error(void);
int  dw_get_rx_len(void);
void dw_set_tx_frame_length(uint16_t frame_len);
void dw_enable_delayed_tx(uint64_t dx_timestamp);
void dw_enable_delayed_rx(uint64_t dx_timestamp);
void dw_disable_delayed_tx_rx(void);

void     dw_set_rx_timeout(uint16_t timeout);
uint16_t dw_get_rx_timeout(void);
void     dw_enable_rx_timeout(void);
void     dw_disable_rx_timeout(void);

/* Ranging  / timestamps */
uint64_t dw_get_rx_timestamp(void);
inline uint64_t dw_get_rx_raw_timestamp(void);
uint64_t dw_get_tx_timestamp(void);
inline void  dw_get_tx_raw_timestamp(uint64_t timestamp);
void     dw_set_antenna_delay(uint16_t delay);
uint16_t dw_get_antenna_delay(void);
uint64_t dw_get_dx_timestamp(void);
void     dw_set_dx_timestamp(uint64_t timestamp);
void     dw_enable_ranging_frame(void);
void     dw_disable_ranging_frame(void);
uint8_t is_ranging_frame(void);

void dw_clear_pending_interrupt(uint64_t mask);

/* Extended frame format. */
void dw_enable_extended_frame(void);
void dw_disable_extended_frame(void);
int  dw_get_rx_extended_len(void);

/* RX double buffering */
void dw_enable_double_buffering(void);
int  dw_good_rx_buffer_pointer(void);
int  dw_is_overrun(void);
void dw_trxsoft_reset(void);
void dw_change_rx_buffer(void);
void dw_trxoff_db_mode(void);
void dw_db_mode_clear_pending_interrupt(void);
void dw_db_init_rx(void);

/* ACK */
void dw_enable_automatic_acknowledge(void);
void dw_disable_automatic_acknowledge(void);
void dw_config_switching_tx_to_rx_ACK(dw1000_data_rate_t speed);

void dw_clear_receive_status(void);
void dw_clear_transmit_status(void);
int  dw_is_receive_status(uint64_t status);
int  dw_is_receive_done(uint64_t status);
int  dw_is_receive_CRC(uint64_t status);
int  dw_is_receive_failed(uint64_t status);
int  dw_is_receive_timeout(uint64_t status);

/*===========================================================================*/
/*=========================== Private Functions =============================*/
void dw_enable_interrupt(uint32_t mask);

void dw_idle(void);

void dw_init_tx(uint8_t wait_ack, uint8_t delayed);
void dw_suppress_auto_FCS_tx(void);

/*===========================================================================*/
/*============================ Test Functions ===============================*/
void dw1000_test_tx_del_on(void);
void dw1000_test_RW_longbits(void);
void dw1000_test(void);

/*===========================================================================*/
/*========================== Device communication ===========================*/

/* Registers */
inline void dw_read_reg(uint32_t reg_addr, uint16_t reg_len, uint8_t *p_data);
inline void dw_write_reg(uint32_t reg_addr, uint16_t reg_len, uint8_t *p_data);
uint32_t dw_read_reg_32(uint32_t reg_addr, uint16_t reg_len);
uint64_t dw_read_reg_64(uint32_t reg_addr, uint16_t reg_len);
/* Sub registers */
uint32_t dw_read_subreg_32(uint32_t reg_addr, uint16_t subreg_addr, 
                           uint16_t subreg_len);
uint64_t dw_read_subreg_64(uint32_t reg_addr, uint16_t subreg_addr, 
                           uint16_t subreg_len);

/* OTP */
uint32_t dw_read_otp_32(uint16_t otp_addr);

/**
 * \brief Indicates whether the current transfer is part of an SPI transaction
 * or not.
 *
 * \details An SPI transaction is a number of logically grouped SPI transfers.
 * For example, a register access on the device consists of writing to the
 * address of the register followed by reading a number of bytes. If a
 * transaction is not used in this case the device will forget the previously
 * committed command.
 */
typedef enum {
  DW_SPI_TRANSFER_DONE = 0, /* Indicates this is last transfer in transaction */
  DW_SPI_TRANSFER_CONT = 1  /* Indicates that there are more transfers to be 
                               done. */
} dw_spi_transfer_flag_t;

#endif /* DW1000_H_ */