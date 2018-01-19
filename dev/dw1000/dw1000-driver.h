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
 *         Interface for the driver of the Decawave dw1000 on Contiki
 *			header file.
 *          Based on the cc2420 driver.
 *
 * \author
 *         Charlier Maximilien  <maximilien-charlier@outlook.com>
 */

#include "contiki.h"
#include "dev/spi.h"
#include "dev/radio.h"
#include "dw1000.h"

int dw1000_driver_init(void);

int dw1000_driver_on(void);
int dw1000_driver_off(void);

void dw1000_driver_clear_pending_interrupt(void);
void dw1000_driver_enable_interrupt(void);
void dw1000_driver_disable_interrupt(void);
int dw1000_driver_interrupt(void);

void dw1000_driver_config(dw1000_channel_t channel, 
                          dw1000_data_rate_t data_rate, 
                          dw1000_preamble_length_t preamble_length, 
                          dw1000_prf_t prf);
/*===========================================================================*/
/* Ranging                                                                   */
/*===========================================================================*/

void dw1000_driver_sstwr_request(void);
void dw1000_driver_sdstwr_request(void);
uint8_t dw1000_driver_is_ranging_request(void);
uint32_t dw1000_driver_get_reply_time(void);
void dw1000_driver_set_reply_time(uint32_t reply_time);
int32_t dw1000_driver_get_propagation_time(void);
dw1000_frame_quality dw1000_driver_get_packet_quality(void);

/*---------------------------------------------------------------------------*/
/** The NETSTACK data structure for the cc2538 RF driver */
extern const struct radio_driver dw1000_driver;
/*---------------------------------------------------------------------------*/