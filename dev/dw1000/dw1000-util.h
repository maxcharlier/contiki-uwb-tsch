/*
 * Copyright (c) 2016, Charlier Maximilien, UMons University.
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
 *         Decawave DW1000 utilitary header file.
 * \author
 *         Charlier Maximilien <maximilien-charlier@outlook.com>
 *         Quoitin Bruno        <bruno.quoitin@umons.ac.be>
 */

#ifndef __DW1000_UTIL_H__
#define __DW1000_UTIL_H__

#include <inttypes.h>

uint8_t make_frame_short(uint8_t ACK, uint8_t seq_num,
                         uint16_t dest_pan_id, uint16_t dest_addr,
                         uint16_t src_pan_id, uint16_t src_addr,
                         uint8_t data_len, uint8_t *data,
                         uint8_t *frame, uint8_t frame_len);

uint8_t make_frame_extended(uint8_t ACK, uint8_t seq_num,
                            uint16_t dest_pan_id, uint64_t dest_addr,
                            uint16_t src_pan_id, uint64_t src_addr,
                            uint8_t data_len, uint8_t *data,
                            uint8_t *frame, uint8_t frame_len);

void print_buf(const char *prefix, uint8_t *buf, uint8_t buf_len);
void print_frame(uint16_t frame_len, uint8_t *frame);
void print_sys_status(uint64_t sys_status);

float theorical_speed(uint16_t preamble_lenght, uint16_t data_rate, uint8_t prf, uint16_t data_lenght);
#endif /* __DW1000_UTIL_H__ */
