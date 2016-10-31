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
 *          Decawave DW1000 utility file.
 *
 * \author
 *         Charlier Maximilien  <maximilien-charlier@outlook.com>
 *         Quoitin Bruno        <bruno.quoitin@umons.ac.be>
 */

#include <stdio.h>
#include <ctype.h>

#include "dw1000-util.h"
#include "dw1000.h"

/**
 * \brief Make a 802.15.4 data frame with short (16-bit) address.
 *
 * \param ACK         ACK Request.
 * \param seq_num     Sequence Number.
 * \param dest_pan_id Destination PAN Identifier.
 * \param dest_addr   Destination Address  (16-bit) address.
 * \param src_pan_id  Source PAN Identifier.
 * \param src_addr    Source Address (16-bit) address.
 * \param data_len    Len of Frame Payload.
 * \param data        Frame Payload.
 * \param[out]  frame       Pointer to a table containing frame header.
 * \return      The length of the frame.
 */
uint8_t
make_frame_short(uint8_t ACK, uint8_t seq_num,
                 uint16_t dest_pan_id, uint16_t dest_addr,
                 uint16_t src_pan_id, uint16_t src_addr,
                 uint8_t data_len, uint8_t *data,
                 uint8_t *frame, uint8_t frame_len)
{
  /* assert(data_len < 128 - 15); */
  uint8_t len = 11 + data_len;
  if(frame_len < len) {
    return len;
  }

  uint16_t frame_control = 0;
  frame_control |= 1;     /* Frame type field: data */
  if(ACK) {
    frame_control |= 1 << 5; /* ACK Request */
  }
  frame_control |= 0x02 << 10; /* Destination address field is a short (16-bit) address. */
  frame_control |= 0x01 << 12; /* Frame version field: indicate an IEEE 802.15.4 frame */
  frame_control |= 0x02 << 14; /* The source address field is a short (16-bit) address. */

  frame[0] = frame_control & 0xFF;
  frame[1] = frame_control >> 8;
  frame[2] = seq_num;
  frame[3] = (uint8_t)dest_pan_id & 0xFF;
  frame[4] = (uint8_t)(dest_pan_id >> 8);
  frame[5] = (uint8_t)dest_addr & 0xFF;
  frame[6] = (uint8_t)(dest_addr >> 8);
  frame[7] = (uint8_t)src_pan_id & 0xFF;
  frame[8] = (uint8_t)(src_pan_id >> 8);
  frame[9] = (uint8_t)src_addr & 0xFF;
  frame[10] = (uint8_t)(src_addr >> 8);
  int i;
  for(i = 0; i < data_len; i++) {
    frame[i + 11] = data[i];
  }
  return len;
}
/**
 * \brief             Make a 802.15.4 data frame with
 *                    extended (64-bit) address.
 *
 * \param ACK         ACK Request.
 * \param seq_num     Sequence Number.
 * \param dest_pan_id Destination PAN Identifier.
 * \param dest_addr   Destination address field is an extended
 *                           (64-bit) address.
 * \param src_pan_id  Source PAN Identifier.
 * \param src_addr    Source address field is an extended
 *                           (64-bit) address.
 * \param data_len    Len of Frame Payload.
 * \param data        Frame Payload.
 * \param frame       Pointer to a table containing the frame header.
 * \return            The length of the frame.
 */
uint8_t
make_frame_extended(uint8_t ACK, uint8_t seq_num,
                    uint16_t dest_pan_id, uint64_t dest_addr,
                    uint16_t src_pan_id, uint64_t src_addr,
                    uint8_t data_len, uint8_t *data,
                    uint8_t *frame, uint8_t frame_len)
{
  /* assert(data_len < 128 - 15); */
  uint8_t len = 23 + data_len;
  if(frame_len < len) {
    return len;
  }

  uint16_t frame_control = 0;
  frame_control |= 1;     /* Frame type field: data */
  if(ACK) {
    frame_control |= 1 << 5; /* ACK Request */
  }
  frame_control |= 0x03 << 10; /* Destination address field is a extended (64-bit) address. */
  frame_control |= 0x01 << 12; /* Frame version field: indicate an IEEE 802.15.4 frame */
  frame_control |= 0x03 << 14; /* The source address field is a extended (64-bit) address. */

  frame[0] = frame_control & 0xFF;
  frame[1] = frame_control >> 8;
  frame[2] = seq_num;

  frame[3] = (uint8_t)dest_pan_id & 0xFF;
  frame[4] = (uint8_t)(dest_pan_id >> 8);
  int i;
  for(i = 0; i < 8; i++) {
    frame[5 + i] = (uint8_t)((dest_addr >> (i * 8)) & 0xFF);
  }

  frame[13] = (uint8_t)src_pan_id & 0xFF;
  frame[14] = (uint8_t)(src_pan_id >> 8);
  for(i = 0; i < 8; i++) {
    frame[15 + i] = (uint8_t)((src_addr >> (i * 8)) & 0xFF);
  }
  for(i = 0; i < data_len; i++) {
    frame[i + 23] = data[i];
  }
  return len;
}
/**
 * \brief  Print a buffer with a prefix.
 * \param prefix   A prefix.
 * \param buf      A buffer.
 * \param buf_len  The buffer length.
 */
void
print_buf(const char *prefix, uint8_t *buf, uint8_t buf_len)
{
  while(buf_len > 0) {
    printf("%s", prefix);
    int i;
    for(i = 0; i < 16; i++) {
      if(i < buf_len) {
        printf("%.2X ", *(buf + i));
      } else {
        printf("   ");
      }
    }
    printf(" | ");
    for(i = 0; (i < 16) && (buf_len - i > 0); i++) {
      if(!iscntrl(*(buf + i))) {
        printf("%c", *(buf + i));
      } else {
        printf(".");
      }
    }
    buf_len -= i;
    buf += i;
    printf("\r\n");
  }
}
/**
 * \brief         Print the short ID (16 bit) only.
 *
 * \param frame   The start of the short ID byte.
 */
void
print_short_id(uint8_t *frame)
{
  printf("%02X%02X", *(frame + 1), *frame);
}
void
print_ext_id(uint8_t *frame)
{
  int j;
  for(j = 0; j < 8; j++) {
    if(j > 0) {
      printf("-%.2X", *(frame + j));
    } else {
      printf("%.2X", *(frame + j));
    }
  }
}
/* Declare all possible frame type */
static const char *FRAME_TYPES[8] = { "beacon", "data", "ACK", "MAC command",
                                      "reserved", "reserved", "reserved", "reserved" };

/**
 * \brief             Print a frame.
 *
 * \param frame_len   The frame length.
 * \param frame       An 802.15.4 MAC frame.
 */
void
print_frame(uint16_t frame_len, uint8_t *frame)
{
  if(frame_len >= 11) {
    print_buf("  ", frame, frame_len);

    printf("802.15.4 MAC Frame\r\n");
    uint16_t frame_control = frame[1] << 8 | frame[0];

    printf("  Length       : %d\r\n", frame_len);
    printf("  Frame ctrl.  : %.4X\r\n", frame_control);
    printf("  Type         : %s\r\n", FRAME_TYPES[frame_control & 7]);
    printf("  ACK req.     : %s\r\n", (frame[0] & (1 << 5)) ? "true" : "false");
    printf("  PAN ID compress.     : %s\r\n", (frame[0] & (1 << 6)) ? "true" : "false");
    printf("  Seq. num.    : %d\r\n", frame[2]);

    int i = 3;

    if((frame_control & (0x03 << 10)) == (0x03 << 10)) {
      printf("  Dst pan ID : ");
      print_short_id(frame + i);
      printf("\r\n      EID    : ");
      print_ext_id(frame + i + 2);
      printf("\r\n");
      i += 10;
    } else if((frame_control & (0x02 << 10)) == (0x02 << 10)) {
      printf("  Dst pan ID : ");
      print_short_id(frame + i);
      printf("\r\n      addr   : ");
      print_short_id(frame + i + 2);
      printf("\r\n");
      i += 4;
    }
    if(!(frame[0] & (1 << 6))) {
      printf("  Src pan ID : ");
      print_short_id(frame + i);
      i += 2;
    }
    if((frame_control & (0x03 << 14)) == (0x03 << 14)) {
      printf("\r\n      EID    : ");
      print_ext_id(frame + i + 2);
      printf("\r\n");
      i += 8;
    } else if((frame_control & (0x02 << 14)) == (0x02 << 14)) {
      printf("\r\n      addr   : ");
      print_short_id(frame + i + 2);
      printf("\r\n");
      i += 2;
    }

    if(frame[0] & (1 << 5)) {
      frame_len = frame_len - 2;
    }
    printf("  Payload :\r\n ");
    if(frame_len - i > 0) {
      print_buf("    ", frame + i, frame_len - i);
      for(; i < frame_len; i++) {
        printf("%c", frame[i]);
      }
    }
    printf("\r\n");
    if(frame[0] & (1 << 5)) {
      printf("  CRC     : %02X%02X\r\n", frame[frame_len + 1], frame[frame_len]);
    }
  } else if(frame_len == 5) {
    printf("ACK received.\r\n");
    printf("Sequence number: %d\r\n", frame[2]);
  } else {
    printf("!!! Message malformed received !!!\r\n");
    printf("Len message: %d\r\n", frame_len);
  }
}
/**
 * \brief Print value of register SYS_STATUS.
 *
 * \param sys_status    The value of SYS_STATUS register.
 */
void
print_sys_status(uint64_t sys_status)
{
  printf("-- SYS_STATUS --\r\n");
  printf("   %016X\r\n", (unsigned int)sys_status);
  if(sys_status & DW_IRQS_MASK) {
    printf("-- ");
    printf(" 1");
    printf("   ");
    printf("IRQS Interrupt Request Status.\r\n");
  }
  printf("-- TX:\r\n");
  if(sys_status & DW_TXFRB_MASK) {
    printf("   ");
    printf(" 4");
    printf("   ");
    printf("TXFRB Transmit Frame Begins.\r\n");
  }
  if(sys_status & DW_TXPRS_MASK) {
    printf("   ");
    printf(" 5");
    printf("   ");
    printf("TXPRS Transmit Preamble Sent.\r\n");
  }
  if(sys_status & DW_TXPHS_MASK) {
    printf("   ");
    printf(" 6");
    printf("   ");
    printf("TXPHS Transmit PHY Header Sent.\r\n");
  }
  if(sys_status & DW_TXFRS_MASK) {
    printf("   ");
    printf(" 7");
    printf("   ");
    printf("TXFRS Transmit Frame Sent.\r\n");
  }
  if(sys_status & DW_TXBERR_MASK) {
    printf("   ");
    printf("28");
    printf("   ");
    printf("TXBERR Transmit Buffer Error.\r\n");
  }

  printf("-- RX:\r\n");
  if(sys_status & DW_RXPRD_MASK) {
    printf("   ");
    printf(" 8");
    printf("   ");
    printf("RXPRD Receiver Preamble Detected Status.\r\n");
  }
  if(sys_status & DW_RXSFDD_MASK) {
    printf("   ");
    printf(" 9");
    printf("   ");
    printf("RXSFDD Receiver SFD Detected.\r\n");
  }
  if(sys_status & DW_LDEDONE_MASK) {
    printf("   ");
    printf("10");
    printf("   ");
    printf("LDEDONE LDE processing done.\r\n");
  }
  if(sys_status & DW_RXPHD_MASK) {
    printf("   ");
    printf("11");
    printf("   ");
    printf("RXPHD Receiver PHY Header Detect.\r\n");
  }
  if(sys_status & DW_RXPHE_MASK) {
    printf("   ");
    printf("12");
    printf("   ");
    printf("RXPHE Receiver PHY Header Error.\r\n");
  }
  if(sys_status & DW_RXDFR_MASK) {
    printf("   ");
    printf("13");
    printf("   ");
    printf("RXDFR Receiver Data Frame Ready.\r\n");
  }
  if(sys_status & DW_RXFCG_MASK) {
    printf("   ");
    printf("14");
    printf("   ");
    printf("RXFCG Receiver FCS Good.\r\n");
  }
  if(sys_status & DW_RXFCE_MASK) {
    printf("   ");
    printf("15");
    printf("   ");
    printf("RXFCE Receiver FCS Error.\r\n");
  }
  if(sys_status & DW_RXRFSL_MASK) {
    printf("   ");
    printf("16");
    printf("   ");
    printf("RXRFSL Receiver Reed Solomon Frame Sync Loss.\r\n");
  }
  if(sys_status & DW_RXRFTO_MASK) {
    printf("   ");
    printf("17");
    printf("   ");
    printf("RXRFTO Receive Frame Wait Timeout.\r\n");
  }
  if(sys_status & DW_LDEERR_MASK) {
    printf("   ");
    printf("18");
    printf("   ");
    printf("LDEERR Leading edge detection processing error.\r\n");
  }
  if(sys_status & DW_RXOVRR_MASK) {
    printf("   ");
    printf("20");
    printf("   ");
    printf("RXOVRR Receiver Overrun.\r\n");
  }
  if(sys_status & DW_RXPTO_MASK) {
    printf("   ");
    printf("21");
    printf("   ");
    printf("RXPTO Preamble detection timeout.\r\n");
  }
  if(sys_status & DW_RXSFDTO_MASK) {
    printf("   ");
    printf("26");
    printf("   ");
    printf("RXSFDTO Receive SFD timeout.\r\n");
  }
  if(sys_status & DW_AFFREJ_MASK) {
    printf("   ");
    printf("29");
    printf("   ");
    printf("AFFREJ Automatic Frame Filtering rejection.\r\n");
  }
  if((sys_status >> 31) & DW_RXRSCS_MASK) {
    printf("   ");
    printf("-0");
    printf("   ");
    printf("RXRSCS Receiver Reed-Solomon Correction Status.\r\n");
  }
  if((sys_status >> 31) & DW_RXPREJ_MASK) {
    printf("   ");
    printf("-1");
    printf("   ");
    printf("RXPREJ Receiver Preamble Rejection.\r\n");
  }

  if(sys_status & DW_HPDWARN_MASK) {
    printf("-- ");
    printf("27");
    printf("   ");
    printf("HPDWARN Half Period Delay Warning.\r\n");
  }
  if(sys_status & DW_GPIOIRQ_MASK) {
    printf("-- ");
    printf("22");
    printf("   ");
    printf("GPIOIRQ Preamble detection timeout.\r\n");
  }
  printf("Double buffering RX: \r\n");
  if(sys_status & DW_HSRBP_MASK) {
    printf("   ");
    printf("30");
    printf("   ");
    printf("HSRBP Host Side Receive Buffer Pointer.\r\n");
  }
  if(sys_status & DW_ICRBP_MASK) {
    printf("   ");
    printf("31");
    printf("   ");
    printf("ICRBP IC side Receive Buffer Pointer.\r\n");
  }
  printf("-- SYS_STATUS -- END.\r\n");
}
/**
 * \brief Compute the overhead of the Reed–Solomon error correction.
 */
#define ReedSolomonParityBit(x) (((x * 8 / 330) + 1) * 48)

/**
 * \brief Compute the theoretical time of a transmission.
 *
 * \param preamble_lenght    The preamble length (64 to 2048).
 * \param data_rate          The data rate (110, 850 or 8600) in kbps.
 * \param prf                The PRF, 16 or 64 MHz.
 *        Note: The PRF do not impact the approximation of the theoretical time.
 * \param data_lenght        The data length in bytes.
 *
 * \return An approximation of the theoretical time of a transmission in millisecond.
 */
long int
theorical_transmission_approx(uint16_t preamble_lenght, uint16_t data_rate, uint8_t prf, uint32_t data_lenght)
{
  uint16_t t_shr, t_prf, t_mac;
  uint32_t s_mac; /* to have a correct precision */

  /** Duration of the synchronization header and the PRF
   * SHR is the length of the SFD and the length of the preamble 
   * multiply by the duration of a symbol (1) */ 
  if(data_rate == DW_DATA_RATE_110_KBPS) {
    t_shr = 64 + preamble_lenght; 
    t_prf = 172; 
  }
  else {
    t_shr = 8 + preamble_lenght; 
    t_prf = 21;
  }

  /* duration of a data symbol  (*100 000) */
  if(data_rate == DW_DATA_RATE_110_KBPS) {
    s_mac = 820513;
  } else if(data_rate == DW_DATA_RATE_850_KBPS) {
    s_mac = 102564;
  } else { /* data_rate == 6800 kbps */
    s_mac = 12821;
  }

  t_mac = (s_mac * ((8 * data_lenght) 
          + ReedSolomonParityBit(data_lenght))) / 100000;

  return t_shr + t_prf + t_mac;
}

/**
 * \brief Convert a duration in micro second to a number of clock ticks
 * \param duration A delay in micro second.
 */
inline rtimer_clock_t microsecond_to_clock_tik(int duration) {
  return ((( (long int) RTIMER_SECOND) * duration) / 1000000) + 1;
}

/**
 * \brief Convert a number of clock ticks to a duration in micro second 
 * \param clock_tiks A number of clock ticks.
 */
inline int16_t clock_ticks_to_microsecond(rtimer_clock_t clock_ticks) {
  return ((long int) (1000000l * clock_ticks) / ((long int) RTIMER_SECOND)) + 1;
}