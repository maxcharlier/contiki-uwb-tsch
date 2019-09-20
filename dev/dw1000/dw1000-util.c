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
 *         Charlier Maximilien  <maximilien-charlier@umons.ac.be>
 *         Quoitin Bruno        <bruno.quoitin@umons.ac.be>
 */

#include <stdio.h>
#include <ctype.h>

#include "dw1000-util.h"
#include "dw1000.h"

/* Declare all possible frame type */
static const char *FRAME_TYPES[8] = { "beacon", "data", "ACK", "MAC command",
                                      "reserved", "reserved", "reserved", 
                                      "reserved" };


/**
 * \brief     Make a 802.15.4 data frame.
 *            If the PANID source and destination are the same, we use the PANID
 *            compression to reduction the length of the frame.
 *
 * \param ack           ACK Request.
 * \param seq_num       Sequence Number.
 * \param dest_pan_id   Destination PAN Identifier.
 * \param dest_add_type Destination type, 64 or 16 bits address.
 *                        IEEE_SHORT_ADDR define an 16-bit address.
 *                        IEEE_EXTENDED_ADDR define an 64-bit address.
 * \param dest_addr     Destination address field.
 * \param src_pan_id    Source PAN Identifier.
 * \param src_add_type  Source type, 64 or 16 bits address.
 *                        IEEE_SHORT_ADDR define an 16-bit address.
 *                        IEEE_EXTENDED_ADDR define an 64-bit address.
 * \param src_addr      Source address field.
 * \param data_len      Len of the frame payload.
 * \param data          The frame payload.
 * \param frame_len     Len of the all frame.
 * \param frame         Pointer to a table containing the frame header.
 * \return              The length of the frame.
 */
uint8_t
make_frame(uint8_t ack, uint8_t seq_num,
           uint16_t dest_pan_id, uint8_t dest_add_type, uint64_t dest_addr,
           uint16_t src_pan_id, uint8_t src_add_type, uint64_t src_addr,
           uint8_t data_len, uint8_t *data,
           uint8_t frame_len, uint8_t *frame)
{
  uint8_t header = 11, dest_len = 2, src_len = 2;
  /* if we use an extended address we use 8 bytes in place of 2 bytes*/
  if(dest_add_type == IEEE_EXTENDED_ADDR){
    header += 6;
    dest_len = 8;
  }
  if(src_add_type == IEEE_EXTENDED_ADDR){
    header += 6;
    src_len = 8;
  }
  if(src_pan_id == dest_pan_id){
    /* PAN ID compression */
    header -= 2;
  }
  uint8_t len = header + data_len;
  if(frame_len < len) {
    return len;
  }

  /* for debugging
  assert(data_len < 128 - 15);*/

  uint16_t frame_control = 0;
  frame_control |= 1;     /* Frame type field: data */
  if(ack) {
    frame_control |= 1 << IEEE_ACK_FIELD_OFSSET; /* ACK Request */
  }
  /* Destination address field. */
  frame_control |= dest_add_type << IEEE_DEST_ADRESS_FIELD_OFSSET; 
  /* Frame version field: indicate an IEEE 802.15.4 frame */
  frame_control |= 0x01 << IEEE_VERSION_FIELD_OFSSET; 
  /* The source address field. */
  frame_control |= src_add_type << IEEE_SOURCE_ADRESS_FIELD_OFFSET; 

  frame[0] = frame_control & 0xFF;
  frame[1] = frame_control >> 8;
  frame[2] = seq_num;

  /* dest pan id */
  frame[3] = (uint8_t) (dest_pan_id & 0xFF);
  frame[4] = (uint8_t) (dest_pan_id >> 8);
  uint8_t i;
  /* dest addr */
  for(i = 0; i < dest_len; i++) {
    frame[5 + i] = (uint8_t) ((dest_addr >> (i * 8)) & 0xFF);
  }

  uint8_t offset = 5 + dest_len;
  /* src pan id */
  if(src_pan_id == dest_pan_id){
    /* PAN ID compression */ 
    frame[0] |= (1 << 6);
  } else{
    frame[offset] = (uint8_t) (src_pan_id & 0xFF);
    frame[offset + 1] = (uint8_t) (src_pan_id >> 8);
    offset += 2;
  }
  /* src addr */
  for(i = 0; i < src_len; i++) {
    frame[offset + i] = (uint8_t) ((src_addr >> (i * 8)) & 0xFF);
  }
  offset += src_len;

  /* copy the data payload */
  for(i = 0; i < data_len; i++) {
    frame[offset + i] = data[i];
  }

  return len;
}
/**
 * \brief   Make a 802.15.4 data frame in response of an 802.15.4 
 *          data frame.
 *
 * \param ack           ACK Request.
 * \param seq_num       Sequence Number.
 * \param src_frame_len Len of the source data frame.
 * \param src_frame     The source data frame.
 * \param resp_data_len Len of the frame payload.
 * \param resp_data     The frame payload.
 * \param frame_len     Len of the all frame.
 * \param frame         Pointer to a table containing the frame header.
 * \return              The length of the frame.
 */
uint8_t
make_response(uint8_t ack, uint8_t seq_num,
           uint8_t src_frame_len, uint8_t *src_frame,
           uint8_t resp_data_len, uint8_t *resp_data,
           uint8_t frame_len, uint8_t *frame)
{
  uint16_t frame_control = src_frame[1] << 8 | src_frame[0];

  uint8_t dest_add_type = (frame_control & IEEE_DEST_ADRESS_FIELD_MASK) 
                          >> IEEE_DEST_ADRESS_FIELD_OFSSET;
  uint16_t dest_pan_id = src_frame[3] | src_frame[4] << 8;
  uint8_t src_add_type = (frame_control & IEEE_SOURCE_ADRESS_FIELD_MASK) 
                          >> IEEE_SOURCE_ADRESS_FIELD_OFFSET;
  uint64_t dest_addr = 0x0;
  uint64_t src_addr = 0x0;
  uint8_t dest_len = 2, src_len = 2;
  /* if we use an extended address we use 8 bytes in place of 2 bytes*/
  if(dest_add_type == IEEE_EXTENDED_ADDR){
    dest_len = 8;
  }
  if(src_add_type == IEEE_EXTENDED_ADDR){
    src_len = 8;
  }

  uint8_t i;
  /* destination address */
  for(i = 0; i < dest_len; i++) {
    dest_addr |= src_frame[5 + i] << i * 8;
  }

  uint8_t offset = 5 + dest_len;
  /* source pan id */
  uint16_t src_pan_id = dest_pan_id;
  if(!(frame_control & (1 << 6))){
    /* if the PANID compression was not triggered */
    src_pan_id = src_frame[offset] | src_frame[offset + 1] << 8;
    offset += 2;
  }
  
  /* source address */
  for(i = 0; i < src_len; i++) {
    src_addr |= src_frame[offset + i] << i * 8;
  }
  return make_frame(ack, seq_num, 
                    src_pan_id, src_add_type, src_addr,
                    dest_pan_id, dest_add_type, dest_addr,
                    resp_data_len, resp_data,
                    frame_len, frame);

}/**
 * \brief   Make a 802.15.4 ACK frame in response of an 802.15.4 
 *          data frame.
 *
 * \param seq_num       Sequence Number.
 * \param frame_len     Max length of the buffer "frame".
 * \param frame         Pointer to a buffer.
 * \return              The length of the frame (without FCS) => 3.
 */
uint8_t
make_ack(uint8_t seq_num, uint8_t frame_len, uint8_t *frame)
{
  uint16_t frame_control = 0x02;
  frame[0] = frame_control & 0xFF;
  frame[1] = frame_control >> 8;
  frame[2] = seq_num;
  return 3;
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
/**
 * \brief             Print a frame.
 *
 * \param frame_len   The frame length.
 * \param frame       An 802.15.4 MAC frame.
 */
void
print_frame(uint16_t frame_len, uint8_t *frame)
{
  if(frame_len >= 9) {
    print_buf("  ", frame, frame_len);

    printf("802.15.4 MAC Frame\r\n");
    uint16_t frame_control = frame[1] << 8 | frame[0];

    printf("  Length       : %d\r\n", frame_len);
    printf("  Frame ctrl.  : %.4X\r\n", frame_control);
    printf("  Type         : %s\r\n", FRAME_TYPES[frame_control & 7]);
    printf("  ACK req.     : %s\r\n", (frame[0] & (1 << 5)) ? "true" : "false");
    printf("  PAN ID compress.     : %s\r\n", 
                                      (frame[0] & (1 << 6)) ? "true" : "false");
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
      printf("\r\n  Dst addr   : ");
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
      print_ext_id(frame + i);
      printf("\r\n");
      i += 8;
    } else if((frame_control & (0x02 << 14)) == (0x02 << 14)) {
      printf("\r\n  Src addr   : ");
      print_short_id(frame + i);
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
      printf("  CRC     : %02X%02X\r\n", 
                                        frame[frame_len + 1], frame[frame_len]);
    }
  } else if(frame_len == 3 || frame_len == 5) { //With or without ACK
    printf("ACK, len = %d.\r\n", frame_len);
    printf("Sequence number: %d\r\n", frame[2]);
  } else {
    printf("!!! Message malformed received !!!\r\n");
    printf("Len message: %d\r\n", frame_len);
  }
}/**
 * \brief Print value of register SYS_STATE.
 *
 * \param sys_statue    The value of SYS_STATE register.
 */
void
print_sys_state(uint64_t sys_state)
{
  printf("--  SYS_STATE  --\r\n");
  printf("sys_state : 0x%016" PRIx64 "\r\n", 
                              (unsigned long long) sys_state);
  uint32_t pmsc_state = sys_state & DW_PMSC_STATE_MASK;
  printf("Current PMSC State Machine value 0x%02X\n", 
                        (unsigned int) (pmsc_state >> DW_PMSC_STATE));
  if(pmsc_state == DW_PMSC_STATE_INIT){
    printf("   ");
    printf("0x00");
    printf("   ");
    printf("DW1000 is in INIT.\r\n");
  }
  else if(pmsc_state == DW_PMSC_STATE_IDLE){
    printf("   ");
    printf("0x01");
    printf("   ");
    printf("DW1000 is in IDLE.\r\n");
  }
  else if(pmsc_state == DW_PMSC_STATE_TX_WAIT){
    printf("   ");
    printf("0x02");
    printf("   ");
    printf("DW1000 is waiting to tart transmitting.\r\n");
  }
  else if(pmsc_state == DW_PMSC_STATE_RX_WAIT){
    printf("   ");
    printf("0x03");
    printf("   ");
    printf("DW1000 is waiting to enter receive mode.\r\n");
  }
  else if(pmsc_state == DW_PMSC_STATE_TX){
    printf("   ");
    printf("0x04");
    printf("   ");
    printf("DW1000 is transmitting.\r\n");
  }
  else if(pmsc_state == DW_PMSC_STATE_RX){
    printf("   ");
    printf("0x05");
    printf("   ");
    printf("DW1000 is in receive mode.\r\n");
  }
  else{
    printf("---");
    printf("0x%02X", (unsigned int) (pmsc_state >> DW_PMSC_STATE));
    printf("   ");
    printf("DW1000 is in unknown mode.\r\n");
  }
  uint32_t tx_state = sys_state & DW_TX_STATE_MASK;
  printf("Current TX State Machine value 0x%02X\r\n", 
                                      (unsigned int) (tx_state >> DW_TX_STATE));
  if(tx_state == DW_TX_STATE_IDLE){
    printf("   ");
    printf("0x00");
    printf("   ");
    printf("Transmitter is in idle.\r\n");
  }
  else if(tx_state == DW_TX_STATE_PREAMBLE){
    printf("   ");
    printf("0x01");
    printf("   ");
    printf("Transmitting preamble.\r\n");
  }
  else if(tx_state == DW_TX_STATE_SFD){
    printf("   ");
    printf("0x02");
    printf("   ");
    printf("Transmitting SFD.\r\n");
  }
  else if(tx_state == DW_TX_STATE_PHR){
    printf("   ");
    printf("0x03");
    printf("   ");
    printf("Transmitting PHR.\r\n");
  }
  else if(tx_state == DW_TX_STATE_SDE){
    printf("   ");
    printf("0x04");
    printf("   ");
    printf("Transmitting PHR parity SECDED bits.\r\n");
  }
  else if(tx_state == DW_TX_STATE_DATA){
    printf("   ");
    printf("0x05");
    printf("   ");
    printf("Transmitting data.\r\n");
  }
  else if(tx_state == DW_TX_STATE_RSP_DATA){
    printf("   ");
    printf("0x06");
    printf("   ");
    printf("Transmitting Reed Solomon parity block.\r\n");
  }
  else{
    printf("----");
    printf("0x%02X", (unsigned int) (tx_state >> DW_TX_STATE));
    printf("   ");
    printf("Transmitter is in unknown mode.\r\n");
  }
  uint32_t rx_state = sys_state & DW_RX_STATE_MASK;
  printf("Current RX State Machine value 0x%02X\r\n", 
                                      (unsigned int) (rx_state >> DW_RX_STATE));
  if(rx_state == DW_RX_STATE_IDLE){
    printf("   ");
    printf("0x00");
    printf("   ");
    printf("Receiver is in idle.\r\n");
  }
  else if(rx_state == DW_RX_STATE_START_ANALOG){
    printf("   ");
    printf("0x01");
    printf("   ");
    printf("Start analog receiver blocks.\r\n");
  }
  else if(rx_state == DW_RX_STATE_RX_READY){
    printf("   ");
    printf("0x04");
    printf("   ");
    printf("Receiver ready.\r\n");
  }
  else if(rx_state == DW_RX_STATE_PREAMBLE_FIND){
    printf("   ");
    printf("0x05");
    printf("   ");
    printf("Receiver is waiting to detect preamble.\r\n");
  }
  else if(rx_state == DW_RX_STATE_PREAMBLE_TO){
    printf("   ");
    printf("0x06");
    printf("   ");
    printf("Preamble timeout.\r\n");
  }
  else if(rx_state == DW_RX_STATE_SFD_FOUND){
    printf("   ");
    printf("0x07");
    printf("   ");
    printf("SFD found.\r\n");
  }
  else if(rx_state == DW_RX_STATE_CONFIGURE_PRH_RX){
    printf("   ");
    printf("0x08");
    printf("   ");
    printf("Configure for PHR reception.\r\n");
  }
  else if(rx_state == DW_RX_STATE_PHR_RX_START){
    printf("   ");
    printf("0x09");
    printf("   ");
    printf("PHR reception started.\r\n");
  }
  else if(rx_state == DW_RX_STATE_DATA_RATE_READY){
    printf("   ");
    printf("0x0A");
    printf("   ");
    printf("Ready for data reception.\r\n");
  }
  else if(rx_state == DW_RX_STATE_DATA_RX_SEQ){
    printf("   ");
    printf("0x0C");
    printf("   ");
    printf("Data reception.\r\n");
  }
  else if(rx_state == DW_RX_STATE_CONFIG_DATA){
    printf("   ");
    printf("0x0D");
    printf("   ");
    printf("Configure for data.\r\n");
  }
  else if(rx_state == DW_RX_STATE_PHR_NOT_OK){
    printf("   ");
    printf("0x0E");
    printf("   ");
    printf("PHR error.\r\n");
  }
  else if(rx_state == DW_RX_STATE_LAST_SYMBOL){
    printf("   ");
    printf("0x0F");
    printf("   ");
    printf("Received last symbol.\r\n");
  }
  else if(rx_state == DW_RX_STATE_WAIT_RSQ_DONE){
    printf("   ");
    printf("0x10");
    printf("   ");
    printf("Wait for Reed Solomon decoder to finish.\r\n");
  }
  else if(rx_state == DW_RX_STATE_RSD_OK){
    printf("   ");
    printf("0x11");
    printf("   ");
    printf("Reed Solomon correct.\r\n");
  }
  else if(rx_state == DW_RX_STATE_RSD_NOT_OK){
    printf("   ");
    printf("0x12");
    printf("   ");
    printf("Reed Solomon error.\r\n");
  }
  else if(rx_state == DW_RX_STATE_RECONFIG_110){
    printf("   ");
    printf("0x13");
    printf("   ");
    printf("Reconfigure for 110 kbps data.\r\n");
  }
  else if(rx_state == DW_RX_STATE_WAIT_110_PHR){
    printf("   ");
    printf("0x14");
    printf("   ");
    printf("Wait for 110 kbps PHR.\r\n");
  }
  else{
    printf("----");
    printf("0x%02X", (unsigned int) (rx_state >> DW_RX_STATE));
    printf("   ");
    printf("Receiver is in unknown mode.\r\n");
  }
  printf("--  SYS_STATE  -- END.\r\n");
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
  printf("   %02X - %04X%04X\r\n", (unsigned int) (sys_status >> 32), 
                                 (unsigned int) (sys_status >> 16), 
                                 ((unsigned int) sys_status));
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
  if(sys_status & DW_LDEERR_MASK << 1) {
    printf("   ");
    printf("19");
    printf("   ");
    printf("Bit reserved.\r\n");
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
  if(sys_status & DW_GPIOIRQ) {
    printf("   ");
    printf("22");
    printf("   ");
    printf("GPIOIRQ GPIO interrupt.\r\n");
  }
  if(sys_status & DW_SLP2INIT_MASK) {
    printf("   ");
    printf("23");
    printf("   ");
    printf("SLP2INIT SLEEP to INIT.\r\n");
  }
  if(sys_status & DW_RFPLL_LL_MASK) {
    printf("   ");
    printf("24");
    printf("   ");
    printf("RFPLL_LL RF PLL Losing Lock.\r\n");
  }
  if(sys_status & DW_CLKPLL_LL_MASK) {
    printf("   ");
    printf("25");
    printf("   ");
    printf("CLKPLL_LL Clock PLL Losing Lock.\r\n");
  }
  if(sys_status & DW_RXSFDTO_MASK) {
    printf("   ");
    printf("26");
    printf("   ");
    printf("RXSFDTO Receive SFD timeout.\r\n");
  }
  if(sys_status & DW_HPDWARN_MASK) {
    printf("-- ");
    printf("27");
    printf("   ");
    printf("HPDWARN Half Period Delay Warning.\r\n");
  }
  if(sys_status & DW_TXBERR_MASK) {
    printf("-- ");
    printf("28");
    printf("   ");
    printf("DW_TXBERR Transmit Buffer Error..\r\n");
  }
  if(sys_status & DW_AFFREJ_MASK) {
    printf("   ");
    printf("29");
    printf("   ");
    printf("AFFREJ Automatic Frame Filtering rejection.\r\n");
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
  printf("High: \n");
  if((sys_status >> 32) & DW_RXRSCS_MASK) {
    printf("   ");
    printf("0");
    printf("   ");
    printf("RXRSCS Receiver Reed-Solomon Correction Status.\r\n");
  }
  if((sys_status >> 32) & DW_RXPREJ_MASK) {
    printf("   ");
    printf("1");
    printf("   ");
    printf("RXPREJ Receiver Preamble Rejection.\r\n");
  }
  if((sys_status >> 32) & DW_TXPUTE_MASK) {
    printf("   ");
    printf("2");
    printf("   ");
    printf("TXPUTE Transmit power up time error.\r\n");
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
 * \return An approximation of the theoretical time of a transmission 
 *         in microsecond.
 */
unsigned long
theorical_transmission_approx(uint16_t preamble_lenght, uint16_t data_rate, 
                              uint8_t prf, uint32_t data_lenght)
{
  uint16_t t_shr = 0, t_prf = 0, t_mac = 0;
  uint32_t s_mac = 0UL; /* to have a correct precision */

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
 * \brief Compute the theoretical time of the transmission of a paload with 
 *     a length of data_lenght.
 *
 * \param data_rate          The data rate (110, 850 or 8600) in kbps.
 * \param data_lenght        The data length in bytes.
 *
 * \return An approximation of the theoretical time of a transmission 
 *         in millisecond.
 */
unsigned long
theorical_transmission_payload(uint16_t data_rate, uint32_t data_lenght)
{
  uint32_t s_mac = 0UL; /* to have a correct precision */

  /* duration of a data symbol  (*100 000) */
  if(data_rate == DW_DATA_RATE_110_KBPS) {
    s_mac = 820513;
  } else if(data_rate == DW_DATA_RATE_850_KBPS) {
    s_mac = 102564;
  } else { /* data_rate == 6800 kbps */
    s_mac = 12821;
  }

  return (s_mac * ((8 * data_lenght) 
          + ReedSolomonParityBit(data_lenght))) / 100000;
}
/**
 * \brief Convert a duration in micro second to a number of clock ticks
 * \param duration A delay in micro second.
 */
rtimer_clock_t 
microsecond_to_clock_tik(int duration) {
  return ((( (long int) RTIMER_SECOND) * duration) / 1000000) + 1;
}
/**
 * \brief Convert a number of clock ticks to a duration in micro second 
 * \param clock_tiks A number of clock ticks.
 */
int16_t 
clock_ticks_to_microsecond(rtimer_clock_t clock_ticks) {
  return ((long int) (1000000l * clock_ticks) / ((long int) RTIMER_SECOND)) + 1;
}

/** 
 * Compute the propagation time using the Asymmetrical approach of Decawave
 * We use signed number to give the possibilities to have negative 
 * propagation time in the case that the antenna delay was to hight 
 * when we calibrate the nodes.
 *
 **/
int32_t
compute_prop_time(int32_t initiator_roundtrip, int32_t initiator_reply,
  int32_t replier_roundtrip, int32_t replier_reply) {
  return (int32_t)(( ((int64_t) initiator_roundtrip * replier_roundtrip) 
                  - ((int64_t) initiator_reply * replier_reply) )
                /  ((int64_t) initiator_roundtrip 
                  +  replier_roundtrip 
                  +  initiator_reply 
                  +  replier_reply));
}