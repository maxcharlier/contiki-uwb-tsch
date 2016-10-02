/*
 * Contiki Zolertia Z1 -- DW1000 SPI + interrupt test
 *
 * (c) 2015, B. Quoitin (bruno.quoitin@umons.ac.be)
 */



#include "contiki.h"
#include "msp430.h"
#include "sys/clock.h"
#include "watchdog.h"
#include "isr_compat.h"

#include "dw1000-arch.h"
#include "dw1000-driver.h"

 
#if DEBUG
  #include <stdio.h>
  #define PRINTF(...) printf(__VA_ARGS__)
#else
  #define PRINTF(...) do {} while (0)
#endif

/* === Port 2 interrupt vector === */
/* Note : on the z1-dw1000 platform, We have disable interrupt from the
 * "button-sensor", see platform/z1-dw1000/dev/button-sensor.c
 * A more long term solution would be to build a Port2 IRQ handler
 * that would distinguish between button, dw1000, ... and dispatch
 * to more specific ISRs */
ISR(DW1000_IRQ, dw1000_irq_handler){
  PRINTF("dw1000-driver ISR \r\n");

  ENERGEST_ON(ENERGEST_TYPE_IRQ);
  if (P2IFG & BV(DW1000_INT_PIN)) {
    P2IFG &= ~BV(DW1000_INT_PIN);
    dw1000_driver_interrupt();
    LPM4_EXIT;
  }
  P2IFG= 0x00;
  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}


/* === Initialize the architecture specific part of the DW1000 === */
/* Note : for the zolertia z1 platform only */
void dw1000_arch_init()
{
  spi_init();

  /* Configure port direction */
  DW1000_CSN_PORT(DIR) |= BV(DW1000_CSN_PIN);
  DW1000_INT_PORT(DIR) &= ~BV(DW1000_INT_PIN);

  /* Enable interrupts on INT pin */
  /* Note : DW1000's IRQ output is active high by default
   * but can be configured otherwize */
  dint();
  P2IES &= ~BV(DW1000_INT_PIN); /* low to high edge */
  P2IE |= BV(DW1000_INT_PIN); /* enabled */
  eint();

  /* UCCKPL=0   (clock active au niveau haut)
   * UCCKPH=1  (capture sur première transition -- i.e. rising edge,
   *            envoi sur seconde transition -- i.e. falling edge) */
  UCB0CTL0 &= ~UCCKPL;
  UCB0CTL0 |= UCCKPH;

  /* Chip select is disabled (=high) */
  DW1000_SPI_DISABLE();
}

/**
 * Make delay in microsecond
 */
void dw1000_us_delay(int ms){
  int i;
  for (i= 0; i < ms; i++) {
    clock_delay(1);
    watchdog_periodic();
  }
}

/**
 * \brief Reads the value from a subregister on the dw1000 as a byte stream.
 * \param[in] reg_addr      Register address as specified in the manual and by
 *                           the DW_REG_* defines.
 * \param[in] subreg_addr   Subregister address as specified in the manual and
 *                           by the DW_SUBREG_* defines.
 * \param[in] subreg_len    Nunmber of bytes to read. Should not be longer than
 *                           the length specified in the manual or the
 *                           DW_SUBLEN_* defines.
 * \param[out] p_data       Data read from the device.
 */
void dw_read_subreg( uint32_t reg_addr, uint32_t subreg_addr, uint32_t subreg_len, uint8_t * p_data)
{
  int reg_inst = 3;
  int i;
  int n_len = subreg_len + reg_inst;

  // Check if 3-octet header is requried or if 2 will do
  uint8_t isThreeOctet = (uint8_t) (subreg_addr > 0x7FUL ? (1UL << 7) : 0);
  // Prepare instruction
  uint8_t instruction[3] = {0x00, 0x00, 0x00};
  instruction[0] = (uint8_t) (0x40UL | (reg_addr & 0x3FUL)); /* write bit = 0, subreg present bit = 1 */
  instruction[1] = (uint8_t) (isThreeOctet | (subreg_addr & 0x7FUL)); /* isThreeOctet is the extended address bit */

  // Read instruction
  if (isThreeOctet != 0) {
    instruction[2] = (uint8_t) ((subreg_addr & 0x7F80UL) >> 7);
    reg_inst = 3;
  } else {
    reg_inst = 2;
    n_len--;
  }

  // SPI communications

  // Asserting CS
  DW1000_SPI_ENABLE();

  // Read instruction
  for (i = 0; i < reg_inst; i++){
    SPI_WRITE((char) instruction[i]);
  }

  SPI_FLUSH(); /* discard data read during previous write*/

  // Read data
  for (i = reg_inst; i < n_len; i++){
    SPI_READ(*(p_data++));
  }

  // De-asserting CS
  DW1000_SPI_DISABLE();
}

/**
 * \brief Writes a value to a subregister on the dw1000 as a byte stream.
 * \param[in] reg_addr      Register address as specified in the manual and by
 *                           the DW_REG_* defines.
 * \param[in] subreg_addr   Subregister address as specified in the manual and
 *                           by the DW_SUBREG_* defines.
 * \param[in] subreg_len    Nunmber of bytes to write. Should not be longer
 *                           than the length specified in the manual or the
 *                           DW_SUBLEN_* defines.
 * \param[in] p_data        A stream of bytes to write to device.
 */
void dw_write_subreg(uint32_t reg_addr, uint32_t subreg_addr, uint32_t subreg_len, uint8_t *p_data)
{
  int reg_inst = 3;
  int i;
  int n_len = subreg_len + reg_inst;

  // Check if 3-octet header is required or if 2 will do
  uint8_t isThreeOctet = (uint8_t) (subreg_addr > 0x7F ? (1 << 7) : 0);
  // Prepare instruction
  uint8_t instruction[3] = {0x00, 0x00, 0x00};
  instruction[0] = (uint8_t) (0xC0 | (reg_addr & 0x3FUL)); /* write bit = 1, subreg present bit = 1 */
  instruction[1] = (uint8_t) (isThreeOctet | (subreg_addr & 0x7F)); /* isThreeOctet is the extended address bit */

  // Write instruction
  if (isThreeOctet != 0) {
    instruction[2] = (uint8_t) ((subreg_addr & 0x7F80UL) >> 7);
    reg_inst = 3;
  } else {
    reg_inst = 2;
    n_len--;
  }

  // SPI communications

  // Asserting CS
  DW1000_SPI_ENABLE();


  //write instruction
  for (i = 0; i < reg_inst; i++){
    SPI_WRITE( (char) instruction[i]);
  }

  //write data
  for (i = reg_inst; i < n_len; i++){
    SPI_WRITE( (char) *(p_data++));
  }

  // De-asserting CS
  DW1000_SPI_DISABLE();

}

/**
 * \brief Takes several arrays and writes them as a single one to the device.
 * This can be used to write nested data frame structures easily.
 *
 * The following pattern is used in the demo ranging application.
 *     \code
 *     uint32_t   n_data_segments = 2;
 *     uint32_t   data_len[2]     = { DW_FRAME_RANGE_LEN, DW_MSG_RANGE_INIT_LEN };
 *     uint8_t  * pp_data[2];
 *     pp_data[0] = (uint8_t *)&frameRange;
 *     pp_data[1] = (uint8_t *)&msgRangeInit;
 *     dw_transmit_multiple_data( pp_data, data_len, n_data_segments, DW_TRANCEIVE_SYNC );
 *     \endcode
 *
 * \param[in] reg_addr          Destination register on dw1000.
 * \param[in] reg_len           Length of register on dw1000.
 * \param[in] pp_data           Array of data segments.
 * \param[in] p_data_len        Length of each data segment.
 * \param[in] len_pp_data       Length of pp_data.
 */
void dw_write_reg_multiple_data( uint32_t reg_addr, uint32_t reg_len, uint8_t  ** pp_data, uint32_t * p_data_len, uint32_t len_pp_data )
{
  // Get total length of data.
  uint32_t data_len = 0;
  uint32_t length = len_pp_data;
  while (length-- ) {data_len += *p_data_len++;}
  p_data_len-=len_pp_data;


  // Bounds check
  if (data_len > reg_len) {return;}

  // Asserting CS
  DW1000_SPI_ENABLE();
  SPI_WRITE(0x80UL | (reg_addr & 0x3FUL));

  int i_transaction, j_transaction;
  for (i_transaction = 0; i_transaction < len_pp_data-1; ++i_transaction)
  {
    // dw_spi_write_n_bytes2( *p_data_len++, *pp_data++, DW_SPI_TRANSFER_CONT);
    uint8_t* pData = *pp_data++;
    uint32_t pDataLen = *p_data_len++;
    for (j_transaction = 0; j_transaction < pDataLen; j_transaction++){
      SPI_WRITE((unsigned char) *pData++);
    }
  }

  uint8_t* pData = *pp_data;
  for (i_transaction = 0; i_transaction < *p_data_len; i_transaction++){
    SPI_WRITE( (unsigned char) *pData++);
  }

  // De-asserting CS
  DW1000_SPI_DISABLE();
}