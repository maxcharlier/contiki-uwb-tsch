#ifndef __DW1000_ARCH_Z1_H__
#define __DW1000_ARCH_Z1_H__

#include "msp430.h"
#include "dev/spi.h"
#include <stdint.h> // To add interger type uint32_t ...
#include "assert.h"


// === DW1000 connected on Z1 "east and south ports" ===
/* P4.0 - Output: SPI Chip Select (CS_N) */
#define DW1000_CSN_PORT(type)    P4##type
#define DW1000_CSN_PIN           0
/* P4.2 - Input: INT from DW1000 */
#define DW1000_IRQ_VECTOR        PORT2_VECTOR
#define DW1000_INT_PORT(type)    P2##type
#define DW1000_INT_PIN           3

/* ENABLE CSn (active low) */
#define DW1000_SPI_ENABLE()     (DW1000_CSN_PORT(OUT) &= ~BV(DW1000_CSN_PIN))
/* DISABLE CSn (active low) */
#define DW1000_SPI_DISABLE()    (DW1000_CSN_PORT(OUT) |= BV(DW1000_CSN_PIN))
#define DW1000_SPI_IS_ENABLED() ((DW1000_CSN_PORT(OUT) & BV(DW1000_CSN_PIN)) != BV(DW1000_CSN_PIN));

void dw1000_arch_init();
void dw1000_us_delay(int ms);

/*===========================================================================*/
/*========================== Device communication ===========================*/ 
void dw_read_subreg(uint32_t reg_addr, uint16_t subreg_addr, uint16_t subreg_len, uint8_t * p_data);
void dw_write_subreg(uint32_t reg_addr, uint16_t subreg_addr, uint16_t subreg_len, const uint8_t * data );

#endif /* __DW1000_ARCH_Z1_H__ */
