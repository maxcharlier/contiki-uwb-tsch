#ifndef __DW1000_ARCH_H__
#define __DW1000_ARCH_H__

#include <stdint.h> // To add interger type uint32_t ...

/*---------------------------------------------------------------------------*/
/**
 * Initialize SPI module & Pins.
 *
 * The function has to accomplish the following tasks:
 * - Enable SPI and configure SPI (CPOL = 0, CPHA = 1)
 * - Configure MISO, MOSI, SCLK accordingly
 * - Configure GPIOx (input)
 * - Configure CSn (output)
 **/
void dw1000_arch_init();
/*---------------------------------------------------------------------------*/
/** 
 * Select DW1000 (pull down CSn pin). 
 **/
inline void dw1000_arch_spi_select(void);
/*---------------------------------------------------------------------------*/
/** 
 * De-select DW1000 (pull up CSn pin). 
 **/
inline void dw1000_arch_spi_deselect(void);
/*---------------------------------------------------------------------------*/
/** 
 * Write a single byte via SPI, return response. 
 **/
int dw1000_arch_spi_rw_byte(uint8_t c);
/*---------------------------------------------------------------------------*/
/** 
 * Write a sequence of bytes while reading back the response.
 * Either read_buf or write_buf can be NULL.
 **/
int dw1000_arch_spi_rw(uint8_t *read_buf, 
                        const uint8_t *write_buf, 
                        uint16_t len);
/*---------------------------------------------------------------------------*/
/**
 * Configure port IRQ for GPIO8.
 *
 *
 * If rising == 1: configure IRQ for rising edge, else falling edge
 * Interrupt has to call dw1000_driver_interrupt()!
 **/
void dw1000_arch_gpio8_setup_irq(void);
/*---------------------------------------------------------------------------*/
/** Reset interrupt flag and enable GPIO8 port IRQ. */
inline void dw1000_arch_gpio8_enable_irq(void);
/*---------------------------------------------------------------------------*/
/* Reset interrupt flag and enable GPIO8 port IRQ. */
inline void dw1000_arch_gpio8_disable_irq(void);
/*---------------------------------------------------------------------------*/
/**
 * Read back the status of the GPIO8 pin.
 * Returns 0 if the pin is low, otherwise 1
 **/
int dw1000_arch_gpio8_read_pin(void);
/*---------------------------------------------------------------------------*/
/**
 * Change the SPI frequency to freq.
 * If freq is bigger than the maximum SPI frequency value of the embedeed 
 * system set this maximum value.
 **/
void dw1000_arch_spi_set_clock_freq(uint32_t freq);
/*---------------------------------------------------------------------------*/
/**
 * Wait a delay in microsecond.
 **/
inline void dw1000_us_delay(int us);

#endif /* __DW1000_ARCH_H__ */
