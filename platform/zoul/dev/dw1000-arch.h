#ifndef __DW1000_ARCH_Z1_H__
#define __DW1000_ARCH_Z1_H__

#include <stdint.h> // To add interger type uint32_t ...

void dw1000_arch_init();
void dw1000_us_delay(int ms);

/*===========================================================================*/
/*========================== Device communication ===========================*/ 
void dw_read_subreg(uint32_t reg_addr, uint16_t subreg_addr, 
            uint16_t subreg_len, uint8_t *p_data);
void dw_write_subreg(uint32_t reg_addr, uint16_t subreg_addr, 
            uint16_t subreg_len, const uint8_t *data);

#endif /* __DW1000_ARCH_Z1_H__ */
