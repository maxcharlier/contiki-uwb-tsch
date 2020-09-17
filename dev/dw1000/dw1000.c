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
 *         Hardware abstraction library for the DecaWave
 *            DW1000 specific driver.
 *          Based on the work of Hasan Derhamy & Kim Albertsson
 * \author
 *         Charlier Maximilien  <maximilien.charlier@umons.ac.be>
 *         Hasan Derhamy        <hasan.derhamy@ltu.se>
 *         Kim Albertsson       <kim.albertsson@ltu.se>
 */

#include <stdio.h>

#include <string.h>
#include <inttypes.h> /* for print unsigned int */
#include "dw1000.h"
#include "dw1000-arch.h"
#include <stdlib.h>

#include "assert.h"

#define DEBUG 0

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while(0)
#endif

/**
 * \brief Used to convert a timestamps read from the device using
 * \ref dw_get_rx_timestamp or \ref dw_get_tx_timestamp to real seconds.
 */
#define DW_MS_TO_DEVICE_TIME_SCALE 62.6566416e6f


/*===========================================================================*/
/*========================== Public Declarations ============================*/

/**
 * \brief Singleton instance of the DW1000 driver. This instance mirrors the
 * configuration on the actual device. Also provides a global access point to
 * device receive buffer data.
 */
dw1000_base_driver dw1000;

/**
 * \brief Initialize the DW1000.
 *        Enable interrupt for receiver data frame ready event.
 *        Load LDE Code
 *        Initialize channel, data rate, preamble
 *        Define RX configuration
 *        Enable RX, TX, SFD and RK0 LED.
 */
void
dw1000_init()
{
  PRINTF("Decawave Initialising begin\r\n");

  dw1000.state = DW_STATE_INITIALIZING;
  dw1000.auto_ack = 0;
  uint8_t tempRead1[8];
  dw_read_reg(DW_REG_DEV_ID, DW_LEN_DEV_ID, tempRead1);
  print_u8_Array_inHex("REG ID:", tempRead1, DW_LEN_DEV_ID);

  /* Check if SPI communication works by reading device ID */
  assert(0xDECA0130 == dw_read_reg_32(DW_REG_DEV_ID, DW_LEN_DEV_ID));

  /* Init the DW1000 */
  dw_soft_reset(); /* Simple reset of device. */

  dw_clear_pending_interrupt(0x00000007FFFFFFFFULL);
/*  const uint32_t mask = DW_MRXPHE_MASK */
/*                      | DW_MRXDFR_MASK */
/*                      | DW_MRXRFTO_MASK */
/*                      | DW_MLDEERR_MASK */
/*                      | DW_MRXOVRR_MASK */
/*                      | DW_MRXPTO_MASK */
/*                      | DW_MRXSFDTO_MASK */
/*                      | DW_MHPDWARN_MASK */
/*                      | DW_MAFFREJ_MASK; */
  const uint32_t mask = DW_MRXDFR_MASK;
  dw_enable_interrupt(mask);

  dw_load_lde_code();


  /* // Disable LDE */
  /* // TODO: Read old value and flip lderun bit */
  /* //value = dw_read_subreg_64(0x36, 0x04, 4); */
  /* //PRINTF("Value: %llx \r\n", value); */
  /* const uint32_t lderune = 0x81000738; */
  /* dw_write_subreg(0x36, 0x04, 4, (uint8_t *)&lderune); */

  /* dw1000_test(); */

  /* Default broadcast */
  /* dw_set_pan_id_and_short_address(transmission_id); */

  uint8_t tempRead[8];
  dw_read_reg(DW_REG_PANADR, DW_LEN_PANADR, tempRead);
  print_u8_Array_inHex("Reading PAN ID:", tempRead, DW_LEN_PANADR);

  dw_read_reg(DW_REG_EID, DW_LEN_EID, tempRead);

  print_u8_Array_inHex("Reading EID:", tempRead, DW_LEN_EID);

  /* ref data-sheet: Mode 6 */
  dw1000.conf.prf = DW_PRF_16_MHZ;
  dw1000.conf.channel = DW_CHANNEL_5;
  dw1000.conf.preamble_length = DW_PREAMBLE_LENGTH_128;
  dw1000.conf.preamble_code = DW_PREAMBLE_CODE_3;
  dw1000.conf.pac_size = DW_PAC_SIZE_8;
  dw1000.conf.sfd_type = DW_SFD_STANDARD;
  dw1000.conf.data_rate = DW_DATA_RATE_850_KBPS;
  dw_conf(&dw1000.conf);
  dw_turn_frame_filtering_off();

  dw1000_rx_conf_t rx_conf;
  rx_conf.is_delayed = 0;
  rx_conf.dx_timestamp = 0;
  rx_conf.timeout = 0;
  dw_conf_rx(&rx_conf);

  // dw_enable_automatic_receiver_Re_Enable();

  /* Print information about the board */
  PRINTF("Initializing device: %lx\r\n", (unsigned long)dw_get_device_id());

  dw1000.state = DW_STATE_IDLE;
}
/**
 * \brief Configure the transceiver to automatically switching
 *      between TX mode and RX modes.
 *
 * Configuration of the Automatic ACK Turnaround Time (ACK_TIM).
 * DecaWave recommend a min value of 0 at 110 kbps, 2 at 850 kbps and
 * 3 at 6800 kbps [ACK_TIM field]. 
 * But the IEEE 802.15.4 standard specifies a 12 symbol +/- 0.5 symbols 
 * turnaround time for ACK transmission [5.3.2 Automatic Receiver Re-Enable]. 
 * We choose therefore a ACK_TIM of 12 symbols.
 *
 * The Wait-for-Response turn-around Time is set a lower value of ACK_TIM to 
 * avoid loss of symbol.
 */
void
dw_config_switching_tx_to_rx_ACK(void)
{
  uint32_t ack_resp = 0UL;
  /* ACK_TIM of 12 symbols */
  ack_resp |= (0xCUL << DW_ACK_TIM) & DW_ACK_TIM_MASK;

  /* W4R_TIM of 9 symbols */
  ack_resp |= (0x9UL << DW_W4R_TIM) & DW_W4R_TIM_MASK;

  dw_write_reg(DW_REG_ACK_RESP, DW_LEN_ACK_RESP, (uint8_t *)&ack_resp);
}
/**
 * \brief Enable Automatic and frame filtering.
 * Required Frame Filtering On
 */
void
dw_enable_automatic_ack(void)
{
  if(!dw1000.auto_ack) {
    dw_turn_frame_filtering_on(); /* required for automatic ACK */
    uint32_t sys_config = dw_read_reg_32(DW_REG_SYS_CFG, DW_LEN_SYS_CFG);

    sys_config |= DW_AUTOACK_MASK;
    PRINTF("ACK sys config: %u\r\n", (unsigned int)sys_config);

    dw_write_reg(DW_REG_SYS_CFG, DW_LEN_SYS_CFG, (uint8_t *)&sys_config);
    dw1000.auto_ack = 1;
  }
}
/**
 * \brief Disable Automatic Acknowledge
 */
void
dw_disable_automatic_ack(void)
{
  if(dw1000.auto_ack) {
    uint32_t sys_config = dw_read_reg_32(DW_REG_SYS_CFG, DW_LEN_SYS_CFG);
    sys_config &= ~DW_AUTOACK_MASK;
    dw_write_reg(DW_REG_SYS_CFG, DW_LEN_SYS_CFG, (uint8_t *)&sys_config);
    dw1000.auto_ack = 0;
  }
}

/**
 * \brief Return if the Automatic acknoledgement is enable.
 */
uint8_t dw_is_automatic_ack(void){
  return dw1000.auto_ack;
}
/**
 * \brief Enable Receiver Auto-Re-enable.
 */
void
dw_enable_automatic_receiver_Re_Enable()
{
  uint32_t sys_config = dw_read_reg_32(DW_REG_SYS_CFG, DW_LEN_SYS_CFG);
  sys_config |= DW_RXAUTR_MASK;
  dw_write_reg(DW_REG_SYS_CFG, DW_LEN_SYS_CFG, (uint8_t *)&sys_config);
}
/**
 * \brief Disable Receiver Auto-Re-enable.
 */
void
dw_disable_automatic_receiver_Re_Enable()
{
  uint32_t sys_config = dw_read_reg_32(DW_REG_SYS_CFG, DW_LEN_SYS_CFG);
  sys_config &= ~DW_RXAUTR_MASK;
  dw_write_reg(DW_REG_SYS_CFG, DW_LEN_SYS_CFG, (uint8_t *)&sys_config);
}
/**
 * \brief Turn on frame filtering
 *    Frame Filtering Allow   Beacon frame reception
 *                Data frame reception.
 *                Acknowledgment frame reception.
 *                MAC command frame reception.
 *
 */
void
dw_turn_frame_filtering_on(void)
{
  uint32_t frameFilteringData = dw_read_reg_32(DW_REG_SYS_CFG, DW_LEN_SYS_CFG);
  PRINTF("Reading frameFilteringData:  0x%08X\r\n",  
              (unsigned long long)  frameFilteringData);

  frameFilteringData &= 0xFFFFFE00;   /* Clear Filtering bit */
  frameFilteringData |= DW_FFEN_MASK; /* Frame Filtering Enable. */
  frameFilteringData |= DW_FFBC_MASK; /* Frame Filtering Behave as a 
                                          Coordinator. */
  frameFilteringData |= DW_FFAB_MASK; /* Frame Filtering Allow Beacon frame 
                                          reception. */
  frameFilteringData |= DW_FFAD_MASK; /* Frame Filtering Allow Data frame 
                                          reception. */
  frameFilteringData |= DW_FFAA_MASK; /* Frame Filtering Allow Acknowledgment 
                                          frame reception. */
  frameFilteringData |= DW_FFAM_MASK; /* Frame Filtering Allow MAC command frame
                                          reception. */
  PRINTF("Modified frameFilteringData: 0x%08X\r\n", frameFilteringData);

  /* send new filtering configuration */
  dw_write_reg(DW_REG_SYS_CFG, DW_LEN_SYS_CFG, (uint8_t *)&frameFilteringData);

#if DEBUG
  frameFilteringData = dw_read_reg_32(DW_REG_SYS_CFG, DW_LEN_SYS_CFG);
  PRINTF("Reading new frameFilteringData: %08x\r\n", frameFilteringData); 
#endif /* DEBUG */

}
/**
 * \brief Turn off frame filtering and automatic ACK.
 *
 */
void
dw_turn_frame_filtering_off(void)
{
  /* read current value from system configuration register */
  uint32_t frameFilteringData = dw_read_reg_32(DW_REG_SYS_CFG, DW_LEN_SYS_CFG);
  /* switch all filtering off and disable filtering */
  /* switch it all off */
  frameFilteringData &= ~(DW_CFG_FF_ALL_EN | DW_FFEN_MASK); 
  dw_write_reg(DW_REG_SYS_CFG, DW_LEN_SYS_CFG, (uint8_t *) &frameFilteringData);
}
/**
 * \brief Return if the frame filtering is enabled.
 */
uint8_t dw_is_frame_filtering_on(void){
  uint8_t frameFilteringData = 0;
  /* we only read the first byte */
  dw_read_subreg(DW_REG_SYS_CFG, 0, 1, (uint8_t *)&frameFilteringData);
  return frameFilteringData & DW_FFEN_MASK; /* Frame Filtering Enable bit */
}



/**
 * \brief Keep the receiver on the reception when the Reed Solomon decoder detects
 * an non-correctable error.
 *
 */
void
dw_disable_receive_abort_on_RSD_error(void)
{
  /* read current value from system configuration register */
  uint32_t sys_cfg = dw_read_reg_32(DW_REG_SYS_CFG, DW_LEN_SYS_CFG);
  /* switch all filtering off and disable filtering */
  /* switch it all off */
  sys_cfg |= DW_DIS_RSDE_MASK; 
  dw_write_reg(DW_REG_SYS_CFG, DW_LEN_SYS_CFG, (uint8_t *) &sys_cfg);
}

/**
 * \brief Disable the reception when the Reed Solomon decoder detects
 * an non-correctable error. (Default state)
 *
 */
void
dw_enable_receive_abort_on_RSD_error(void)
{
  /* read current value from system configuration register */
  uint32_t sys_cfg = dw_read_reg_32(DW_REG_SYS_CFG, DW_LEN_SYS_CFG);
  /* switch all filtering off and disable filtering */
  /* switch it all off */
  sys_cfg &= ~(DW_DIS_RSDE | DW_DIS_RSDE_MASK); 
  dw_write_reg(DW_REG_SYS_CFG, DW_LEN_SYS_CFG, (uint8_t *) &sys_cfg);
}

/**
 * \brief Enable the extended frame format.
 *      The default setting gives IEEE standard PHR encoding and a maximum data
 *      payload of 127 octets. The other option enables the proprietary long
 *      frames mode which allows a data payload of up to 1023 octets.
 *      In this mode the PHR encoding does not follow the IEEE standard.
 */
void
dw_enable_extended_frame(void)
{
  uint32_t sys_cfg = dw_read_reg_32(DW_REG_SYS_CFG, DW_LEN_SYS_CFG);
  sys_cfg |= DW_PHR_MODE_MASK;
  dw_write_reg(DW_REG_SYS_CFG, DW_LEN_SYS_CFG, (uint8_t *)&sys_cfg);
}
/**
 * \brief Disable the extended frame format.
 *      The default setting gives IEEE standard PHR encoding and a maximum data
 *      payload of 127 octets. The other option enables the proprietary long
 *      frames mode which allows a data payload of up to 1023 octets.
 *      In this mode the PHR encoding does not follow the IEEE standard.
 */
void
dw_disable_extended_frame(void)
{
  uint32_t sys_cfg = dw_read_reg_32(DW_REG_SYS_CFG, DW_LEN_SYS_CFG);
  sys_cfg &= ~DW_PHR_MODE_MASK;
  dw_write_reg(DW_REG_SYS_CFG, DW_LEN_SYS_CFG, (uint8_t *)&sys_cfg);
}
/**
 * Re-enable LED blink mode after deepsleep.
 *
 * LED wil blink with a lower intensity if the wakeup period is short.
 * For exemple with a wakeup duration of 3 ms, the led only stay on for 3 ms 
 * in place of 20 ms.
 */
void
dw_enable_gpio_led_from_deepsleep(void)
{
  uint16_t data = 0UL;
  /* PMSC_CTRL0 bit DW_GPDCE is not saved */
  dw_read_subreg(DW_REG_PMSC, DW_SUBREG_PMSC_CTRL0+2, 1, (uint8_t *) &data);

  data |= (1UL << (DW_GPDCE - 16)) & (DW_GPDCE_MASK >> 16); /* GPIO De-bounce Clock Enable. */
  dw_write_subreg(DW_REG_PMSC, DW_SUBREG_PMSC_CTRL0+2, 1, (uint8_t *) &data);

  /* active blinking mode */
  /* BLINK_TIM fiedl is 16 bits long */
  data = (0xF << DW_BLINK_TIM) & DW_BLINK_TIM_MASK; /* blink time to 20 ms 
                                                        (default 400 ms) */

  // dw_write_subreg(DW_REG_PMSC, DW_SUBREG_PMSC_LEDC, 2, (uint8_t *) &data);
}
/**
 * Configure GPIO mode:
 *    Enable interrupt TX, RX,
 *    Enable RX, TX, SFD and RK0 LED.
 */
void
dw_enable_gpio_led(void)
{
  uint32_t data = 0UL;
  /* set GPIO to LED */
  data = dw_read_subreg_32(DW_REG_GPIO_CTRL, DW_SUBREG_GPIO_MODE, 
                            DW_SUBLEN_GPIO_MODE);
  data |= (1UL << DW_MSGP0) & DW_MSGP0_MASK; /* set GPIO0 as the RXOKLED
                                                  output. */
  data |= (1UL << DW_MSGP1) & DW_MSGP1_MASK; /* set GPIO1 as the SFDLED
                                                  output. */
  data |= (1UL << DW_MSGP2) & DW_MSGP2_MASK; /* set GPIO2 as the RXLED
                                                  output. */
  data |= (1UL << DW_MSGP3) & DW_MSGP3_MASK; /* set GPIO3 as the TXLED
                                                  output. */
  dw_write_subreg(DW_REG_GPIO_CTRL, DW_SUBREG_GPIO_MODE, DW_SUBLEN_GPIO_MODE, 
                  (uint8_t *)&data);
  /* required: see manual p.182 */
  data = dw_read_subreg_32(DW_REG_PMSC, DW_SUBREG_PMSC_CTRL0, 
                            DW_SUBLEN_PMSC_CTRL0);
  data |= (1UL << DW_GPDCE) & DW_GPDCE_MASK; /* GPIO De-bounce Clock Enable. */
  data |= (1UL << DW_KHZCLKEN) & DW_KHZCLKEN_MASK; /* Kilohertz clock Enable. */
  dw_write_subreg(DW_REG_PMSC, DW_SUBREG_PMSC_CTRL0, DW_SUBLEN_PMSC_CTRL0, 
                  (uint8_t *)&data);

  /* active blinking mode */
  data = dw_read_subreg_32(DW_REG_PMSC, DW_SUBREG_PMSC_LEDC, 
                            DW_SUBLEN_PMSC_LEDC);
  data |= (1UL << DW_BLNKEN) & DW_BLNKEN_MASK; /* enable blink mode */
  data |= (0xFUL << DW_BLNKNOW) & DW_BLNKNOW_MASK; /* force LEDs to blink 
                                                      once */
  data &= ~DW_BLINK_TIM_MASK; /* set Blink time count value to 0 */
  data |= (0xF << DW_BLINK_TIM) & DW_BLINK_TIM_MASK; /* blink time to 20 ms 
                                                        (default 400 ms) */

  dw_write_subreg(DW_REG_PMSC, DW_SUBREG_PMSC_LEDC, DW_SUBLEN_PMSC_LEDC, 
                  (uint8_t *)&data);

  /* reset force blink bits. Needed to make the LEDs blinking */
  data &= ~((0xFUL << DW_BLNKNOW) & DW_BLNKNOW_MASK); 

  dw_write_subreg(DW_REG_PMSC, DW_SUBREG_PMSC_LEDC, DW_SUBLEN_PMSC_LEDC, 
                  (uint8_t *)&data);
}
/**
 * \brief Configure GPIO mode:
 *        Disable interrupt TX, RX,
 *        DIsable RX, TX, SFD and RK0 LED.
 */
void
dw_disable_gpio_led(void)
{
  uint32_t data = 0UL;
  /* set GPIO to LED */
  data = dw_read_subreg_32(DW_REG_GPIO_CTRL, DW_SUBREG_GPIO_MODE, 
                            DW_SUBLEN_GPIO_MODE);
  data &= ~DW_MSGP0_MASK; /* reset GPIO0 */
  data &= ~DW_MSGP1_MASK; /* reset GPIO1 */
  data &= ~DW_MSGP2_MASK; /* reset GPIO2 */
  data &= ~DW_MSGP3_MASK; /* reset GPIO3. */
  dw_write_subreg(DW_REG_GPIO_CTRL, DW_SUBREG_GPIO_MODE, DW_SUBLEN_GPIO_MODE, 
                  (uint8_t *)&data);

  /* required: see manual p.182 */
  data = dw_read_subreg_32(DW_REG_PMSC, DW_SUBREG_PMSC_CTRL0, 
                            DW_SUBLEN_PMSC_CTRL0);
  data &= ~DW_GPDCE_MASK; /* reset GPIO De-bounce Clock. */
  data &= ~DW_KHZCLKEN_MASK; /* reset Kilohertz clock. */
  dw_write_subreg(DW_REG_PMSC, DW_SUBREG_PMSC_CTRL0, DW_SUBLEN_PMSC_CTRL0, 
                  (uint8_t *)&data);

  /* disable blinking mode */
  data = dw_read_subreg_32(DW_REG_PMSC, DW_SUBREG_PMSC_LEDC, 
                            DW_SUBLEN_PMSC_LEDC);
  data &= ~DW_BLNKEN_MASK; /* reset blink mode */
  data &= ~DW_BLNKNOW_MASK; /* force LEDs to blink once */
  data &= ~DW_BLINK_TIM_MASK; /* reset Blink time count value */
  dw_write_subreg(DW_REG_PMSC, DW_SUBREG_PMSC_LEDC, DW_SUBLEN_PMSC_LEDC, 
                  (uint8_t *)&data);
}
/**
 * \brief Set a SFD timeout value. This is used to abort a reception when a 
 *        preamble is detected but not a SFD. 
 *        With more details: The SFD detection timeout starts running as soon as
 *        preamble is detected. If the SFD sequence is not detected before the 
 *        timeout period expires then the timeout will act to abort the 
 *        reception currently in progress.
 *
 * \param value the SFD timeout value (expressed in preamble symbol).
 *
 *        The value is set in function of the configuration (preamble length 
 *        and data rate).
 *
 *        Example of value (from 
 *        https://github.com/damaki/DW1000/wiki/Configuring-the-DW1000)
 *        Expected Rx Preamble Length   Data Rate     Recommended SFD Timeout
 *          64                            110 kbps      64 + 64 + 1
 *          64                            850 kbps      64 + 8 + 1
 *          64                            6.8 Mbps      64 + 8 + 1
 *          1024                          110 kbps      1024 + 64 + 1
 *          1024                          850 kbps      1024 + 8 + 1
 *          4096                          6.8 Mbps      4096 + 8 + 1
 *
 *      /!\ Please do NOT set DRX_SFDTOC to zero (disabling SFD detection 
 *          timeout). With the SFD timeout disabled and in the event of false 
 *          preamble detection, the IC will remain in receive mode until 
 *          commanded to do otherwise by the external micro-controller. This can
 *          lead to significant reduction in battery life.
 *      ==> If you set a value of 0 this value will replace by the default value
 *          4096+64+1 symbols
 */
void
dw_set_sfd_timeout(uint16_t value){
  if(value == 0)
    value = 4096+64+1;
  dw_write_subreg(DW_REG_DRX_CONF, DW_SUBREG_DRX_SFDTOC, DW_SUBLEN_DRX_SFDTOC, 
                      (uint8_t *) &value);
}
/* \brief SFD initialization: This can be done by writing to the system control
 * Control Register with both the transmission start-bit TXSTRT and the 
 * transceiver off bit TRXOFF set at the same time. [5.3.1.2 SFD Initialization]
 */
void
dw_sfd_init(void){
  uint32_t sys_ctrl = 0UL;
  dw_read_reg(DW_REG_SYS_CTRL, DW_LEN_SYS_CTRL, (uint8_t *)&sys_ctrl);
  sys_ctrl |= DW_TXSTRT_MASK | DW_TRXOFF_MASK;
  dw_write_reg(DW_REG_SYS_CTRL, DW_LEN_SYS_CTRL, (uint8_t *)&sys_ctrl);
  /* bits are automatically clear by the transceiver */ 
}
/**
 * \brief Force the load of the LDE code from the ROM memory to the RAM memory 
 *    to be able to compute correctly the timestamps. 
 *  See LDELOAD in the manual (Table 4)
 *    /!\ Warning:
 *    Ensure that the SPI operating frequency is set < 3MHz. 
 *    (During procedure the system uses the 19.2 MHz XTI clock which will
 *    not support higher SPI data rates)
 */
void
dw_load_lde_code(void){
  const uint16_t lde1 = 0x0301;
  const uint16_t lde = DW_LDELOAD_MASK; /* load LDE code */
  const uint8_t lde3[2] = {0x00, 0x02};

  dw_write_subreg(DW_REG_PMSC, DW_SUBREG_PMSC_CTRL0, 2, (uint8_t *)&lde1);
  dw_write_subreg(DW_REG_OTP_IF, DW_SUBREG_OTP_CTRL, 2, (uint8_t *)&lde);
  dw1000_us_delay(150); /* Wait at least 150 us > see Table 4 p24 */

  /* From DecaRanging software:
     Need to write lower byte separately before setting the higher byte(s) */
  dw_write_subreg(DW_REG_PMSC, DW_SUBREG_PMSC_CTRL0, 1, &lde3[0]);
  dw_write_subreg(DW_REG_PMSC, DW_SUBREG_PMSC_CTRL0 + 1, 1, &lde3[1]);
}
/**
 * \brief Enable the automatic load of the LDE code on Wake UP (after a SLEEP).
 */
void
dw_active_lde_on_wakeup(void){
  uint16_t value = 0;
  /* active load lde wake up */
  dw_read_subreg(DW_REG_AON, DW_SUBREG_AON_WCFG, 2, (uint8_t *)&value);
  value |= (0x1 << DW_ONW_LLDE) & DW_ONW_LLDE_MASK;
  dw_write_subreg(DW_REG_AON, DW_SUBREG_AON_WCFG, 2, (uint8_t *)&value);
}
/**
 * \brief Apply a soft reset
 *    /!\ Warning:
 *    Ensure that the SPI operating frequency is set < 3MHz. 
 *    (During procedure the system uses the 19.2 MHz XTI clock which will
 *    not support higher SPI data rates)
 */
void
dw_soft_reset(void)
{
  /* Set SYSCLKS to 01 > Force system clock to be the 19.2 MHz XTI clock. */
  uint32_t ctrlReg = 0UL;
  dw_read_subreg(DW_REG_PMSC, DW_SUBREG_PMSC_CTRL0, DW_SUBLEN_PMSC_CTRL0, 
                 (uint8_t *)&ctrlReg);
  ctrlReg &= ~DW_SYSCLKS_MASK;
  ctrlReg |= (0x01 << DW_SYSCLKS) & DW_SYSCLKS_MASK;
  dw_write_subreg(DW_REG_PMSC, DW_SUBREG_PMSC_CTRL0, DW_SUBLEN_PMSC_CTRL0, 
                  (uint8_t *)&ctrlReg);

  /* Clear SOFTRESET >> all zeros */
  ctrlReg &= ~DW_SOFTRESET_MASK;
  dw_write_subreg(DW_REG_PMSC, DW_SUBREG_PMSC_CTRL0, DW_SUBLEN_PMSC_CTRL0, 
                  (uint8_t *)&ctrlReg);

  /* Source: 10us sleep : 
   * https://github.com/lab11/dw1000-driver/blob/master/deca_device.c
   * The DW1000 needs a 10us sleep to let clk PLL lock after reset - 
   * the PLL will automatically lock after the reset */
  dw1000_us_delay(10);

  /* Set SOFTRESET to all ones */
  ctrlReg |= DW_SOFTRESET_MASK; /* Set SOFTRESET to all ones */
  dw_write_subreg(DW_REG_PMSC, DW_SUBREG_PMSC_CTRL0, DW_SUBLEN_PMSC_CTRL0, 
                  (uint8_t *)&ctrlReg);

  ctrlReg &= ~DW_SYSCLKS_MASK; /* Set SYSCLKS to 00 > Auto system clock. */
  dw_write_subreg(DW_REG_PMSC, DW_SUBREG_PMSC_CTRL0, DW_SUBLEN_PMSC_CTRL0, 
                  (uint8_t *)&ctrlReg);

  dw_idle(); /* force to idle */
}
/**
 * \brief Uploads and applies a given configuration to the DW1000.
 *
 * \param[in] dw_conf   Configuration to be applied.
 */
void
dw_conf(dw1000_base_conf_t *dw_conf)
{
  const uint32_t agc_tune2_val = 0X2502A907UL;  /* Always use this */
  const uint16_t agc_tune3_val = 0x0035;    /* Always use this */

  /* === Configure PRF */
  dw_set_prf(dw_conf->prf);

  /* === Configure rx/tx channel */
  dw_set_channel(dw_conf->channel);

  dw_set_manual_tx_power(dw_conf->channel, dw_conf->prf);

  /* === Configure Preamble length */
  dw_set_preamble_length(dw_conf->preamble_length);

  /* === Configure Preamble code */
  dw_set_preamble_code(dw_conf->preamble_code);

  /* === Configure LDE Replica Coefficient */
  dw_lde_repc_config(dw_conf->preamble_code, dw_conf->data_rate);
  dw_configure_lde(dw_conf->preamble_code);

  /* === Configure PAC size */
  dw_set_pac_size(dw_conf->pac_size, dw_conf->prf);

  /* === Configure SFD  and data rate */
  dw_set_datarate_and_sfd(dw_conf->data_rate, dw_conf->sfd_type);

  /* == Configure SFD timeout */
  /* preamble length + 1 + SFD length - PAC size */
  if(dw_conf->data_rate == DW_DATA_RATE_110_KBPS) {
    dw_set_sfd_timeout(dw_conf->preamble_length
                       + 1
                       + 64 /* SFD length */
                       - dw_conf->pac_size);
  }
  else{
    dw_set_sfd_timeout(dw_conf->preamble_length
                         + 1
                         + 16 /* SFD length */
                         - dw_conf->pac_size);
  }


  /* We adjust the crystal frequency.
    We use the mid range value (0x0F) */
  dw_fs_xtalt(0xFU);

  /* Commit configuration to device */
  dw_write_subreg(DW_REG_AGC_CTRL, DW_SUBREG_AGC_TUNE2, DW_SUBLEN_AGC_TUNE2,
                  (uint8_t *) &agc_tune2_val);
  dw_write_subreg(DW_REG_AGC_CTRL, DW_SUBREG_AGC_TUNE3, DW_SUBLEN_AGC_TUNE3,
                  (uint8_t *) &agc_tune3_val);
  dw1000.conf = *dw_conf;
  /* DW_LOG("Configuration complete."); */
}
/**
 * \brief Configure the DW1000 according to a channel.
 *    Include the configuration of RF_TXCTRL (Analog TX Control Register),
 *      RF_TXCTRL (Analog TX Control Register),
 *      TC_PGDELAY (Transmitter Calibration - Pulse Generator Delay),
 *      FS_PLLCFG (Frequency synthesizer - PLL configuration),
 *      FS_PLLTUNE (Frequency synthesizer - PLL Tuning)
 *
 * \param[in] channel   The channel.
 */
void dw_set_channel(dw1000_channel_t channel){
  /*  Channel Control Register */
  uint8_t chan_ctrl_val = 0; /* 8 first bits on the Channel control register */
  /* Analog RX Control Register : define if we use 500 or 1300 MHz channel */
  uint8_t rf_rxctrlh_val = 0; /* 8 bits register */
  /* Analog TX Control Register */
  uint32_t rf_txctrl_val = 0UL; /* 24 bits register */
  /* Transmitter Calibration - Pulse Generator Delay */
  uint8_t tc_pgdelay_val = 0;
  /* Frequency synthesiser – PLL configuration */
  uint32_t fs_pllcfg_val = 0UL;
  /* Frequency synthesiser – PLL Tuning */
  uint8_t fs_plltune_val = 0;
  
  /* === Configure rx/tx channel */
  uint8_t channelNum = ((uint8_t)channel & 0xF);
  chan_ctrl_val |= (channelNum << DW_TXCHAN) & DW_TXCHAN_MASK;
  chan_ctrl_val |= (channelNum << DW_RXCHAN) & DW_RXCHAN_MASK;

  switch(channel) {
  case DW_CHANNEL_1:
    rf_rxctrlh_val = 0xD8;
    rf_txctrl_val = 0x00005C40UL;
    tc_pgdelay_val = 0xC9;
    fs_pllcfg_val = 0x09000407UL;
    fs_plltune_val = 0x1E;
    break;
  case DW_CHANNEL_2:
    rf_rxctrlh_val = 0xD8;
    rf_txctrl_val = 0x00045CA0UL;
    tc_pgdelay_val = 0xC2;
    fs_pllcfg_val = 0x08400508UL;
    fs_plltune_val = 0x26;
    break;
  case DW_CHANNEL_3:
    rf_rxctrlh_val = 0xD8;
    rf_txctrl_val = 0x00086CC0UL;
    tc_pgdelay_val = 0xC5;
    fs_pllcfg_val = 0x08401009UL;
    fs_plltune_val = 0x56;
    break;
  case DW_CHANNEL_4:
    rf_rxctrlh_val = 0xBC;
    rf_txctrl_val = 0x00045C80UL;
    tc_pgdelay_val = 0x95;
    fs_pllcfg_val = 0x08400508UL;
    fs_plltune_val = 0x26;
    break;
  case DW_CHANNEL_5:
    rf_rxctrlh_val = 0xD8;
    rf_txctrl_val = 0x001E3FE3UL;
    tc_pgdelay_val = 0xB5;
    fs_pllcfg_val = 0x0800041DUL;
    fs_plltune_val = 0xBE;
    break;
  case DW_CHANNEL_7:
    rf_rxctrlh_val = 0xBC;
    rf_txctrl_val = 0x001E7DE0UL;
    tc_pgdelay_val = 0x93;
    fs_pllcfg_val = 0x0800041DUL;
    fs_plltune_val = 0xBE;
    break;
  }

  /* we only rewite the channel used */
  dw_write_subreg(DW_REG_CHAN_CTRL, 0, 1, (uint8_t *) &chan_ctrl_val);

  dw_write_subreg(DW_REG_FS_CTRL, DW_SUBREG_FS_PLLCFG, DW_SUBLEN_FS_PLLCFG,
                  (uint8_t *) &fs_pllcfg_val);
  dw_write_subreg(DW_REG_RF_CONF, DW_SUBREG_RF_RXCTRLH, DW_SUBLEN_RF_RXCTRLH,
                  (uint8_t *) &rf_rxctrlh_val);
  dw_write_subreg(DW_REG_RF_CONF, DW_SUBREG_RF_TXCTRL, DW_SUBLEN_RF_TXCTRL,
                    (uint8_t *) &rf_txctrl_val);
  dw_write_subreg(DW_REG_TX_CAL, DW_SUBREG_TC_PGDELAY, DW_SUBLEN_TC_PGDELAY,
                  (uint8_t *) &tc_pgdelay_val);
  dw_write_subreg(DW_REG_FS_CTRL, DW_SUBREG_FS_PLLTUNE, DW_SUBLEN_FS_PLLTUNE,
                  (uint8_t *) &fs_plltune_val);
}
/**
 * \brief Configure the transceiver according to the PRF. 
 **/
void
dw_set_prf(dw1000_prf_t prf){
  /* Transmit Frame Control */
  uint8_t tx_fctrl_val = 0;
  /* Channel Control Register */
  uint8_t chan_ctrl_val = 0;
  /* Automatic Gain Control configuration and control Tuning register 1 */
  uint16_t agc_tune1_val;
  /* Digital Receiver Configuration Tuning Register 1a */
  uint16_t drx_tune1a_val;

  /* we focus on the PRF bits : TXPRF (bit 16 and 17). Thus we only touch the 
  3nd byte */
  dw_read_subreg(DW_REG_TX_FCTRL, 2, 1, (uint8_t *)&tx_fctrl_val);
  tx_fctrl_val &= ~(DW_TXPRF_MASK >> 16);

  /* we focus on the PRF bits : RXPRF (bit 18 and 19). Thus we only touch the 
  3nd byte */
  dw_read_subreg(DW_REG_CHAN_CTRL, 2, 1, (uint8_t *)&chan_ctrl_val);
  chan_ctrl_val &= ~(DW_RXPRF_MASK >> 16);

  switch(prf) {
    case DW_PRF_16_MHZ:
      agc_tune1_val   = 0x8870;
      drx_tune1a_val  = 0x0087;
      tx_fctrl_val   |= (0x01UL << (DW_TXPRF - 16)) & (DW_TXPRF_MASK >> 16);
      chan_ctrl_val  |= (0x01UL << (DW_RXPRF - 16)) & (DW_RXPRF_MASK >> 16);
      break;

    case DW_PRF_64_MHZ:
      agc_tune1_val   = 0x889B;
      drx_tune1a_val  = 0x008D;
      tx_fctrl_val   |= (0x02UL << (DW_TXPRF - 16)) & (DW_TXPRF_MASK >> 16);
      chan_ctrl_val  |= (0x02UL << (DW_RXPRF - 16)) & (DW_RXPRF_MASK >> 16);
      break;
  }

  dw_write_subreg(DW_REG_TX_FCTRL, 2, 1, (uint8_t *)&tx_fctrl_val);
  dw_write_subreg(DW_REG_CHAN_CTRL, 2, 1, (uint8_t *)&chan_ctrl_val);
  dw_write_subreg(DW_REG_AGC_CTRL, DW_SUBREG_AGC_TUNE1, DW_SUBLEN_AGC_TUNE1,
                  (uint8_t *) &agc_tune1_val);
  dw_write_subreg(DW_REG_DRX_CONF, DW_SUBREG_DRX_TUNE1a, DW_SUBLEN_DRX_TUNE1a,
                  (uint8_t *) &drx_tune1a_val);
}

/**
 * \Brief Configure the transceiver according to the preamble length 
 **/
void
dw_set_preamble_length(dw1000_preamble_length_t preamble_length){

  /* Transmit Frame Control */
  uint16_t tx_fctrl_val = 0;
  /* Digital Receiver Configuration - Tuning Register 1b */
  uint16_t drx_tune1b_val = 0;
  /* Digital Receiver Configuration - Tuning Register 4H */
  uint16_t drx_tune4h_val = 0;

  if(preamble_length == DW_PREAMBLE_LENGTH_64) {
    drx_tune1b_val = 0x0010;
  } 
  else if(preamble_length <= DW_PREAMBLE_LENGTH_1024) {
    drx_tune1b_val = 0x0020;
  } 
  else if(preamble_length > DW_PREAMBLE_LENGTH_1024) {
    drx_tune1b_val = 0x0064;
  }

  if(preamble_length == DW_PREAMBLE_LENGTH_64) {
    drx_tune4h_val = 0x0010;
  } 
  else {
    drx_tune4h_val = 0x0028;
  }
  
  /* we focus on the TXPSR bits (bit 18 and 19) and PE (20 and 21) 
  Thus we only touch the 3nd and 4nd byte */
  dw_read_subreg(DW_REG_TX_FCTRL, 2, 2, (uint8_t *)&tx_fctrl_val);
  tx_fctrl_val &= ~(DW_TXPSR_MASK >> 16);
  tx_fctrl_val &= ~(DW_PE_MASK >> 16);
  switch(preamble_length) {
  case DW_PREAMBLE_LENGTH_64:
    tx_fctrl_val |= (0x01UL << (DW_TXPSR - 16)) & (DW_TXPSR_MASK >> 16);
    tx_fctrl_val |= (0x00UL << (DW_PE - 16)) & (DW_PE_MASK >> 16);
    break;
  case DW_PREAMBLE_LENGTH_128:
    tx_fctrl_val |= (0x01UL << (DW_TXPSR - 16)) & (DW_TXPSR_MASK >> 16);
    tx_fctrl_val |= (0x01UL << (DW_PE - 16)) & (DW_PE_MASK >> 16);
    break;
  case DW_PREAMBLE_LENGTH_256:
    tx_fctrl_val |= (0x01UL << (DW_TXPSR - 16)) & (DW_TXPSR_MASK >> 16);
    tx_fctrl_val |= (0x02UL << (DW_PE - 16)) & (DW_PE_MASK >> 16);
    break;
  case DW_PREAMBLE_LENGTH_512:
    tx_fctrl_val |= (0x01UL << (DW_TXPSR - 16)) & (DW_TXPSR_MASK >> 16);
    tx_fctrl_val |= (0x03UL << (DW_PE - 16)) & (DW_PE_MASK >> 16);
    break;
  case DW_PREAMBLE_LENGTH_1024:
    tx_fctrl_val |= (0x02UL << (DW_TXPSR - 16)) & (DW_TXPSR_MASK >> 16);
    tx_fctrl_val |= (0x00UL << (DW_PE - 16)) & (DW_PE_MASK >> 16);
    break;
  case DW_PREAMBLE_LENGTH_1536:
    tx_fctrl_val |= (0x02UL << (DW_TXPSR - 16)) & (DW_TXPSR_MASK >> 16);
    tx_fctrl_val |= (0x01UL << (DW_PE - 16)) & (DW_PE_MASK >> 16);
    break;
  case DW_PREAMBLE_LENGTH_2048:
    tx_fctrl_val |= (0x02UL << (DW_TXPSR - 16)) & (DW_TXPSR_MASK >> 16);
    tx_fctrl_val |= (0x02UL << (DW_PE - 16)) & (DW_PE_MASK >> 16);
    break;
  case DW_PREAMBLE_LENGTH_4096:
    tx_fctrl_val |= (0x03UL << (DW_TXPSR - 16)) & (DW_TXPSR_MASK >> 16);
    tx_fctrl_val |= (0x00UL << (DW_PE - 16)) & (DW_PE_MASK >> 16);
    break;
  }
  dw_write_subreg(DW_REG_TX_FCTRL, 2, 2, (uint8_t *)&tx_fctrl_val);

  dw_write_subreg(DW_REG_DRX_CONF, DW_SUBREG_DRX_TUNE1b, DW_SUBLEN_DRX_TUNE1b,
                  (uint8_t *) &drx_tune1b_val);

  dw_write_subreg(DW_REG_DRX_CONF, DW_SUBREG_DRX_TUNE4h, DW_SUBLEN_DRX_TUNE4h,
                  (uint8_t *) &drx_tune4h_val);
}
/**
 * \brief Configure the transceiver according to the preamble code.
 **/
void
dw_set_preamble_code(dw1000_preamble_code_t preamble_code){
  /* Channel Control Register */
  uint16_t chan_ctrl_val = 0;
  /* For the Channel Control Register, we focus on the TX_PCODE (bit 22 to 26)
    and the RX_PCODE (bit 27 to 31).
    Thus we only touch the 3nd and 4nd byte */
  dw_read_subreg(DW_REG_CHAN_CTRL, 2, 2, (uint8_t *)&chan_ctrl_val);

  chan_ctrl_val &= ~(DW_TX_PCODE_MASK >> 16);
  chan_ctrl_val &= ~(DW_RX_PCODE_MASK >> 16);

  chan_ctrl_val |= (((uint32_t) preamble_code) << (DW_TX_PCODE - 16)) 
                    & (DW_TX_PCODE_MASK >> 16);
  chan_ctrl_val |= (((uint32_t) preamble_code) << (DW_RX_PCODE - 16)) 
                    & (DW_RX_PCODE_MASK >> 16);

  dw_write_subreg(DW_REG_CHAN_CTRL, 2, 2, (uint8_t *)&chan_ctrl_val);
}
/**
 * \Brief Configure the transceiver according to the SFD type and the data rate.
 * We currently not support the usage of user specified SFD.
 */
void
dw_set_datarate_and_sfd(dw1000_data_rate_t data_rate, dw1000_sfd_type_t sfd_type){
  /* Channel Control Register */
  uint8_t chan_ctrl_val = 0;
  /* User-specified short/long TX/RX SFD sequences - SFD Length */
  uint8_t user_sfd_lenght = 0;
  /* Digital Receiver Configuration - Tuning Register 0b */
  uint16_t drx_tune0b_val = 0;

  uint8_t tx_fctrl_val = 0;
  uint8_t sys_cfg_val = 0;

  /* For the Channel Control Register, we focus on the DWSFD (bit 17), the
     TNSSFD (bit 20) and the RNSSFD (bit 21)
  Thus we only touch the 3nd byte */
  dw_read_subreg(DW_REG_CHAN_CTRL, 2, 1, (uint8_t *)&chan_ctrl_val);
  chan_ctrl_val &= ~(DW_DWSFD_MASK  >> 16);
  chan_ctrl_val &= ~(DW_TNSSFD_MASK  >> 16);
  chan_ctrl_val &= ~(DW_RNSSFD_MASK  >> 16);

  switch(sfd_type) {
  case DW_SFD_STANDARD:
    chan_ctrl_val &= ~((1UL << (DW_DWSFD - 16)) & (DW_DWSFD_MASK >> 16));
    break;
  case DW_SFD_NON_STANDARD:
    chan_ctrl_val |= (1UL << (DW_DWSFD - 16)) & (DW_DWSFD_MASK >> 16); /* use DW ns SFD */
    /* The user manual specify that TNSSFD and RNSSFD are ignored when DWSFD 
        is set but the receiver do not detect message without theses tow bits */
    chan_ctrl_val |= (DW_TNSSFD_MASK >> 16);
    chan_ctrl_val |= (DW_RNSSFD_MASK >> 16);
    break;
  case DW_SFD_USER_SPECIFIED:
    /* Not implemented yet! */  
    DW_ERROR("dw_conf - SFD: User specified SFD not implemented");
    break;
  }

  /* We need to specify the length for the non-standard SFD. */
  if(sfd_type == DW_SFD_NON_STANDARD){
    /* This value must be set only for data rate greater 
        or equal than 850 kbps
        Value choose according the "Table 21: Recommended SFD sequence 
        configurations for best performance" of the manual.*/
    switch(data_rate) {
      case DW_DATA_RATE_110_KBPS:
         /* Default value, there are not operational effect.
            The SFD length is always 64 at 110 kbps */
        user_sfd_lenght = 64; 
      break;
      case DW_DATA_RATE_850_KBPS:
        user_sfd_lenght = 16;
      break;
      case DW_DATA_RATE_6800_KBPS:
        user_sfd_lenght = 8;
      break;
    }
  }

  switch(data_rate) {
  case DW_DATA_RATE_110_KBPS:
    if(sfd_type == DW_SFD_STANDARD) {
      drx_tune0b_val = 0x000A;
    } else if(sfd_type == DW_SFD_NON_STANDARD) {
      drx_tune0b_val = 0x0016;
    }
    break;
  case DW_DATA_RATE_850_KBPS:
    if(sfd_type == DW_SFD_STANDARD) {
      drx_tune0b_val = 0x0001;
    } else if(sfd_type == DW_SFD_NON_STANDARD) {
      drx_tune0b_val = 0x0006;
    }
    break;
  case DW_DATA_RATE_6800_KBPS:
    if(sfd_type == DW_SFD_STANDARD) {
      drx_tune0b_val = 0x0001;
    } else if(sfd_type == DW_SFD_NON_STANDARD) {
      drx_tune0b_val = 0x0002;
    }
    break;
  }

  /* For the System Configuration bitmap Register, we focus on the RXM110K 
    (bit 22). Thus we only touch the 3nd byte */
  dw_read_subreg(DW_REG_SYS_CFG, 2, 1, (uint8_t *) &sys_cfg_val);

  /* For the Transmit Frame Control Register, we focus on the TXBR (bit 13-14).
   Thus we only touch the 2nd byte */
  dw_read_subreg(DW_REG_TX_FCTRL, 1, 1, (uint8_t *) &tx_fctrl_val);
  /* suppres previews bit rate */
  tx_fctrl_val &= ~(DW_TXBR_MASK >> 8);

  switch(data_rate) {
  case DW_DATA_RATE_110_KBPS:
    /* Enable Receiver Mode 110 kbps data rate */
    sys_cfg_val |= (1UL << (DW_RXM110K - 16)) & (DW_RXM110K_MASK >> 16);
    /* 110 kbps bit rate */
    tx_fctrl_val |= (0x00UL << (DW_TXBR - 8)) & (DW_TXBR_MASK >> 8);
    break;
  case DW_DATA_RATE_850_KBPS:
    /* Disable Receiver Mode 110 kbps data rate */
    sys_cfg_val &= ~(DW_RXM110K_MASK >> 16); 
    /* 850 kbps bit rate */
    tx_fctrl_val |= (0x01UL << (DW_TXBR - 8)) & (DW_TXBR_MASK >> 8);
    break;
  case DW_DATA_RATE_6800_KBPS:
    /* Disable Receiver Mode 110 kbps data rate */
    sys_cfg_val &= ~(DW_RXM110K_MASK >> 16); 
    /* 6800 kbps bit  rate */
    tx_fctrl_val |= (0x02UL << (DW_TXBR - 8)) & (DW_TXBR_MASK >> 8);
    break;
  }

  dw_write_subreg(DW_REG_CHAN_CTRL, 2, 1, (uint8_t *)&chan_ctrl_val);
  dw_write_subreg(DW_REG_SYS_CFG, 2, 1, (uint8_t *) &sys_cfg_val);
  dw_write_subreg(DW_REG_TX_FCTRL, 1, 1, (uint8_t *) &tx_fctrl_val);

  dw_write_subreg(DW_REG_USR_SFD, DW_SUBREG_SFD_LENGTH, DW_SUBLEN_SFD_LENGTH,
                  (uint8_t *) &user_sfd_lenght);
  dw_write_subreg(DW_REG_DRX_CONF, DW_SUBREG_DRX_TUNE0b, DW_SUBLEN_DRX_TUNE0b,
                  (uint8_t *) &drx_tune0b_val);
}
/**
 * \Brief Configure the preamble acquisition chunk (PAC) according 
 * to the PAC size and the PRF. 
 **/
void 
dw_set_pac_size(dw1000_pac_size_t pac_size, dw1000_prf_t prf)
{
  uint32_t drx_tune2_val = 0;
  switch(pac_size) {
  case DW_PAC_SIZE_8:
    if(prf == DW_PRF_16_MHZ) {
      drx_tune2_val = 0x311A002DUL;
    } else if(prf == DW_PRF_64_MHZ) {
      drx_tune2_val = 0x313B006BUL;
    }
    break;
  case DW_PAC_SIZE_16:
    if(prf == DW_PRF_16_MHZ) {
      drx_tune2_val = 0x331A0052UL;
    } else if(prf == DW_PRF_64_MHZ) {
      drx_tune2_val = 0x333B00BEUL;
    }
    break;
  case DW_PAC_SIZE_32:
    if(prf == DW_PRF_16_MHZ) {
      drx_tune2_val = 0x351A009AUL;
    } else if(prf == DW_PRF_64_MHZ) {
      drx_tune2_val = 0x353B015EUL;
    }
    break;
  case DW_PAC_SIZE_64:
    if(prf == DW_PRF_16_MHZ) {
      drx_tune2_val = 0x371A011DUL;
    } else if(prf == DW_PRF_64_MHZ) {
      drx_tune2_val = 0x373B0296UL;
    }
    break;
  }
  dw_write_subreg(DW_REG_DRX_CONF, DW_SUBREG_DRX_TUNE2, DW_SUBLEN_DRX_TUNE2,
                  (uint8_t *) &drx_tune2_val);
}
/**
 * \Brief Configure the LDE Replica Coefficient.
 */
void
dw_lde_repc_config(dw1000_preamble_code_t preamble_code, 
                    dw1000_data_rate_t data_rate)
{
  uint16_t lde_repc = 0;
  /* The following value has gives in the user-manual v2.10, table:
  "Table 49: Sub-Register 0x2E:2804 – LDE_REPC configurations for 
      (850 kbps & 6.8 Mbps)"*/
  switch(preamble_code) {
    case DW_PREAMBLE_CODE_1:
      lde_repc = ((uint16_t)DW_LDE_REPC_1);
      break;
    case DW_PREAMBLE_CODE_2:
      lde_repc = ((uint16_t)DW_LDE_REPC_2);
      break;
    case DW_PREAMBLE_CODE_3:
      lde_repc = ((uint16_t)DW_LDE_REPC_3);
      break;
    case DW_PREAMBLE_CODE_4:
      lde_repc = ((uint16_t)DW_LDE_REPC_4);
      break;
    case DW_PREAMBLE_CODE_5:
      lde_repc = ((uint16_t)DW_LDE_REPC_5);
      break;
    case DW_PREAMBLE_CODE_6:
      lde_repc = ((uint16_t)DW_LDE_REPC_6);
      break;
    case DW_PREAMBLE_CODE_7:
      lde_repc = ((uint16_t)DW_LDE_REPC_7);
      break;
    case DW_PREAMBLE_CODE_8:
      lde_repc = ((uint16_t)DW_LDE_REPC_8);
      break;
    case DW_PREAMBLE_CODE_9:
      lde_repc = ((uint16_t)DW_LDE_REPC_9);
      break;
    case DW_PREAMBLE_CODE_10:
      lde_repc = ((uint16_t)DW_LDE_REPC_10);
      break;
    case DW_PREAMBLE_CODE_11:
      lde_repc = ((uint16_t)DW_LDE_REPC_11);
      break;
    case DW_PREAMBLE_CODE_12:
      lde_repc = ((uint16_t)DW_LDE_REPC_12);
      break;
    case DW_PREAMBLE_CODE_13: /* DPS Preamble code */
      lde_repc = ((uint16_t)DW_LDE_REPC_13);
      break;
    case DW_PREAMBLE_CODE_14: /* DPS Preamble code */
      lde_repc = ((uint16_t)DW_LDE_REPC_14);
      break;
    case DW_PREAMBLE_CODE_15: /* DPS Preamble code */
      lde_repc = ((uint16_t)DW_LDE_REPC_15);
      break;
    case DW_PREAMBLE_CODE_16: /* DPS Preamble code */
      lde_repc = ((uint16_t)DW_LDE_REPC_16);
      break;
    case DW_PREAMBLE_CODE_17:
      lde_repc = ((uint16_t)DW_LDE_REPC_17);
      break;
    case DW_PREAMBLE_CODE_18:
      lde_repc = ((uint16_t)DW_LDE_REPC_18);
      break;
    case DW_PREAMBLE_CODE_19:
      lde_repc = ((uint16_t)DW_LDE_REPC_19);
      break;
    case DW_PREAMBLE_CODE_20:
      lde_repc = ((uint16_t)DW_LDE_REPC_20);
      break;
    case DW_PREAMBLE_CODE_21: /* DPS Preamble code */
      lde_repc = ((uint16_t)DW_LDE_REPC_21);
      break;
    case DW_PREAMBLE_CODE_22: /* DPS Preamble code */
      lde_repc = ((uint16_t)DW_LDE_REPC_22);
      break;
    case DW_PREAMBLE_CODE_23: /* DPS Preamble code */
      lde_repc = ((uint16_t)DW_LDE_REPC_23);
      break;
    case DW_PREAMBLE_CODE_24: /* DPS Preamble code */
      lde_repc = ((uint16_t)DW_LDE_REPC_24);
      break;
  }
  /* From the user manual v2.10, p170:
  NB: When operating at 110 kbps the unsigned values in Table 49 have to be 
  divided by 8, (right shifted 3, shifting zeroes into the high order bits), 
  before programming into Sub-Register 0x2E:2804 – LDE_REPC. */
  if(data_rate == DW_DATA_RATE_110_KBPS) {
    lde_repc >>= 3; /* see page 170. */
    lde_repc &= 0x1FFF;
  }
  dw_write_subreg(DW_REG_LDE_IF, DW_SUBREG_LDE_REPC, DW_SUBLEN_LDE_REPC,
                  (uint8_t *) &lde_repc);
}
/**
 * \brief Configure the LDE algorithm parameter for better performance and 
 *      compatibility with the DW1000 configuration.
 */
void
dw_configure_lde(dw1000_prf_t prf)
{  
  uint32_t lde_cfg1 = 0UL;
  uint32_t lde_cfg2 = 0UL;

  /* Configure LDE for better performance
    Following the user manual v2.10 section "2.5.5.4 NTM" */
  lde_cfg1 &= ~DW_NTM_MASK;
  lde_cfg1 |= (0xD << DW_NTM) & DW_NTM_MASK; 
  lde_cfg1 &= ~DW_PMULT_MASK;
  lde_cfg1 |= (0x3 << DW_PMULT) & DW_PMULT_MASK;

  /* the following value has gives in the user manual v2.10 table
  "Table 48: Sub-Register 0x2E:1806– LDE_CFG2 values" */
  switch(prf) {
    case DW_PRF_16_MHZ:
      lde_cfg2 = 0x1607;
      break;

    case DW_PRF_64_MHZ:
      lde_cfg2 = 0x0607;
      break;
  }
  dw_write_subreg(DW_REG_LDE_IF, DW_SUBREG_LDE_CFG1, DW_SUBLEN_LDE_CFG1,
                  (uint8_t *) &lde_cfg1);
  dw_write_subreg(DW_REG_LDE_IF, DW_SUBREG_LDE_CFG2, DW_SUBLEN_LDE_CFG2,
                  (uint8_t *) &lde_cfg2);
}
/**
 * \brief return if the Internal Low Drop Out (LDO) Regulators voltage has 
 been configured by DecaWave during the DecaWave Test */
int
dw_is_ldotune(void){
  /* The 40 bit LDOTUNE_CAL start at 0x04, if the first byte is != 0 
  then LDOTUNE is calibrated */
  // printf("LDOTUNE CAL value %08lX\n", dw_read_otp_32(0X04));
  return dw_read_otp_32(0X04) > 0;
}
/**
 * \brief Initialize the loading of the LDOTUNE_CAL value 
 parameter from OTP to LDOTUNE register if it is calibrated.
 */
void
dw_load_ldotune(void){
  uint8_t opt_sf = DW_LDO_KICK;
  if(dw_is_ldotune()){
    dw_write_subreg(DW_REG_OTP_IF, DW_SUBREG_OTP_SF, DW_SUBLEN_OTP_SF,
                    &opt_sf);
  }
}
/**
 * Put the transceiver in DEEP SLEEP state.
 * /!\ Warning the transceiver need 3 ms to wake up.
 * Need to use the WAKEUP pin to wakeup the transceiver.
 */
void
set_in_deep_sleep(void){
  /* Configure the Always ON system control */
  uint16_t aon_wcfg = 0; /* AON Wake-up Configuration */
  uint8_t aon_ctrl = 0; /* AON Control */
  uint8_t aon_cfg0 = 0; /* AON Configuration Register 0 */
  uint8_t aon_cfg1 = 0; /* AON Configuration Register 1 */

  /* According to DECADRIVER "The 3 bits in AON CFG1 register must be 
  cleared to ensure proper operation of the DW1000 in DEEPSLEEP mode. */
  dw_write_subreg(DW_REG_AON, DW_SUBREG_AON_CFG1, 1,
                  &aon_cfg1);

  /* Reset AON Control value*/
  dw_write_subreg(DW_REG_AON, DW_SUBREG_AON_CTRL, DW_SUBLEN_AON_CTRL,
                  &aon_ctrl);

  /* On wake-up run the temperature and voltage ADC */
  aon_wcfg |= DW_ONW_RADC_MASK; 
  /* On wake-up upload the configuration from the AON memory */
  aon_wcfg |= DW_ONW_LDC_MASK;
  /* On wake-up load the LDE code 
  (useful for correct timestamps and RSSI value) */
  aon_wcfg |= DW_ONW_LLDE_MASK;

  /* On wake-up load the LDO tune value */
  if(dw_is_ldotune()){
    aon_wcfg |= DW_ONW_LLDO_MASK;
  }

  /* Sleep enable configuration bit. In order to put the DW1000 into the 
  SLEEP state this bit needs to be set and then the configuration needs 
  to be uploaded to the AON using the UPL_CFG bit in AON_CTRL */
  aon_cfg0 |= DW_SLEEP_EN_MASK;

  /* Enable Wake up using SPI CSn or WAKE UP pin. */
  aon_cfg0 |= DW_WAKE_PIN_MASK;
  // aon_cfg0 |= DW_WAKE_SPI_MASK;

  /* Enable interrupt flag for the clock PLL lock event and 
  SLEEP to INIT event */
  dw_clear_pending_interrupt(DW_MCPLOCK_MASK|DW_MSLP2INIT_MASK);
  dw_enable_interrupt(DW_MCPLOCK_MASK|DW_MSLP2INIT_MASK);

  dw_write_subreg(DW_REG_AON, DW_SUBREG_AON_WCFG, DW_SUBLEN_AON_WCFG,
                  (uint8_t *) &aon_wcfg);

  dw_write_subreg(DW_REG_AON, DW_SUBREG_AON_CFG0, 1,
                   &aon_cfg0);


  /* Upload the AON block configurations to the AON and then enter in sleep
  because SLEEP_EN is set.*/
  aon_ctrl = DW_SAVE_MASK;
  dw_write_subreg(DW_REG_AON, DW_SUBREG_AON_CTRL, DW_SUBLEN_AON_CTRL,
                  &aon_ctrl);
}
/**
 * \brief Set the TX power according the Table 20: "Reference values Register 
 *    file: 0x1E – Transmit Power Control for Manual Transmit Power Control" 
 *    of the user manual (v2.10).
 *    We disable the Smart Transmit Power Control.
 *
 * \param[in] channel   The channel.
 * \param[in] prf       The PRF.
 */
void dw_set_manual_tx_power(dw1000_channel_t channel, dw1000_prf_t prf){
  uint32_t tx_power_val = 0UL;  
  uint32_t sys_cfg_val = dw_read_reg_32(DW_REG_SYS_CFG, DW_LEN_SYS_CFG);
  /* Configure the TX power based on the channel and the PRF
    Based on the manual: Table 20: Reference values Register file: 
    0x1E – Transmit Power Control for Manual Transmit Power
    Control (Smart Transmit Power Control disabled) */
  sys_cfg_val |= DW_DIS_STXP_MASK;  /* Disable Smart Transmit Power Control */

  switch(channel) {
  case DW_CHANNEL_1:
    if(prf == DW_PRF_16_MHZ) {
      tx_power_val = 0x75757575ul;
    } else if(prf == DW_PRF_64_MHZ) {
      tx_power_val = 0x67676767ul;
    }
    break;
  case DW_CHANNEL_2:
    if(prf == DW_PRF_16_MHZ) {
      tx_power_val = 0x75757575ul;
    } else if(prf == DW_PRF_64_MHZ) {
      tx_power_val = 0x67676767ul;
    }
    break;
  case DW_CHANNEL_3:
    if(prf == DW_PRF_16_MHZ) {
      tx_power_val = 0x6F6F6F6Ful;
    } else if(prf == DW_PRF_64_MHZ) {
      tx_power_val = 0x8B8B8B8Bul;
    }
    break;
  case DW_CHANNEL_4:
    if(prf == DW_PRF_16_MHZ) {
      tx_power_val = 0x5F5F5F5Ful;
    } else if(prf == DW_PRF_64_MHZ) {
      tx_power_val = 0x9A9A9A9Aul;
    }
    break;
  case DW_CHANNEL_5:
    if(prf == DW_PRF_16_MHZ) {
      tx_power_val = 0x48484848ul;
    } else if(prf == DW_PRF_64_MHZ) {
      tx_power_val = 0x85858585ul;
    }
    break;
  case DW_CHANNEL_7:
    if(prf == DW_PRF_16_MHZ) {
      tx_power_val = 0x92929292ul;
    } else if(prf == DW_PRF_64_MHZ) {
      tx_power_val = 0xD1D1D1D1ul;
    }
    break;
  }

  dw_write_reg(DW_REG_SYS_CFG, DW_LEN_SYS_CFG, (uint8_t *) &sys_cfg_val);
  dw_write_reg(DW_REG_TX_POWER, DW_LEN_TX_POWER, (uint8_t *) &tx_power_val);
}
/**
 * \brief Set the TX power according the Table 19: "Reference values for 
 *    Register file: 0x1E – Transmit Power Control, for Smart 
 *    Transmit Power Control" of the user manual (v2.18).
 *    We disable the Smart Transmit Power Control.
 *
 * \param[in] channel   The channel.
 * \param[in] prf       The PRF.
 */
void dw_set_smart_tx_power(dw1000_channel_t channel, dw1000_prf_t prf){
  uint32_t tx_power_val = 0UL;  
  uint32_t sys_cfg_val = dw_read_reg_32(DW_REG_SYS_CFG, DW_LEN_SYS_CFG);
  /* Configure the TX power based on the channel and the PRF
    Based on the manual: Table 19: Reference values for Register file: 0x1E – Transmit Power Control, for Smart Transmit Power Control */
  sys_cfg_val &= ~DW_DIS_STXP_MASK;  /* Enable Smart Transmit Power Control */

  switch(channel) {
  case DW_CHANNEL_1:
    if(prf == DW_PRF_16_MHZ) {
      tx_power_val = 0x15355575ul;
    } else if(prf == DW_PRF_64_MHZ) {
      tx_power_val = 0x07274767ul;
    }
    break;
  case DW_CHANNEL_2:
    if(prf == DW_PRF_16_MHZ) {
      tx_power_val = 0x15355575ul;
    } else if(prf == DW_PRF_64_MHZ) {
      tx_power_val = 0x07274767ul;
    }
    break;
  case DW_CHANNEL_3:
    if(prf == DW_PRF_16_MHZ) {
      tx_power_val = 0x0F2F4F6Ful;
    } else if(prf == DW_PRF_64_MHZ) {
      tx_power_val = 0x2B4B6B8Bul;
    }
    break;
  case DW_CHANNEL_4:
    if(prf == DW_PRF_16_MHZ) {
      tx_power_val = 0x1F1F3F5Ful;
    } else if(prf == DW_PRF_64_MHZ) {
      tx_power_val = 0x3A5A7A9Aul;
    }
    break;
  case DW_CHANNEL_5:
    if(prf == DW_PRF_16_MHZ) {
      tx_power_val = 0x0E082848ul;
    } else if(prf == DW_PRF_64_MHZ) {
      tx_power_val = 0x25466788ul;
    }
    break;
  case DW_CHANNEL_7:
    if(prf == DW_PRF_16_MHZ) {
      tx_power_val = 0x32527292ul;
    } else if(prf == DW_PRF_64_MHZ) {
      tx_power_val = 0x5171B1D1ul;
    }
    break;
  }

  dw_write_reg(DW_REG_SYS_CFG, DW_LEN_SYS_CFG, (uint8_t *) &sys_cfg_val);
  dw_write_reg(DW_REG_TX_POWER, DW_LEN_TX_POWER, (uint8_t *) &tx_power_val);
}
/**
 * \brief Change the TX Power value.
 *      TX_POWER – Transmit Power Control
 * 
 * \param[in] tx_power_val   The new TX POWER configuration value.
 *                           This value should follow the characteristic gives
 *                            in the manual.
 * \param[in] manual         True (1) if you want to use the manual mode.
 *                           False(0) if you want use the Smart Transmit
 *                                 Power Control.
 *  /!\ Before use this mode you should disable the antenna (TRX off).
 */
void
dw_change_tx_power(uint32_t tx_power_val, uint8_t manual){
  uint32_t sys_cfg_val = dw_read_reg_32(DW_REG_SYS_CFG, DW_LEN_SYS_CFG);
  if(manual){
      sys_cfg_val |= DW_DIS_STXP_MASK;  /* Disable Smart Transmit Power Control */
  }
  else{
    sys_cfg_val &= ~DW_DIS_STXP_MASK;  /* Enable Smart Transmit Power Control */
  }
  dw_write_reg(DW_REG_SYS_CFG, DW_LEN_SYS_CFG, (uint8_t *) &sys_cfg_val);
  dw_write_reg(DW_REG_TX_POWER, DW_LEN_TX_POWER, (uint8_t *) &tx_power_val);
  dw_sfd_init();
}

/**
 * \brief return the TX POWEr value.
 *    TX_POWER – Transmit Power Control
 */
uint32_t
dw_get_tx_power(void){
  uint32_t tx_power_val = 0UL;
  dw_read_reg(DW_REG_TX_POWER, DW_LEN_TX_POWER, (uint8_t *) &tx_power_val);
  return tx_power_val;
}
/**
 * \brief Configures the DW1000 to be ready to receive message. 
 *        See \ref dw1000_tx_conf_t.
 * \param[in] rx_conf   Configuration specification.
 */
void
dw_conf_rx(dw1000_rx_conf_t *rx_conf)
{
  /* Timeout */
  dw_set_rx_timeout(rx_conf->timeout);
  if(rx_conf->timeout) {
    dw_enable_rx_timeout();
  } else {
    dw_disable_rx_timeout();
  }

  /* Delayed reception */
  if(rx_conf->is_delayed) {
    dw_set_dx_timestamp(rx_conf->dx_timestamp);

    uint32_t sys_ctrl_val = 0UL;
    sys_ctrl_val = dw_read_reg_32(DW_REG_SYS_CTRL, DW_LEN_SYS_CTRL);
    sys_ctrl_val |= DW_RXDLYE_MASK;
    dw_write_reg(DW_REG_SYS_CTRL, DW_LEN_SYS_CTRL, (uint8_t *)&sys_ctrl_val);
  }

/*  enableFiltering(); */
}
/**
 * \brief Configures the DW1000 to be ready to transmit message. 
 *        See \ref dw1000_tx_conf_t.
 *
 * \param[in] tx_conf   Configuration specification.
 */
void
dw_conf_tx(dw1000_tx_conf_t *tx_conf)
{
  /* TODO: Handling of long data frames (length > 128 or whatever.) */
  /* TODO: Cache data..? */
  /* TODO: Should check dw1000 configuration for FCS enable and add the 2 
   *       conditionally. */
  uint32_t data_len = tx_conf->data_len;
  data_len += 2; /* The +2 is for FCS */
  dw_set_tx_frame_length(data_len);

  /* Delayed transmission */
  if(tx_conf->is_delayed) {
    dw_enable_delayed_tx(tx_conf->dx_timestamp);
  } else {
    dw_disable_delayed_tx_rx();
  }
}
/**
 * \brief Set the Transmit Frame Length.
 * Standard IEEE 802.15.4 UWB frames can be up to 127 bytes long.
 * The value specified here determines the length of the data portion
 * of the transmitted frame. This length includes the two-octet CRC
 * appended automatically (if this not disable for this send)
 * at the end of the frame.
 * If the length is bigger than 127 the transceiver use a Extended 
 * (no standard) IEEE 802.15.4 UWB frames (it can be up to 1023 bytes long).
 *
 * \param frame_len the Transmit Frame Length.
 */
void
dw_set_tx_frame_length(uint16_t frame_len)
{
  uint16_t tx_frame_control_lo = 0;
  dw_read_subreg(DW_REG_TX_FCTRL, 0x0, 2, (uint8_t*) &tx_frame_control_lo);

  /* reseting the length */
  tx_frame_control_lo &= ~(DW_TFLEN_MASK | DW_TFLE_MASK); 

  tx_frame_control_lo |= (frame_len << DW_TFLEN) 
                            & (DW_TFLEN_MASK | DW_TFLE_MASK);
  dw_write_subreg(DW_REG_TX_FCTRL, 0x0, 2, (uint8_t *) &tx_frame_control_lo);
}
/**
 * \brief Enable delayed transmission.
 * \param dx_timestamp Value to be programmed into DW1000 dx_timestamp 
 *                     register.
 */
void
dw_enable_delayed_tx(uint64_t dx_timestamp)
{
  dw_set_dx_timestamp(dx_timestamp);

  uint32_t ctrl_reg_val = dw_read_reg_32(DW_REG_SYS_CTRL, DW_LEN_SYS_CTRL);
  ctrl_reg_val |= DW_TXDLYS_MASK;   /* sender */
  dw_write_reg(DW_REG_SYS_CTRL, DW_LEN_SYS_CTRL, (uint8_t *) &ctrl_reg_val);
}
/**
 * \brief Enable delayed reception.
 * \param dx_timestamp Value to be programmed into DW1000 dx_timestamp 
 *                     register.
 */
void
dw_enable_delayed_rx(uint64_t dx_timestamp)
{
  dw_set_dx_timestamp(dx_timestamp);

  uint32_t ctrl_reg_val = dw_read_reg_32(DW_REG_SYS_CTRL, DW_LEN_SYS_CTRL);
  ctrl_reg_val |= DW_RXDLYE_MASK;   /* receiver */
  dw_write_reg(DW_REG_SYS_CTRL, DW_LEN_SYS_CTRL, (uint8_t *)&ctrl_reg_val);
}
/**
 * \brief Disable delayed transmission and reception.
 */
void
dw_disable_delayed_tx_rx(void)
{
  uint32_t ctrl_reg_val = 0UL;
  ctrl_reg_val = dw_read_reg_32(DW_REG_SYS_CTRL, DW_LEN_SYS_CTRL);
  ctrl_reg_val &= ~DW_TXDLYS_MASK;   /* sender */
  ctrl_reg_val &= ~DW_RXDLYE_MASK;   /* receiver */
  /* PRINTF("Systeme control %04X\r\n", (unsigned int)  ctrl_reg_val); */
  dw_write_reg(DW_REG_SYS_CTRL, DW_LEN_SYS_CTRL, (uint8_t *)&ctrl_reg_val);
}
/**
 * \brief Reads the current configuration from device and prints it using
 *  PRINTF. The current configuration is all registers that can be modified by
 *  \ref dw_conf, \ref dw_conf_rx and \ref dw_conf_tx.
 */
void
dw_conf_print()
{
  uint32_t sys_cfg_val = 0UL;
  uint64_t tx_fctrl_val = 0ULL;
  uint32_t chan_ctrl_val = 0UL;
  uint32_t agc_tune1_val = 0UL;
  uint32_t agc_tune2_val = 0UL;
  uint32_t agc_tune3_val = 0UL;
  uint32_t drx_tune0b_val = 0UL;
  uint32_t drx_tune1a_val = 0UL;
  uint32_t drx_tune1b_val = 0UL;
  uint32_t drx_tune2_val = 0UL;
  uint32_t drx_tune4h_val = 0UL;
  uint32_t rf_rxctrlh_val = 0UL;
  uint32_t rf_txctrl_val = 0UL;
  uint32_t tc_pgdelay_val = 0UL;
  uint32_t fs_pllcfg_val = 0UL;
  uint32_t fs_plltune_val = 0UL;
  uint32_t ec_ctrl_val = 0UL;
  uint32_t fs_xtalt_val = 0UL;
  uint32_t lde_cfg1 = 0UL;
  uint32_t lde_cfg2 = 0UL;
  uint32_t lde_repc = 0UL;
  uint32_t tx_power_val = 0UL;
  uint32_t rx_finfo_val = 0UL;
  uint32_t gpio_mode = 0UL;
  uint32_t gpio_dir = 0UL;
  uint32_t gpio_dout = 0UL;
  uint8_t  user_sfd_lenght = 0;
  float temperature_val = 0;
  float voltage_val = 0;

  sys_cfg_val = dw_read_reg_32(DW_REG_SYS_CFG, DW_LEN_SYS_CFG);
  dw_read_reg(DW_REG_TX_FCTRL, DW_LEN_TX_FCTRL, (uint8_t*) &tx_fctrl_val);
  chan_ctrl_val = dw_read_reg_32(DW_REG_CHAN_CTRL, DW_LEN_CHAN_CTRL);
  agc_tune1_val = dw_read_subreg_32(DW_REG_AGC_CTRL, DW_SUBREG_AGC_TUNE1,
                                    DW_SUBLEN_AGC_TUNE1);
  agc_tune2_val = dw_read_subreg_32(DW_REG_AGC_CTRL, DW_SUBREG_AGC_TUNE2,
                                    DW_SUBLEN_AGC_TUNE2);
  agc_tune3_val = dw_read_subreg_32(DW_REG_AGC_CTRL, DW_SUBREG_AGC_TUNE3,
                                    DW_SUBLEN_AGC_TUNE3);
  drx_tune0b_val = dw_read_subreg_32(DW_REG_DRX_CONF, DW_SUBREG_DRX_TUNE0b,
                                    DW_SUBLEN_DRX_TUNE0b);
  drx_tune1a_val = dw_read_subreg_32(DW_REG_DRX_CONF, DW_SUBREG_DRX_TUNE1a,
                                    DW_SUBLEN_DRX_TUNE1a);
  drx_tune1b_val = dw_read_subreg_32(DW_REG_DRX_CONF, DW_SUBREG_DRX_TUNE1b,
                                    DW_SUBLEN_DRX_TUNE1b);
  drx_tune2_val = dw_read_subreg_32(DW_REG_DRX_CONF, DW_SUBREG_DRX_TUNE2,
                                    DW_SUBLEN_DRX_TUNE2);
  drx_tune4h_val = dw_read_subreg_32(DW_REG_DRX_CONF, DW_SUBREG_DRX_TUNE4h,
                                    DW_SUBLEN_DRX_TUNE4h);
  rf_rxctrlh_val = dw_read_subreg_32(DW_REG_RF_CONF, DW_SUBREG_RF_RXCTRLH,
                                    DW_SUBLEN_RF_RXCTRLH);
  rf_txctrl_val = dw_read_subreg_32(DW_REG_RF_CONF, DW_SUBREG_RF_TXCTRL,
                                    DW_SUBLEN_RF_TXCTRL);
  tc_pgdelay_val = dw_read_subreg_32(DW_REG_TX_CAL, DW_SUBREG_TC_PGDELAY,
                                    DW_SUBLEN_TC_PGDELAY);
  fs_pllcfg_val = dw_read_subreg_32(DW_REG_FS_CTRL, DW_SUBREG_FS_PLLCFG,
                                    DW_SUBLEN_FS_PLLCFG);
  fs_plltune_val = dw_read_subreg_32(DW_REG_FS_CTRL, DW_SUBREG_FS_PLLTUNE,
                                    DW_SUBLEN_FS_PLLTUNE);
  dw_read_reg(DW_REG_EC_CTRL, DW_LEN_EC_CTRL, (uint8_t *) &ec_ctrl_val);
  dw_read_subreg(DW_REG_FS_CTRL, DW_SUBREG_FS_XTALT, DW_SUBLEN_FS_XTALT,
                  (uint8_t *) &fs_xtalt_val);
  dw_read_subreg(DW_REG_LDE_IF, DW_SUBREG_LDE_CFG1, DW_SUBLEN_LDE_CFG1,
                  (uint8_t *) &lde_cfg1);
  dw_read_subreg(DW_REG_LDE_IF, DW_SUBREG_LDE_CFG2, DW_SUBLEN_LDE_CFG2,
                  (uint8_t *) &lde_cfg2);
  dw_read_subreg(DW_REG_LDE_IF, DW_SUBREG_LDE_REPC, DW_SUBLEN_LDE_REPC,
                  (uint8_t *) &lde_repc);
  dw_read_reg(DW_REG_TX_POWER, DW_LEN_TX_POWER, (uint8_t *) &tx_power_val);
  dw_read_reg(DW_REG_RX_FINFO, DW_LEN_RX_FINFO, (uint8_t *) &rx_finfo_val);
  dw_read_subreg(DW_REG_USR_SFD, DW_SUBREG_SFD_LENGTH, DW_SUBLEN_SFD_LENGTH,
                  (uint8_t *) &user_sfd_lenght);

  gpio_mode = dw_read_subreg_32(DW_REG_GPIO_CTRL, DW_SUBREG_GPIO_MODE,
                                    DW_SUBLEN_GPIO_MODE);
  gpio_dir = dw_read_subreg_32(DW_REG_GPIO_CTRL, DW_SUBREG_GPIO_DIR,
                                    DW_SUBLEN_GPIO_DIR);
  gpio_dout = dw_read_subreg_32(DW_REG_GPIO_CTRL, DW_SUBREG_GPIO_DOUT,
                                    DW_SUBLEN_GPIO_DOUT);
  temperature_val = dw_get_temperature(DW_ADC_SRC_LATEST);
  voltage_val = dw_get_voltage(DW_ADC_SRC_LATEST);

  printf("============================\r\n");
  printf("DW1000 Current Configuration\r\n");
  printf("============================\r\n");
  printf("Device id   : 0x%08" PRIx32 "\r\n", dw_get_device_id());
  printf("SYS_STATUS  : 0x%016" PRIx64 "\r\n", (unsigned long long) 
                        dw_read_reg_64(DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS));
  printf("SYS_MASK    : 0x%016" PRIx64 "\r\n", (unsigned long long) 
                        dw_read_reg_64(DW_REG_SYS_MASK, DW_LEN_SYS_MASK));
  printf("SYS_STATE   : 0x%016" PRIx64 "\r\n", (unsigned long long) 
                        dw_read_reg_64(DW_REG_SYS_STATE, DW_LEN_SYS_STATE));
  printf("============================\r\n");
  printf("sys_cfg    : 0x%08" PRIx32 "\r\n", sys_cfg_val);
  printf("tx_fctrl   : 0x%016" PRIx64 "\r\n", tx_fctrl_val);
  printf("chan_ctrl  : 0x%08" PRIx32 "\r\n", chan_ctrl_val);
  printf("agc_tune1  : 0x%08" PRIx32 "\r\n", agc_tune1_val);
  printf("agc_tune2  : 0x%08" PRIx32 "\r\n", agc_tune2_val);
  printf("agc_tune3  : 0x%08" PRIx32 "\r\n", agc_tune3_val);
  printf("drx_tune0b : 0x%08" PRIx32 "\r\n", drx_tune0b_val);
  printf("drx_tune1a : 0x%08" PRIx32 "\r\n", drx_tune1a_val);
  printf("drx_tune1b : 0x%08" PRIx32 "\r\n", drx_tune1b_val);
  printf("drx_tune2  : 0x%08" PRIx32 "\r\n", drx_tune2_val);
  printf("drx_tune4h : 0x%08" PRIx32 "\r\n", drx_tune4h_val);
  printf("rf_rxctrlh : 0x%08" PRIx32 "\r\n", rf_rxctrlh_val);
  printf("rf_txctrl  : 0x%08" PRIx32 "\r\n", rf_txctrl_val);
  printf("tc_pgdelay : 0x%08" PRIx32 "\r\n", tc_pgdelay_val);
  printf("fs_pllcfg  : 0x%08" PRIx32 "\r\n", fs_pllcfg_val);
  printf("fs_plltune : 0x%08" PRIx32 "\r\n", fs_plltune_val);
  printf("ec_ctrl    : 0x%08" PRIx32 "\r\n", ec_ctrl_val);
  printf("fs_xtalt   : 0x%08" PRIx32 "\r\n", fs_xtalt_val);
  printf("lde_cfg1   : 0x%08" PRIx32 "\r\n", lde_cfg1);
  printf("lde_cfg2   : 0x%08" PRIx32 "\r\n", lde_cfg2);
  printf("lde_repc   : 0x%08" PRIx32 "\r\n", lde_repc);
  printf("tx_power   : 0x%08" PRIx32 "\r\n", tx_power_val);
  printf("rx_finfo   : 0x%08" PRIx32 "\r\n", rx_finfo_val);
  printf("user_sfd_lenght   : 0x%08X\r\n", user_sfd_lenght);
  printf("Temperature       : 0x%08" PRIx32 " (float)\n", *(long unsigned int*)
                                    &temperature_val);
  printf("Voltage           : 0x%08" PRIx32 " (float)\n",*(long unsigned int*)
                                      &voltage_val);
  printf("gpio_mode   : 0x%08" PRIx32 "\r\n", gpio_mode);
  printf("gpio_dir   : 0x%08" PRIx32 "\r\n", gpio_dir);
  printf("gpio_dout   : 0x%08" PRIx32 "\r\n", gpio_dout);
}
/*===========================================================================*/
/* Utility                                                                   */
/*===========================================================================*/
/**
 * \brief Generates a sequence number for use with a new transmission.
 * \return A new sequence number (unique mod 256).
 */
uint8_t
dw_get_seq_no()
{
  static uint8_t seq_no = 0;
  return seq_no++;
}
/**
 * \brief Converts from floating point milliseconds to device time ticks.
 * \param[in]  t    Time in milliseconds (ms).
 * \return Time in device clock ticks (~15.65 ps per tick).
 */
uint64_t
dw_ms_to_device_time(float t)
{
  return (uint64_t)(t * DW_MS_TO_DEVICE_TIME_SCALE);
}
/**
 * \brief Get the component unique id.
 * \return Component unique id.
 */
uint32_t
dw_get_device_id(void)
{
  static uint64_t device_id = 0x0ULL;
  if(device_id == 0x0ULL) {
    device_id = dw_read_reg_64(DW_REG_DEV_ID, DW_LEN_DEV_ID);
  }
  return (uint32_t)device_id;
}
uint8_t euid_set = 0;

void
print_u8_Array_inHex(char *string, uint8_t *array, uint32_t arrayLength)
{
  uint32_t i = 0UL;
  printf("%s 0x", string);
  for(i = 0; i < arrayLength; i++) {
    printf("%02" PRIx8, array[i]);
  }
  printf("\r\n");
}
/**
 * \brief Get the component PANID.
 * \return the component PANID.
 */
uint16_t
dw_get_pan_id()
{
  uint16_t panIdShortAddress = 0;
  dw_read_subreg(DW_REG_PANADR, 0x02, 2, (uint8_t *)&panIdShortAddress);
  return panIdShortAddress;
}
/**
 * \brief Set the component PANID.
 * \param[in] pan_id the component PANID.
 */
void
dw_set_pan_id(uint16_t pan_id)
{
  dw_write_subreg(DW_REG_PANADR, 0x02, 2, (uint8_t *)&pan_id);
}
/**
 * \brief Get the component short address.
 * \return the component short address.
 */
uint16_t
dw_get_short_addr()
{
  uint16_t panIdShortAddress = 0;
  dw_read_reg(DW_REG_PANADR, 2, (uint8_t *)&panIdShortAddress);
  return panIdShortAddress;
}
/**
 * \brief Set the component short address.
 * \param[in] short_addr the component short address.
 */
void
dw_set_short_addr(uint16_t short_addr)
{
  dw_write_reg(DW_REG_PANADR, 2, (uint8_t *)&short_addr);
}
/**
 * \brief Get the extended address (64 bits addr) also call extended unique ID.
 * \return the  extended addr.
 */
uint64_t
dw_get_extended_addr(void)
{
  uint64_t ext_addr = 0ULL;
  dw_read_reg(DW_REG_EID, DW_LEN_EID, (uint8_t *)&ext_addr);
  return ext_addr;
}
/**
 * \brief Set the component extended unique ID.
 * \param[in] euid the component extended unique ID.
 */
void
dw_set_extended_addr(uint64_t ext_addr)
{
  dw_write_reg(DW_REG_EID, DW_LEN_EID, (uint8_t *) &ext_addr);
}
/**
 * \brief Returns the current system clock of the DW1000.
 * \return Current system clock time.
 */
uint64_t
dw_get_device_time()
{
  return dw_read_reg_64(DW_REG_SYS_TIME, DW_LEN_SYS_TIME);
}
/*===========================================================================*/
/* ADC                                                                       */
/*===========================================================================*/

/**
 * \brief Enables power to the ADC circuitry.
 */
void
dw_enable_adc()
{
  uint32_t pmsc_val = dw_read_subreg_32(DW_REG_PMSC, DW_SUBREG_PMSC_CTRL0, 
                      DW_SUBLEN_PMSC_CTRL0);
  pmsc_val |= DW_ADCCE_MASK;
  dw_write_subreg(DW_REG_PMSC, DW_SUBREG_PMSC_CTRL0, DW_SUBLEN_PMSC_CTRL0, 
                  (uint8_t *)&pmsc_val);
}
/**
 * \brief Disables power to the ADC circuitry.
 */
void
dw_disable_adc()
{
  uint32_t pmsc_val = dw_read_subreg_32(DW_REG_PMSC, DW_SUBREG_PMSC_CTRL0, 
                      DW_SUBLEN_PMSC_CTRL0);
  pmsc_val &= ~DW_ADCCE_MASK;
  dw_write_subreg(DW_REG_PMSC, DW_SUBREG_PMSC_CTRL0, DW_SUBLEN_PMSC_CTRL0, 
                  (uint8_t *)&pmsc_val);
}/**
 * \brief Private function. Forces the ADC to update sensor samples.
 *
 * See DW1000-User_Manual-V2.10.pdf page 56 - "Measuring IC
 * temperature and voltage" for details on how sampling is performed.
 */
void dw_adc_sample()
{
  /*  Make sure ADC clock is enabled */
  dw_enable_adc();

  /* These writes should be single writes and in sequence */
  uint8_t val;
  val = 0x80; /* Enable TLD Bias */
  dw_write_subreg(0x28, 0x11, 1, &val);
  val = 0x0A; /* Enable TLD Bias and ADC Bias */
  dw_write_subreg(0x28, 0x12, 1, &val);
  val = 0x0F; /* Enable Outputs (only after Biases are up and running) */
  dw_write_subreg(0x28, 0x12, 1, &val);

  /* Take sample. */
  uint8_t tc_sarc_val = 0;
  // dw_write_subreg(DW_REG_TX_CAL, DW_SUBREG_TC_SARC, 1, &tc_sarc_val);
  tc_sarc_val = DW_SAR_CTRL_MASK;
  dw_write_subreg(DW_REG_TX_CAL, DW_SUBREG_TC_SARC, 1, &tc_sarc_val);

  /* The enable should set for a minimum of 2.5 μs to 
    allow the SAR time to complete its reading. */
  dw1000_us_delay(3);

  /* Disable sampling */
  tc_sarc_val = 0;
  dw_write_subreg(DW_REG_TX_CAL, DW_SUBREG_TC_SARC, 1, &tc_sarc_val);

  return;
}
/**
 * \brief Gets a temperature reading from the DW1000.
 *
 * \param[in] temp_source     If given as DW_ADC_SRC_LATEST a new senors
 *                          sample will be taken and reported.
 *                          If given as DW_ADC_SRC_WAKEUP the reading from
 *                          the last wakeup will be used.
 *
 * \return Temperature measurement from ADC.
 */
float dw_get_temperature( dw_adc_src_t temp_source )
{
  /* Get calibration data from OTP. Tmeas @ 23 degrees resides in addr 0x9. */
  uint32_t otp_temp = dw_read_otp_32(0x009) & 0xFF;
  uint32_t read_temp = 0;

  /* Load to CPU sample */
  switch (temp_source)
  {
    case DW_ADC_SRC_LATEST:
      dw_adc_sample();
      read_temp   = dw_read_subreg_32(DW_REG_TX_CAL, DW_SUBREG_TC_SARL, 
                              DW_SUBLEN_TC_SARL);
      read_temp  &= DW_SAR_LTEMP_MASK;
      read_temp >>= DW_SAR_LTEMP;
      break;

    case DW_ADC_SRC_WAKEUP:
      read_temp   = dw_read_subreg_32(DW_REG_TX_CAL, DW_SUBREG_TC_SARW, 
                              DW_SUBLEN_TC_SARW);
      read_temp  &= DW_SAR_WTEMP_MASK;
      read_temp >>= DW_SAR_WTEMP;
      break;
  }

  return ((float)read_temp - (float)otp_temp)*1.14f + 23.f;
}
/**
 * \brief Gets a voltage reading from the DW1000.
 *
 * \param[in] voltage_source  If given as DW_ADC_SRC_LATEST a new senors
 *                          sample will be taken and reported.
 *                          If given as DW_ADC_SRC_WAKEUP the reading from
 *                          the last wakeup will be used.
 *
 * NOTE: The effective range of measurement is 2.25 V to 3.76 V.
 *
 * \return Voltage measurement from ADC.
 */
float dw_get_voltage( dw_adc_src_t voltage_source )
{
  /* Get calibration data from OTP. Vmeas @ 3.3V residies in addr 0x8. */
  uint32_t otp_voltage = dw_read_otp_32(0x008) & 0xFF;
  uint32_t read_voltage = 0;

  switch (voltage_source)
  {
    case DW_ADC_SRC_LATEST:
      dw_adc_sample();
      read_voltage   = dw_read_subreg_32(DW_REG_TX_CAL, DW_SUBREG_TC_SARL, 
                                            DW_SUBLEN_TC_SARL);
      read_voltage  &= DW_SAR_LVBAT_MASK;
      read_voltage >>= DW_SAR_LVBAT;
      break;

    case DW_ADC_SRC_WAKEUP:
      read_voltage   = dw_read_subreg_32(DW_REG_TX_CAL, DW_SUBREG_TC_SARW, 
                                                        DW_SUBLEN_TC_SARW);
      read_voltage  &= DW_SAR_WVBAT_MASK;
      read_voltage >>= DW_SAR_WVBAT;
      break;
  }

  return ((float)read_voltage - (float)otp_voltage)/173.f + 3.3f;
}
/*===========================================================================*/
/* Communication quality assessment                                          */
/*===========================================================================*/
/**
 * \brief Gives a measure of the standard deviation of the noise level in the
 * data in the Rx Frame Quality Information register (\ref DW_REG_RX_FQUAL).
 * Can  be used as an absolute value or compared to the value reported in the
 * \ref DW_FP_AMPL2 field of the Rx Frame Quality Information register. A large
 * noise value is generally bad. If the noise value is larger than the value
 * in FP_AMPL2 the quality is quite possibly bad.
 *
 * \return Noise level of measurement.
 */
float
dw_get_noise_level()
{
  return (float)((dw_read_reg_64(DW_REG_RX_FQUAL, DW_LEN_RX_FQUAL) 
        & (DW_STD_NOISE_MASK)) >> DW_STD_NOISE);
}
/**
 * \brief Returns the estimated receive signal amplitude in the first path.
 * Used to calculate the estimated power in the first path.
 * \return Amplitude in first path.
 */
float
dw_get_fp_ampl()
{
  return (float)((dw_read_reg_64(DW_REG_RX_FQUAL, DW_LEN_RX_FQUAL) 
        & (DW_FP_AMPL2_MASK)) >> DW_FP_AMPL2);
}
/**
 * \brief Get value required for the computation of the First Path Power Level
 *        and the Receive Power and noise information.
 *        With the following format : F1 F2 F3 N C N_correction NOISE 
 *        and clock_offset detailed in the manual:
 *        In the section 4.7.1 Estimating the signal power in the first path
 *        And in the section 4.7.2 Estimating the receive signal power
 *        N_correction == 1 if the value RXPACC is saturated
 *        All value was unit16_t expect N_correction (uint8_t) and clock_offset.
 */
void 
dw_get_receive_quality(dw1000_frame_quality* quality)
{
  uint16_t rx_pacc_nosat = 0;
  /* Read FP_AMPL1 */
  dw_read_subreg(DW_REG_RX_TIME, DW_SUBREG_FP_AMPL1, 
                                  DW_SUBLEN_FP_AMPL1, 
                                  (uint8_t*) &(quality->fp_ampl1));

  /* Read FP_AMPL2 */
  dw_read_subreg(DW_REG_RX_FQUAL, DW_SUBREG_FP_AMPL2, 
                                  DW_SUBLEN_FP_AMPL2, 
                                  (uint8_t*) &(quality->fp_ampl2));

  /* Read STD_NOISE */
  dw_read_subreg(DW_REG_RX_FQUAL, 0, 2, (uint8_t*) &(quality->std_noise));

  /* Read FP_AMPL3 */
  dw_read_subreg(DW_REG_RX_FQUAL, DW_SUBREG_FP_AMPL3, 
                                  DW_SUBLEN_FP_AMPL3, 
                                  (uint8_t*) &(quality->fp_ampl3));

  /* Read RXPACC */    
  /* N = the Preamble Accumulation Count value reported in the RXPACC */
  /* we read only 2 bytes in place of the all register */
  dw_read_subreg(DW_REG_RX_FINFO, 2, 2, (uint8_t*) &(quality->rx_pacc));
  quality->rx_pacc = quality->rx_pacc >> (DW_RXPACC - 16);
  quality->rx_pacc &= (DW_RXPACC_MASK >> DW_RXPACC);

  /* read RXPACC_NOSAT */
  dw_read_subreg(DW_REG_DRX_CONF, DW_SUBREG_RXPACC_NOSAT, 
                        DW_SUBLEN_RXPACC_NOSAT, (uint8_t*) &rx_pacc_nosat);

  /* CIR_PWR */
  /* C = the Channel Impulse Response Power value reported in the CIR_PWR */
  dw_read_subreg(DW_REG_RX_FQUAL, DW_SUBREG_CIR_PWR, 
                                  DW_SUBLEN_CIR_PWR, 
                                  (uint8_t*) &(quality->cir_pwr));

  /* check if RXPACC is saturated and need to be corrected */
  if(quality->rx_pacc == rx_pacc_nosat)
    quality->n_correction = 0x01;
  else
    quality->n_correction = 0x00;
  quality->clock_offset = dw_get_clock_offset();
}

/**
 * \brief Print value required for the computation of the First Path Power Level
 *        and the Receive Power.
 *        With the following format : F1 F2 F3 N C N_correction NOISE and the 
 *        clock_offset detailed in the manual:
 *        In the section 4.7.1 Estimating the signal power in the first path
 *        And in the section 4.7.2 Estimating the receive signal power
 *        N_correction == 1 if the value RXPACC is saturated.
 *
 */
void 
print_receive_quality(dw1000_frame_quality quality)
{
  printf(" 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X 0x%02X 0x%04X %d", 
      quality.fp_ampl1, quality.fp_ampl2, quality.fp_ampl3, 
      quality.rx_pacc, quality.cir_pwr, quality.n_correction, 
      quality.std_noise, (int) quality.clock_offset);
}

/*===========================================================================*/
/* Error counter                                                             */
/*===========================================================================*/
/**
 * \brief Event Counters Enable. 
 *  Waring: The Event Counters increase the power consumption.
 */
void 
enable_error_counter(void){
  uint16_t value = (0x1 << DW_EVC_EN) & DW_EVC_EN_MASK;
  dw_write_reg(DW_REG_EVC_CTRL, 2, (uint8_t *) &value);
}

/**
 * \brief Event Counters Disable.
 */
void 
disable_error_counter(void){
  uint16_t value = 0;
  dw_write_reg(DW_REG_EVC_CTRL, 2, (uint8_t *) &value);
}
/**
 * \brief Reset the counter of each counter.
 */
void 
reset_error_counter(void){
  uint16_t value = (0x1 << DW_EVC_CLR) & DW_EVC_CLR_MASK;
  /* we need to write at least 2 bytes according the user manual */
  dw_write_reg(DW_REG_EVC_CTRL, 2, (uint8_t *) &value);
  enable_error_counter();
}
/**
 * \brief Display the value of each counter.
 */
void
print_error_counter(void){
  printf("-----------------------------------\n");
  printf("-----------Error Counter-----------\n");
  uint16_t value = 0;
  dw_read_subreg(DW_REG_EVC_CTRL, DW_SUBREG_EVC_PHE, 2, (uint8_t*) &value);
  value &= DW_EVC_PHE_MASK;
  if( value > 0)
    printf("PHR Error Event Counter %d\n", value);

  dw_read_subreg(DW_REG_EVC_CTRL, DW_SUBREG_EVC_RSE, 2, (uint8_t*) &value);
  value &= DW_EVC_RSE_MASK;
  if( value > 0)
    printf("Reed Solomon decoder (Frame Sync Loss) Error Event Counter %d\n", 
            value);

  dw_read_subreg(DW_REG_EVC_CTRL, DW_SUBREG_EVC_FCG, 2, (uint8_t*) &value);
  value &= DW_EVC_FCG_MASK;
  if( value > 0)
    printf("Frame Check Sequence Good Event Counter %d\n", value);

  dw_read_subreg(DW_REG_EVC_CTRL, DW_SUBREG_EVC_FCE, 2, (uint8_t*) &value);
  value &= DW_EVC_FCE_MASK;
  if( value > 0)
    printf("Frame Check Sequence Error Counter %d\n", value);

  dw_read_subreg(DW_REG_EVC_CTRL, DW_SUBREG_EVC_FFR, 2, (uint8_t*) &value);
  value &= DW_EVC_FFR_MASK;
  if( value > 0)
    printf("Frame Filter Rejection Counter %d\n", value);

  dw_read_subreg(DW_REG_EVC_CTRL, DW_SUBREG_EVC_OVR, 2, (uint8_t*) &value);
  value &= DW_EVC_OVR_MASK;
  if( value > 0)
    printf("RX Overrun Error Counter %d\n", value);

  dw_read_subreg(DW_REG_EVC_CTRL, DW_SUBREG_EVC_STO, 2, (uint8_t*) &value);
  value &= DW_EVC_STO_MASK;
  if( value > 0)
    printf("SFD Timeout Error Counter %d\n", value);

  dw_read_subreg(DW_REG_EVC_CTRL, DW_SUBREG_EVC_PTO, 2, (uint8_t*) &value);
  value &= DW_EVC_PTO_MASK;
  if( value > 0)
    printf("Preamble Detection Timeout Event Counter %d\n", value);

  dw_read_subreg(DW_REG_EVC_CTRL, DW_SUBREG_EVC_FWTO, 2, (uint8_t*) &value);
  value &= DW_EVC_FWTO_MASK;
  if( value > 0)
    printf("RX Frame Wait Timeout Counter %d\n", value);

  dw_read_subreg(DW_REG_EVC_CTRL, DW_SUBREG_EVC_TXFS, 2, (uint8_t*) &value);
  value &= DW_EVC_TXFS_MASK;
  if( value > 0)
    printf("TX Frame Sent Counter %d\n", value);

  dw_read_subreg(DW_REG_EVC_CTRL, DW_SUBREG_EVC_HPW, 2, (uint8_t*) &value);
  value &= DW_EVC_TPW_MASK;
  if( value > 0)
    printf("Half Period Warning Counter %d\n", value);

  dw_read_subreg(DW_REG_EVC_CTRL, DW_SUBREG_EVC_TPW, 2, (uint8_t*) &value);
  value &= DW_EVC_TPW_MASK;
  if( value > 0)
    printf("Transmitter Power-Up Warning Counter %d\n", value);
  printf("-----------------------------------\n");
}


/*===========================================================================*/
/* RX/TX                                                                     */
/*===========================================================================*/

/**
 * \brief Get the length of the last frame received.
 * If she is too long return 128 (DW_RX_BUFFER_MAX_LEN)
 * \return The length of the last frame received.
 */
int
dw_get_rx_len(void)
{
  /* we can read only the two first bytes of the register */
  uint16_t rx_frame_info_lo = 0;
  dw_read_subreg(DW_REG_RX_FINFO, 0x0, 2, (uint8_t*) &rx_frame_info_lo);

  /* check if we don't have a to long length */
  rx_frame_info_lo = rx_frame_info_lo & (DW_RXFLEN_MASK | DW_RXFLE_MASK);
  return (rx_frame_info_lo < DW_RX_BUFFER_MAX_LEN) ? 
              (rx_frame_info_lo) : (DW_RX_BUFFER_MAX_LEN);
}
/**
 * \brief Get the length of the last frame received.
 * \return The length of the last frame received.
 */
int
dw_get_rx_extended_len(void)
{
  /* we can read only the two first bytes of the register */
  uint16_t rx_frame_info_lo = 0;
  dw_read_subreg(DW_REG_RX_FINFO, 0x0, 2, (uint8_t*) &rx_frame_info_lo);

  /* check if we don't have a to long length */
  return rx_frame_info_lo & (DW_RXFLEN_MASK | DW_RXFLE_MASK);
}
/**
 * \brief Check error bit in the sys status register.
 *    Change dw1000 state if rx error.
 */
void
dw_get_rx_error()
{
  uint32_t *status_reg;
  uint64_t status_reg_64 = 0ULL;
  uint32_t isError = 0UL;

  const uint32_t error_mask_lo = DW_RXPHE_MASK | DW_RXRFTO_MASK | 
                DW_RXPTO_MASK | DW_RXSFDTO_MASK | DW_RXRFSL_MASK;
  const uint32_t error_mask_hi = DW_RXPREJ_MASK;
  status_reg_64 = dw_read_reg_64(DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS);
  status_reg = (uint32_t *)&status_reg_64;
  isError = *(status_reg + 0) & error_mask_lo;
  isError |= *(status_reg + 1) & error_mask_hi;

  if(isError) {
    dw1000.state = DW_STATE_ERROR;
  } else { dw1000.state = DW_STATE_IDLE;
  }
}
/**
 * \brief Sets the DW1000 rx timeout interval. If no preamble has been
 *      discovered in this time the event flag RXRFTO is set.
 * \param[in] us    Timeout period in microseconds (~1.026 us per tick).
 */
void
dw_set_rx_timeout(uint16_t us)
{
  dw_write_reg(DW_REG_RX_FWTO, DW_LEN_RX_FWTO, (uint8_t *)&us);
}
/**
 * \brief Read the current timeout period from the DW1000.
 * \return The current timeout period in microseconds (~1.026 us per tick).
 */
uint16_t
dw_get_rx_timeout()
{
  return (uint16_t) dw_read_reg_32(DW_REG_RX_FWTO, DW_LEN_RX_FWTO);
}
/**
 * \brief Enables rx timeout. After the period set in register RX_FWTO the bit
 *      RXRFTO will be set in register SYS_STATUS and the reception will be
 *      aborted.
 */
void
dw_enable_rx_timeout()
{
  uint32_t cfgReg = 0UL;
  cfgReg = dw_read_reg_32(DW_REG_SYS_CFG, DW_LEN_SYS_CFG);
  cfgReg |= DW_RXWTOE_MASK;
  dw_write_reg(DW_REG_SYS_CFG, DW_LEN_SYS_CFG, (uint8_t *)&cfgReg);
}
/**
 * \brief Disables rx timeout.
 */
void
dw_disable_rx_timeout()
{
  uint32_t cfgReg = 0UL;
  cfgReg = dw_read_reg_32(DW_REG_SYS_CFG, DW_LEN_SYS_CFG);
  cfgReg &= ~DW_RXWTOE_MASK;
  dw_write_reg(DW_REG_SYS_CFG, DW_LEN_SYS_CFG, (uint8_t *)&cfgReg);
  PRINTF("CFG: %" PRIx32 "\r\n", cfgReg);
}
/**
 * \brief Return if the RX timeout feature is enable.
 */
uint8_t
dw_is_rx_timeout()
{
  uint32_t cfgReg = 0UL;
  cfgReg = dw_read_reg_32(DW_REG_SYS_CFG, DW_LEN_SYS_CFG);
  return (cfgReg & DW_RXWTOE_MASK) > 0;
}

/**
 * \brief Gets the timestamps for the latest received frame.
 * \note  This is the Raw Timestamp for the received frame.
 *        This is the value of the system clock (125 MHz) captured at 
 *        the time of the first chip of the first PHR symbol. The 
 *        precision here is approximately 125 MHz, i.e. the 9 least 
 *        significant bits are zero.
 * \return The reeded timestamps. 
 */
uint64_t
dw_get_rx_raw_timestamp(void)
{
  uint64_t value = 0ULL;
  dw_read_subreg(DW_REG_RX_TIME, DW_SUBREG_RX_RAWST, DW_SUBLEN_RX_RAWST, 
                  (uint8_t *) &value);
  return value;
}
/**
 * \brief Gets the timestamps for the latest received frame.
 * \note  This is the fully adjusted time of reception.
 */
uint64_t
dw_get_rx_timestamp(void)
{
  uint64_t value = 0ULL;
  dw_read_subreg(DW_REG_RX_TIME, DW_SUBREG_RX_STAMP, DW_SUBLEN_RX_STAMP, 
                  (uint8_t*) &value);
  return value;
}
/**
 * \brief Gets the timestamps for the latest transmitted frame.
 * \note  This is the Raw Timestamp for the transmitted frame.
 *        This is the value of the system clock (125 MHz) captured at 
 *        the time of the first chip of the first PHR symbol. The 
 *        precision here is approximately 125 MHz, i.e. the 9 least 
 *        significant bits are zero.
 * \param timestamp The reeded timestamps will be placed here. 
 */
uint64_t
dw_get_tx_raw_timestamp(void)
{
  uint64_t value = 0ULL;
  dw_read_subreg(DW_REG_TX_TIME, DW_SUBREG_TX_RAWST, DW_SUBLEN_TX_RAWST, 
                  (uint8_t*) &value);
  return value;
}
/**
 * \brief Gets the timestamps for the latest transmitted frame.
 * \note  This is the fully adjusted time of transmission.
 */
uint64_t
dw_get_tx_timestamp(void)
{
  uint64_t value = 0ULL;
  dw_read_subreg(DW_REG_TX_TIME, DW_SUBREG_TX_STAMP, DW_SUBLEN_TX_STAMP, 
                  (uint8_t*) &value);
  return value;
}
/**
 * \brief Set the ranging bit in the PHY header (PHR) of the transmitted 
 *        frame to 1. Identifying the frame as a ranging frame.
 *        In some receiver implementations this may be used to enable hardware 
 *        or software associated with time-stamping the frame. In the DW1000 
 *        receiver the time-stamping of the receive frame does not depend or use
 *        the ranging bit in the received PHR.
 */
void
dw_enable_ranging_frame(void)
{
  uint8_t value = 0;
  /* TR bit is the 15nd bit */
  dw_read_subreg(DW_REG_TX_FCTRL, 0x1, 1, &value); 
  value |= (DW_TR_MASK >> 8);
  dw_write_subreg(DW_REG_TX_FCTRL, 0x1, 1, &value);
}
/**
 * \brief Disable the ranging bit in the PHY header (PHR) of the transmitted 
 *        frame. Identifying the frame as not a ranging frame.
 *        In some receiver implementations this may be used to enable hardware 
 *        or software associated with time-stamping the frame. In the DW1000 
 *        receiver the time-stamping of the receive frame does not depend or use
 *        the ranging bit in the received PHR.
 */
void
dw_disable_ranging_frame(void)
{
  uint8_t value = 0;
  /* TR bit is the 15nd bit */
  dw_read_subreg(DW_REG_TX_FCTRL, 0x1, 1, &value); 
  value &= ~(DW_TR_MASK >> 8);
  dw_write_subreg(DW_REG_TX_FCTRL, 0x1, 1, &value);
}
/**
 * \brief Check if the last received frame is a ranging frame. This reflects the
 *        ranging bit in the received PHY header identifying the frame as a 
 *        ranging packet. This value is updated when a good PHR is detected 
 *        (when the RXPHD status bit is set).
 * \return if the last received frame is a ranging frame.
 */
uint8_t 
dw_is_ranging_frame(void)
{
  uint8_t value = 0;
  /* RNG bit is the 15nd bit */
  dw_read_subreg(DW_REG_RX_FINFO, 0x1, 1, &value);
  return (value & (DW_RNG_MASK >> 8)) > 0;
}
/**
 * \brief Set the TX and RX antenna delay.
 *    These antennas delay are used to shift the Tx and Rx timestamps 
 *    and are expressed in tick (~15.65 ps).
 *
 * \details This function assumes the repartition given by the ASP012:
 *      TX antenna delay = given antenna delay * 44%
 *      RX antenna delay = given antenna delay * 56%
 */
void
dw_set_antenna_delay(uint16_t antenna_delay)
{
  dw_set_tx_antenna_delay((uint16_t) (((uint32_t) (antenna_delay) * 44) / 100));
  dw_set_rx_antenna_delay((uint16_t) (((uint32_t) (antenna_delay) * 56) / 100));
  // dw_set_tx_antenna_delay(antenna_delay >> 1);
  // dw_set_rx_antenna_delay(antenna_delay >> 1);
}
/**
 * \brief Set a default value for the antenna delay.
 *  These values come from the DecaRanging source code.
 *  File DecaRangingARMbased/Source_UNDER_LICENSE_ONLY/
 *        DecaRangingEVB1000_MP_rev3p05/src/application/instance_calib.c
 *  "(uint16) ((DWT_PRF_64M_RFDLY/ 2.0) * 1e-9 / DWT_TIME_UNITS)"
 */
void
dw_set_default_antenna_delay(dw1000_prf_t prf){
  uint16_t antenna_delay = 0U;
  if(prf == DW_PRF_16_MHZ){
    antenna_delay = 32837u;
  }
  else{ /* 64 MHz PRF */
    antenna_delay = 32872u;
  }
  dw_set_antenna_delay(antenna_delay);
}
/**
 * \brief Set the TX antenna delay.
 *    This antenna delay was used to shift the Tx and Rx timestamps 
 *    and was expressed in tick (~15.65 ps).
 */
void
dw_set_tx_antenna_delay(uint16_t tx_delay)
{
  dw_write_reg(DW_REG_TX_ANTD, DW_LEN_TX_ANTD, (uint8_t *) &tx_delay);
}
/**
 * \brief Get the TX antenna delay.
 *    This antenna delay was used to shift the Tx and Rx timestamps 
 *    and was expressed in tick (~15.65 ps).
 */
uint16_t
dw_get_tx_antenna_delay(void)
{
  uint16_t tx_delay = 0;
  dw_read_reg(DW_REG_TX_ANTD, DW_LEN_TX_ANTD, (uint8_t *) &tx_delay);
  return tx_delay;
}
/**
 * \brief Set the RX antenna delay.
 *    This antenna delay was used to shift the Tx and Rx timestamps 
 *    and was expressed in tick (~15.65 ps).
 */
void
dw_set_rx_antenna_delay(uint16_t rx_delay)
{
  dw_write_subreg(DW_REG_LDE_IF, DW_SUBREG_LDE_RXANTD, DW_SUBLEN_LDE_RXANTD, 
                  (uint8_t *) &rx_delay);
}
/**
 * \brief Get the RX antenna delay.
 *    This antenna delay was used to shift the Tx and Rx timestamps 
 *    and was expressed in tick (~15.65 ps).
 */
uint16_t
dw_get_rx_antenna_delay(void)
{
  uint16_t rx_delay = 0;
  dw_read_subreg(DW_REG_LDE_IF, DW_SUBREG_LDE_RXANTD, DW_SUBLEN_LDE_RXANTD, 
                  (uint8_t *) &rx_delay);
  return rx_delay;
}
/**
 * \brief Get the current antenna delay. (~15.65 ps per tick)
 * \return The current antenna delay.
 */
uint16_t
dw_get_antenna_delay()
{
  return (uint16_t) dw_read_reg_32(DW_REG_TX_ANTD, DW_LEN_TX_ANTD);
}
/**
 *  \brief Get the Receiver Time Tracking Offset. This value is provide by 
 *      analyzing the correction made by the phase-lock-loop (PLL) to decode the
 *      signal, it provide an estimate of the difference between the 
 *      transmitting and the receiver clock.
 * \return The Receiver Time Tracking Offset in part per million (ppm) * 100.
 *        The value must be between -20*100 and 20*100 (theoretically).
 */
int16_t
dw_get_clock_offset(void){
  /* Clock offset = RX TOFS / RX TTCKI */
  int32_t rx_tofs = 0L;
  uint32_t rx_ttcki = 0UL;

  /* RX TOFS is a signed 19-bit number, the 19nd bit is the sign */
  dw_read_subreg(DW_REG_RX_TTCKO, 0x0, 3, (uint8_t *) &rx_tofs);
  rx_tofs &= DW_RXTOFS_MASK;

  /*
  printf("clock offset1 0x%.2X%.4X\n", (unsigned int) (rx_tofs >> 16), 
                                      (unsigned int) rx_tofs); */

  /* convert a 19 signed bit number to a 32 bits signed number */
  if((rx_tofs & (0x1UL << 18)) != 0){ /* the 19th bit is 1 => negative number */
    /* a signed int is represented in Ones' complement */
    rx_tofs |= ~DW_RXTOFS_MASK; /* all bit between 31 and 19 are set to 1 */
  }

  /*
  printf("clock offset2 0x%.2X%.4X\n", (unsigned int) (rx_tofs >> 16), 
                                      (unsigned int) rx_tofs); */

  /* brief dummy : The value in RXTTCKI will take just one of two values 
      depending on the PRF: 0x01F00000 @ 16 MHz PRF, 
      and 0x01FC0000 @ 64 MHz PRF. */
  rx_ttcki = dw_read_reg_32(DW_REG_RX_TTCKI, DW_LEN_RX_TTCKI);

  /*
  printf("RX TOFS %ld\n", (long int) rx_tofs);
  printf("RX TTCKI %lu\n", (long unsigned int) rx_ttcki);
  */
  int32_t clock_full = (rx_tofs * (1000000LL * 100LL)) / rx_ttcki;
  int16_t clock_offset = clock_full & 0x7FFF;
  /* copy the sign of clock_full */
  clock_offset |= (clock_full >> (32 - 16)) & 0x8000;
  return clock_offset;
}
/**
 * \brief Setter for the delayed transmit/receive register. If delayed operation
 * is enabled the transmission / reception will not take place until the system
 * time has exceeded this value.
 *
 * \r\note The low order nine bits are ignored. Thus, when working with
 * dx_timestamps the macro \ref DW_TIMESTAMP_CLEAR_LOW_9 can be quite helpful.
 */
void
dw_set_dx_timestamp(uint64_t timestamp)
{
  dw_write_reg(DW_REG_DX_TIME, DW_LEN_DX_TIME, (uint8_t *) &timestamp);  

  uint32_t sys_status = 0UL;
  dw_read_subreg(DW_REG_SYS_STATUS, 0x0, 4, (uint8_t*) &sys_status);
  if((sys_status & DW_HPDWARN_MASK) != 0){
    printf("dw_set_dx_timestamp error\n");
  }
}
/**
 * \brief Getter for the delayed transmit/receive register.
 */
uint64_t
dw_get_dx_timestamp()
{
  return dw_read_reg_64(DW_REG_DX_TIME, DW_LEN_DX_TIME) & 0x000000FFFFFFFFFFULL;
}
/**
 * \brief Enables interrupts as specified in parameter mask. Previous values in
 * SYS_MASK are overwritten. Example usage:
 * \code
 * dw_enable_interrupt( DW_MTXFRS_MASK | DW_MRXPHE_MASK | DW_MRXDFR_MASK );
 * \endcode
 * The interrupt constants can be found in dw1000-base.h under BITFIELDS for
 * DW_REG_SYS_MASK.
 *
 * \param[in]   mask  Value to overwrite SYS_MASK register with.
 */
void
dw_enable_interrupt(uint32_t mask)
{
  dw_write_reg(DW_REG_SYS_MASK, DW_LEN_SYS_MASK, (uint8_t *)&mask);
}
/**
 * \brief Clear a pending masked interrupt. Usage same as \ref
 * dw_enable_interrupt.
 *
 * \param[in]   mask  Value to overwrite SYS_MASK register with.
 */
void
dw_clear_pending_interrupt(uint64_t mask)
{
  dw_write_reg(DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS, (uint8_t *)&mask);
}
/**
 * \brief Generate a extended Unique ID (not a IEEE standard).
 * \return a extended Unique ID
 */
uint64_t
dw_generate_extendedUniqueID()
{
  uint64_t eid = 0ULL;
  uint i;
  for(i = 0; i < 4; i++) {
    eid |= ((uint64_t)rand()) << (i * 16);
  }
  return eid;
}
/*-----------------------------------------------------------------------------
   Private functions
   ---------------------------------------------------------------------------*/

/**
 * \brief Aborts current transmission or reception and returns device to idle 
 *        mode.
 */
void
dw_idle(void)
{
  /* write only one bit > DW_TRXOFF is the 6th bit. */
  /* assume that SYS_CTRL is always empty */
  uint8_t sys_ctrl_val= (1 << DW_TRXOFF) & DW_TRXOFF_MASK;
  dw_write_reg(DW_REG_SYS_CTRL, 1, &sys_ctrl_val);

  dw1000.state = DW_STATE_IDLE;
}
/**
 * \brief Configure the receiver for the RX SNIF mode or disable 
 *                   the RX SNIF mode.
 *
 * input parameters:
 * \param[in]      enable - a true (1) value enable the RX SNIF mode;
 *                        - a false (0) value disable the RX SNIF mode;
 *                            i.e. The receiver is always on in RX Mode.
 * \param[in]      rx_on  - SNIFF mode ON period in PACs;
 *                          The DW1000 add automatically 1 on rx_on value.
 * \param[in]      rx_off - SNIFF mode OFF period in us; 
 *                            more precisely, in 1.0256 microsecond intervals.
 */
void 
dw_set_snif_mode(uint8_t enable, uint8_t rx_on, uint8_t rx_off)
{
  uint32_t rx_sniff_val = 0UL;
  if(enable){ /* enable SNIF MODE */ 
    rx_sniff_val |= (rx_on << DW_SNIFF_ONT) & DW_SNIFF_ONT_MASK;
    rx_sniff_val |= (rx_off << DW_SNIFF_OFFT) & DW_SNIFF_OFFT_MASK;
    dw_write_reg(DW_REG_RX_SNIFF, DW_LEN_RX_SNIFF, (uint8_t *) &rx_sniff_val);
  }
  else{ /* Standard RX Mode: always On */
    dw_write_reg(DW_REG_RX_SNIFF, DW_LEN_RX_SNIFF, (uint8_t *) &rx_sniff_val);
  }
}
/**
 * \brief Initiates a new reception on the DW1000. Assumes that it has been
 * configured already.
 */
void
dw_init_rx(void)
{
  dw1000.state = DW_STATE_RECEIVING;
  /* Enable antenna */
  /* RXENAB is the 8th bit (first in the second byte) */
  /* assume that SYS_CTRL is always empty */
  uint8_t sys_ctrl_val = (1 << (DW_RXENAB - 8) & (DW_RXENAB_MASK >> 8));
  dw_write_subreg(DW_REG_SYS_CTRL, 0x1, 1, &sys_ctrl_val);
}
/**
 * \brief Initiates a new reception on the DW1000. Assumes that it has been
 * configured already.
 */
void 
dw_init_delayed_rx(void)
{
  dw1000.state = DW_STATE_RECEIVING;
  /* Enable antenna */
  /* RXENAB is the 8th bit (first in the second byte) */
  /* RXDLYE is the 9th bit (second in the second byte) */
  /* assume that SYS_CTRL is always empty */
  uint8_t sys_ctrl_val = (1 << (DW_RXENAB - 8) & (DW_RXENAB_MASK >> 8));
  sys_ctrl_val |= (1 << (DW_RXDLYE - 8) & (DW_RXDLYE_MASK >> 8));
  dw_write_subreg(DW_REG_SYS_CTRL, 0x1, 1, &sys_ctrl_val);

  uint32_t sys_status = 0UL;
  dw_read_subreg(DW_REG_SYS_STATUS, 0x0, 4, (uint8_t*) &sys_status);
  if((sys_status & DW_HPDWARN_MASK) != 0){
    dw_idle();
    dw_init_rx();
    printf("dw_init_delayed_rx error\n");
  }
}
/**
 * \brief Starts a new transmission. Data must either already be uploaded to
 *        DW1000 or be uploaded VERY shortly.
 *
 * \param wait_4_resp If true, program the transmitter to wait for 
 *                  response.
 * \param delayed If delayed is true, program a delayed transmission.
 */
void
dw_init_tx(uint8_t wait_4_resp, uint8_t delayed)
{
  dw1000.state = DW_STATE_TRANSMITTING;
  /* Start transmission */
  /* Only read and write the first byte of the register */
  uint8_t sys_ctrl_lo = 0;
  dw_read_reg(DW_REG_SYS_CTRL, 1, &sys_ctrl_lo);
  sys_ctrl_lo |= DW_TXSTRT_MASK;

  if(wait_4_resp)
    sys_ctrl_lo |= DW_WAIT4RESP_MASK; /* Wait for Response bit */
  else
    sys_ctrl_lo &= ~DW_WAIT4RESP_MASK;
  if(delayed)
    sys_ctrl_lo |= DW_TXDLYS_MASK; /* Transmitter Delayed Sending bit */
  else
    sys_ctrl_lo &= ~DW_TXDLYS_MASK;

  dw_write_reg(DW_REG_SYS_CTRL, 1, &sys_ctrl_lo);
}

/**
 * \brief Suppress auto-FCS Transmission (on this next frame).
 *        This control works in conjunction with dw_init_tx()
 *
 *        Usage: call dw_suppress_auto_FCS_tx() before dw_init_tx().
 */
void
dw_suppress_auto_FCS_tx(void)
{
  /* Start transmission */
  uint32_t ctrl_reg_val = 0UL;
  ctrl_reg_val = dw_read_reg_32(DW_REG_SYS_CTRL, DW_LEN_SYS_CTRL);
  ctrl_reg_val |= DW_SFCST_MASK;
  dw_write_reg(DW_REG_SYS_CTRL, DW_LEN_SYS_CTRL, (uint8_t *)&ctrl_reg_val);
}
/**
 * \brief Clear receive event status:
 *            Receiver Data Frame Ready.
 *            LDE processing done.
 *            Leading edge detection processing error.
 *            Receiver PHY Header Error.
 *            Receiver FCS Error.
 *            Receiver FCS Good.
 *            Receiver Reed Solomon Frame Sync Loss.
 *            Receive Frame Wait Timeout.
 */
void
dw_clear_receive_status(void)
{
  dw_clear_transmit_status(); /* because auto ACK send trame. */
  uint64_t sys_status = DW_RXPRD_MASK
    | DW_RXSFDD_MASK
    | DW_LDEDONE_MASK
    | DW_RXPHD_MASK
    | DW_RXPHE_MASK
    | DW_RXDFR_MASK
    | DW_RXFCG_MASK
    | DW_RXFCE_MASK
    | DW_RXRFSL_MASK
    | DW_RXRFTO_MASK
    | DW_LDEERR_MASK
    | DW_RXOVRR_MASK
    | DW_RXPTO_MASK
    | DW_AFFREJ_MASK
    | DW_RXRSCS_MASK
    | (DW_RXPREJ_MASK << 32);
  dw_write_reg(DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS, (uint8_t *)&sys_status);
}
/**
 * \brief Clear transmit event status:
 *              Transmit Frame Begins.
 *              Transmit Preamble Sent.
 *              Transmit PHY Header Sent.
 *              Transmit Frame Sent.
 */
void
dw_clear_transmit_status()
{
  uint32_t sys_status = DW_TXFRB_MASK
    | DW_TXPRS_MASK
    | DW_TXPHS_MASK
    | DW_TXFRS_MASK;
  dw_write_reg(DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS, (uint8_t *)&sys_status);
}
/**
 * \brief Check value of system status
 *
 * \param[in]   status  Value of the SYS_STATUS register.
 * \return 1 if Receiver Data Frame Ready otherwise 0.
 */
int
dw_is_receive_done(uint64_t status)
{
  const uint32_t wait_mask_lo = DW_RXDFR_MASK
    | DW_RXPHE_MASK
    | DW_RXRFTO_MASK
    | DW_RXPTO_MASK
    | DW_RXSFDTO_MASK
    | DW_RXRFSL_MASK;
  return (status & wait_mask_lo) > 0;
}
/**
 * \brief Check value of system status
 *
 * \param[in]   status  Value of the SYS_STATUS register.
 * \return 1 if Receiver FCS Good otherwise 0.
 */
int
dw_is_receive_CRC(uint64_t status)
{
  return (status & DW_RXFCG_MASK) > 0;
}
/**
 * \brief Check value of system status
 *        Error if
 *          Leading edge detection processing error.
 *          Receiver FCS Error.
 *          Receiver PHY Header Error.
 *          Receiver Reed Solomon Frame Sync Loss
 *          Automatic Frame Filtering rejection.
 *
 * \param[in]   status  Value of the SYS_STATUS register.
 * \return 1 if error when receive otherwise 0.
 */
int
dw_is_receive_failed(uint64_t status)
{
  uint64_t error_mask = DW_LDEERR_MASK
    | DW_RXFCE_MASK
    | DW_RXPHE_MASK
    | DW_RXRFSL_MASK
    | DW_AFFREJ_MASK;
  return (status & error_mask) > 0; /* Receiver Data Frame Ready. */
}
/**
 * \brief Check value of system status
 *
 * \param[in]   status  Value of the SYS_STATUS register.
 * \return      1       if there are a receive status
 *              0        otherwise
 */
int
dw_is_receive_status(uint64_t status)
{
  uint64_t mask = DW_RXPRD_MASK
    | DW_RXSFDD_MASK
    | DW_LDEDONE_MASK
    | DW_RXPHD_MASK
    | DW_RXPHE_MASK
    | DW_RXDFR_MASK
    | DW_RXFCG_MASK
    | DW_RXFCE_MASK
    | DW_RXRFSL_MASK
    | DW_RXRFTO_MASK
    | DW_LDEERR_MASK
    | DW_RXOVRR_MASK
    | DW_RXPTO_MASK
    | DW_AFFREJ_MASK
    | DW_RXRSCS_MASK;
    /* | (DW_RXPREJ_MASK << 32); Preamble rejection give false 
                                  positive detection*/
  return (status & mask) > 0;
}
/**
 * \brief Check value of system status
 *
 * \param[in]   status  Value of the SYS_STATUS register.
 * \return 1 if Receive Frame Wait Timeout otherwise 0.
 */
int
dw_is_receive_timeout(uint64_t status)
{
  return (status & DW_RXRFTO_MASK) > 0;
}
/*-----------------------------------------------------------------------------
   Test functions
   ---------------------------------------------------------------------------*/

/**
 * \brief Prints a message if SPI-communication is working properly.
 */
void
dw1000_test()
{
  if(dw1000_is_spi_working()) {
    PRINTF("You can now talk with the device!\r\n");
  } else {
    PRINTF("ERROR for talk with the device! %08X\r\n", (unsigned int)
            dw_read_reg_32(DW_REG_DEV_ID, DW_LEN_DEV_ID));
  }
}

/**
 * \brief Return if SPI-communication is working properly.
 */
int
dw1000_is_spi_working()
{
  return ( 0xDECA0130UL == dw_read_reg_32(DW_REG_DEV_ID, 
                                                    DW_LEN_DEV_ID));
}
/**
 * \brief Testing function:
 *     Write 1024 bytes in TX buffer and read 1024 bytes after.
 *     Save data existing in TX buffer and replace this
 *     data at the end.
 */
void
dw1000_test_RW_longbits()
{
  uint16_t size = 1024;
  uint8_t data_write[size], data_read[size], data_save[size];
  uint i;
  for(i = 0; i < size; i++) {
    data_write[i] = (uint8_t)rand();
    data_read[i] = 0;
  }

  PRINTF("READ WRITE TEST on 1024 tx_buffer.\r\n");
  dw_read_reg(DW_REG_TX_BUFFER, size, data_save);
  dw_write_reg(DW_REG_TX_BUFFER, size, data_write);
  dw_read_reg(DW_REG_TX_BUFFER, size, data_read);
  dw_write_reg(DW_REG_TX_BUFFER, size, data_save);

  uint16_t error = 0;
  for(i = 0; i < size; i++) {
    if(data_write[i] != data_read[i]) {
      error++;
    }
  }
  if(error == 0) {
    PRINTF("READ WRITE TEST on 1024 tx_buffer: SUCCESS\r\n");
  } else {
    PRINTF("READ WRITE TEST on 1024 tx_buffer: error: %i dis-concordance\r\n", 
            error);
  }
}
/**
 * \brief Testing function:
 *   dw_write_subreg();
 *   TX led use GPIO3
 *
 */
void
dw1000_test_tx_del_on()
{
  uint32_t reg = 0UL;
  uint32_t subReg = 0UL;
  uint32_t lenReg = 0UL;
  uint32_t data = 0UL;

  /* GPIO_CTRL > GPIO_DIR GPD3 set to 0 */
  reg = DW_REG_GPIO_CTRL;
  subReg = DW_SUBREG_GPIO_DIR;
  lenReg = DW_SUBLEN_GPIO_DIR;
  data = (0 << DW_GDP3) | (1 << DW_GDM3);
  dw_write_subreg(reg, subReg, lenReg, (uint8_t *)&data);

  /* GPIO_CTRL > GPIO_DOUT GOD3 set to 1 */
  subReg = DW_SUBREG_GPIO_DOUT;
  lenReg = DW_SUBLEN_GPIO_DOUT;
  data = (1 << DW_GOP3) | (1 << DW_GOM3);
  dw_write_subreg(reg, subReg, lenReg, (uint8_t *)&data);

  /* GPIO_CTRL > GPIO_MODE MSGP3 set to 1 */
  subReg = DW_SUBREG_GPIO_MODE;
  lenReg = DW_SUBLEN_GPIO_MODE;
  data = (1 << DW_MSGP3) & DW_MSGP3_MASK;
  dw_write_subreg(reg, subReg, lenReg, (uint8_t *)&data);

  PRINTF("Write Testing: TX LED On.\r\n");
}
/*----------------------   ARCH   ---------------------*/

/**
 * \brief       Reads the value from a register on the dw1000 as
 *              a stream of bytes.
 * \param[in] reg_addr Register address as specified in the manual or by
 *                     the DW_REG_* defines.
 * \param[in] reg_len  Number of bytes to read. Should not be longer than
 *                     the length specified in the manual or the DW_LEN_*
 *                     defines.
 * \param[out] pData   Data read from the device.
 */
void
dw_read_reg(uint32_t reg_addr, uint16_t reg_len, uint8_t *pData)
{
  dw_read_subreg(reg_addr, 0x0, reg_len, pData);
  // dw_access_subreg(DW_READ, reg_addr, 0x0, reg_len, pData);
}
/**
 * \brief       Reads the value from a register on the dw1000 as
 *              a 32-bit integer.
 * \param[in] reg_addr Register address as specified in the manual or by
 *                     the DW_REG_* defines.
 * \param[in] reg_len  Number of bytes to read. Should not be longer than
 *                     the length specified in the manual or the DW_LEN_*
 *                     defines. Neither should it be larger than 4 bytes.
 * \return             A 32-bit unsigned integer read from a register
 *                     of the dw1000.
 */
uint32_t
dw_read_reg_32(uint32_t reg_addr, uint16_t reg_len)
{
  uint32_t result = 0UL;
  /* avoid memory corruption */
  assert(reg_len <= 4);

  dw_read_reg(reg_addr, reg_len, (uint8_t *)&result);
  return result;
}
/**
 * \brief Reads the value from a register on the dw1000 as a 64-bit integer.
 * \param[in] reg_addr      Register address as specified in the manual or by
 *                           the DW_REG_* defines.
 * \param[in] reg_len       Number of bytes to read. Should not be longer than
 *                           the length specified in the manual or the DW_LEN_*
 *                           defines. Neither should it be larger than 8 bytes.
 * \return A 64-bit unsigned integer read from a register of the dw1000.
 */
uint64_t
dw_read_reg_64(uint32_t reg_addr, uint16_t reg_len)
{
  uint64_t result = 0ULL;

  /* avoid memory corruption */
  assert(reg_len <= 8);

  dw_read_reg(reg_addr, reg_len, (uint8_t *)&result);
  return result;
}
/**
 * \brief Reads the value from a sub-register on the dw1000 as 32-bit integer.
 * \param[in] reg_addr      Register address as specified in the manual and by
 *                           the DW_REG_* defines.
 * \param[in] subreg_addr   Sub-register address as specified in the manual and
 *                           by the DW_SUBREG_* defines.
 * \param[in] subreg_len    Number of bytes to read. Should not be longer than
 *                           the length specified in the manual or the
 *                           DW_SUBLEN_* defines. Neither should it be larger
 *                           than 4 bytes.
 * \return A 32-bit unsigned integer read from a register of the dw1000.
 */
uint32_t
dw_read_subreg_32(uint32_t reg_addr, uint16_t subreg_addr, uint16_t subreg_len)
{
  uint32_t result = 0ULL;
  dw_read_subreg(reg_addr, subreg_addr, subreg_len, (uint8_t *)&result);
  return result;
}
/**
 * \brief Reads the value from a sub-register on the dw1000 as 64-bit integer.
 * \param[in] reg_addr      Register address as specified in the manual and by
 *                           the DW_REG_* defines.
 * \param[in] subreg_addr   Sub-register address as specified in the manual and
 *                           by the DW_SUBREG_* defines.
 * \param[in] subreg_len    Number of bytes to read. Should not be longer than
 *                           the length specified in the manual or the
 *                           DW_SUBLEN_* defines. Neither should it be larger
 *                           than 8 bytes.
 * \return A 64-bit unsigned integer read from a register of the dw1000.
 */
uint64_t
dw_read_subreg_64(uint32_t reg_addr, uint16_t subreg_addr, uint16_t subreg_len)
{
  uint64_t result = 0ULL;
  dw_read_subreg(reg_addr, subreg_addr, subreg_len, (uint8_t *)&result);
  return result;
}
/**
 * \brief     Writes a stream of bytes to the specified dw1000 register.
 * \param[in] reg_addr  Register address as specified in the manual and by
 *                      the DW_REG_* defines.
 * \param[in] reg_len   Number of bytes to write. Should not be longer than
 *                      the length specified in the manual or the DW_LEN_*
 *                      defines.
 * \param[in] p_data    A stream of bytes to write to device.
 */
void
dw_write_reg(uint32_t reg_addr, uint16_t reg_len, uint8_t *p_data)
{
  dw_write_subreg(reg_addr, 0x0, reg_len, p_data);
  // dw_access_subreg(DW_WRITE, reg_addr, 0x0, reg_len, p_data);

}


/**
 * \brief   Reads a value from the one time programmable memory.
 *
 * \param[in] otp_addr The address to read data from.
 *
 * \return  Contents of the otp memory location.
 */
uint32_t
dw_read_otp_32(uint16_t otp_addr)
{
  static const uint8_t cmd[] = {
    DW_OTPRDEN_MASK | DW_OTPREAD_MASK, /* Enable manual read on specified ADDR*/
    0x00 /* Reset otp_ctrl */
  };

  uint32_t read_data = 0UL;
  dw_write_subreg(DW_REG_OTP_IF, DW_SUBREG_OTP_ADDR, DW_SUBLEN_OTP_ADDR, 
                  (uint8_t *)&otp_addr);
  /* Enable manual read on a specified ADDR */
  dw_write_subreg(DW_REG_OTP_IF, DW_SUBREG_OTP_CTRL, 1, (uint8_t *)&cmd[0]);
  /* Reset the Control register (OTPRDEN is not self clearing) */
  dw_write_subreg(DW_REG_OTP_IF, DW_SUBREG_OTP_CTRL, 1, (uint8_t *)&cmd[1]);
  read_data = dw_read_subreg_32(DW_REG_OTP_IF, DW_SUBREG_OTP_RDAT, 
                                    DW_SUBLEN_OTP_RDAT);

  return read_data;
}
/*-----------------------------------------------------------------------------
   Rx double buffering
   ---------------------------------------------------------------------------*/

/**
 * \brief Enabling double-buffered operation.
 *        See Figure 14: Flow chart for using double RX buffering
 *        of the manual
 */
void
dw_enable_double_buffering(void)
{
  /* enable double-buffered with DIS_DRXB to 0. */
  uint8_t cfgReg = 0;
  dw_read_subreg(DW_REG_SYS_CFG, 0x1, 1, &cfgReg);
  cfgReg &= ~(DW_DIS_DRXB_MASK >> 8); /* 12th bit */
  dw_write_subreg(DW_REG_SYS_CFG, 0x1, 1, &cfgReg);

  dw_enable_automatic_receiver_Re_Enable();
}
/**
 * \brief Check effective and excepted value of receive buffer pointer.
 *        Check HSRBP == ICRBP
 *
 * \return if expected and effective pointer are the same.
 */
int
dw_good_rx_buffer_pointer(void)
{
  uint8_t statusReg = 0;
  dw_read_subreg(DW_REG_SYS_STATUS, 0x3, 1, &statusReg);
  /* HSRBP is the 30th bit */
  /* ICRBP is the 31th bit */
  return (((statusReg & (DW_HSRBP_MASK >> 24)) >> (DW_HSRBP - 24)) ==  
          ((statusReg & (DW_ICRBP_MASK >> 24)) >> (DW_ICRBP - 24))); 
}
/**
 * \brief   Check if an overrun condition occur in the IC receiver.
 *          If an overrun occur reset the receiver with trxoff and rxon.
 *
 * \return if an overrun condition occur in the IC receiver.
 */
int
dw_is_overrun(void)
{
  uint8_t statusReg = 0;
  /* DW_RXOVRR is the 20nd bit > in the 3nd byte */
  dw_read_subreg(DW_REG_SYS_STATUS, 0x2, 1, &statusReg);
  return statusReg & (DW_RXOVRR_MASK >> 16);
}
/**
 * \brief   Apply a receiver-only soft reset.
 *          Call this function if an overrun occur in double buffering mode.
 *          If an overrun occur reset the receiver.
 */
void
dw_trxsoft_reset(void)
{
  /* To apply a receiver-only soft reset, clear and set bit 28 only. */

  /* Clear */
  // ctrlReg &= ~((0x01UL << DW_SOFTRESET) & DW_SOFTRESET_MASK);
  uint8_t ctrlReg = 0x0EUL << (DW_SOFTRESET - 24);
  dw_write_subreg(DW_REG_PMSC, 0x3, 1, &ctrlReg);

  /* Set */
  ctrlReg |= 0x0FUL << (DW_SOFTRESET - 24);
  dw_write_subreg(DW_REG_PMSC, 0x3, 1, &ctrlReg);
}
/**
 * \brief   Change the Receive Buffer Pointer.
 */
void
dw_change_rx_buffer(void)
{
  /* Host Side Receive Buffer Pointer Toggle to 1. */
  uint8_t ctrlReg = 0;
  dw_read_subreg(DW_REG_SYS_CTRL, 0x3, 1, &ctrlReg);
  ctrlReg |= DW_HRBPT_MASK >> 24; /* 24th bit > first bit of the 4th byte */
  dw_write_subreg(DW_REG_SYS_CTRL, 0x3, 1, &ctrlReg);
}
/**
 * \brief   TRXOFF in Double-Buffered Mode.
 *          See Figure 15 : TRXOFF in Double-Buffered Mode
 *          Of the manual
 */
void
dw_trxoff_db_mode(void)
{
  /* Mask Double buffered status bits; FCE, FCG, DFR, LDE_DONE
      to prevent glitch when cleared */
  uint8_t maskReg = 0;
  /* read/write only one byte */
  dw_read_subreg(DW_REG_SYS_MASK, 0x1, 1, &maskReg);
  maskReg |= (DW_MRXFCE_MASK          /* 15th bit */
    | DW_MRXFCG_MASK                  /* 14th bit */
    | DW_MRXDFR_MASK                  /* 13th bit */
    | DW_MLDEDONE_MASK) >> 8;         /* 10th bit */
  dw_write_subreg(DW_REG_SYS_MASK, 0x1, 1, (uint8_t *) &maskReg);

  /* Set TXRXOFF bit = 1, in reg:0D,
      to disable the receiver */
  dw_idle();

  /* Clear RX event flags in SYS_STATUS reg:0F; bits FCE,
      FCG, DFR, LDE_DONE */
  uint8_t statusReg = (DW_RXFCE_MASK  /* 15th bit */
    | DW_RXFCG_MASK                   /* 14th bit */
    | DW_RXDFR_MASK                   /* 13th bit */
    | DW_LDEDONE_MASK) >> 8;          /* 10th bit */
  dw_write_subreg(DW_REG_SYS_STATUS, 0x1, 1, (uint8_t *)&statusReg);

  /* Unmask Double buffered status
      bits; FCE, FCG, DFR, LDE_DONE */
  dw_read_subreg(DW_REG_SYS_MASK, 0x1, 1, &maskReg);
  maskReg &= ~((DW_MRXFCE_MASK        /* 15th bit */
    | DW_MRXFCG_MASK                  /* 14th bit */
    | DW_MRXDFR_MASK                  /* 13th bit */
    | DW_MLDEDONE_MASK) >> 8);        /* 10th bit */
  dw_write_subreg(DW_REG_SYS_MASK, 0x1, 1, (uint8_t *) &maskReg);
}
/**
 * \brief Initiates a new reception on the DW1000.
 *        Before start the transmission check if the Receive Buffer Pointer
 *        is good if not, change the receiver pointer.
 *
 *        See Figure 14: Flow chart for using double RX buffering
 *        of the manual
 */
void
dw_db_init_rx(void)
{
  if(!dw_good_rx_buffer_pointer()) { /* check HSRBP == ICRBP */
    /* Host Side Receive Buffer Pointer Toggle to 1. */
    dw_change_rx_buffer();
  }
  dw_init_rx();
}
/**
 * \brief Clear pending interruption in double buffering mode.
 *      See Figure 14: Flow chart for using double RX buffering
 *      Of the manual
 */
void
dw_db_mode_clear_pending_interrupt(void)
{
  /* Mask Double buffered status bits; FCE, FCG, DFR, LDE_DONE
      to prevent glitch when cleared */
  uint32_t maskReg = dw_read_reg_32(DW_REG_SYS_MASK, DW_LEN_SYS_MASK);
  maskReg |= DW_MRXFCE_MASK
    | DW_MRXFCG_MASK
    | DW_MRXDFR_MASK
    | DW_MLDEDONE_MASK;
  dw_write_reg(DW_REG_SYS_MASK, DW_LEN_SYS_MASK, (uint8_t *)&maskReg);

  /* Clear RX event flags in SYS_STATUS reg:0F; bits FCE,
      FCG, DFR, LDE_DONE */
  uint32_t statusReg = DW_RXFCE_MASK
    | DW_RXFCG_MASK
    | DW_RXDFR_MASK
    | DW_LDEDONE_MASK;
  dw_write_reg(DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS, (uint8_t *)&statusReg);

  /* Unmask Double buffered status
      bits; FCE, FCG, DFR, LDE_DONE */
  maskReg = dw_read_reg_32(DW_REG_SYS_MASK, DW_LEN_SYS_MASK);
  maskReg &= ~(DW_MRXFCE_MASK
               | DW_MRXFCG_MASK
               | DW_MRXDFR_MASK
               | DW_MLDEDONE_MASK);
  dw_write_reg(DW_REG_SYS_MASK, DW_LEN_SYS_MASK, (uint8_t *)&maskReg);
}
/**
 * \brief Configure the DW1000 to be in continuous waves transmit mode.
 *    We follow the procedure gived in the section 8.1 "IC Calibration – 
 *     Crystal Oscillator Trim" of the user manual (v2.10) to achieve this.
 *  \note /!\ The transmitter should be reset before call this function to know
 *          all register value.
 */
void dw_cw_mode(dw1000_channel_t channel){
  uint32_t rx_conf_val = 0x0009A000UL;
  dw_write_reg(DW_REG_RF_CONF, DW_LEN_RF_CONF, (uint8_t*) &rx_conf_val);
  uint32_t psmc_ctrl1_val = 0x00000000UL;
  dw_write_subreg(DW_REG_PMSC, DW_SUBREG_PMSC_CTRL1, DW_SUBLEN_PMSC_CTRL1, 
                    (uint8_t*) &psmc_ctrl1_val);
  dw_set_channel(channel);
  /* PRF 16 MHz is the default configuration */
  dw_set_manual_tx_power(channel, DW_PRF_16_MHZ);
  uint32_t psmc_ctrl0_val = 0x22UL;
  dw_write_subreg(DW_REG_PMSC, DW_SUBREG_PMSC_CTRL0, DW_SUBLEN_PMSC_CTRL0, 
                    (uint8_t*) &psmc_ctrl0_val);
  /* the procedure say to write 4 bytes but 
      the register have a size of 2 bytes */
  uint32_t psmc_txfseq_val = 0x0UL;
  dw_write_subreg(DW_REG_PMSC, DW_SUBREG_PMSC_TXFSEQ, DW_SUBLEN_PMSC_TXFSEQ, 
                    (uint8_t*) &psmc_txfseq_val);  
  rx_conf_val = 0x0009A000UL;
  dw_write_reg(DW_REG_RF_CONF, DW_LEN_RF_CONF, (uint8_t*) &rx_conf_val);
  uint8_t tc_pgtest_val = 0x13;
  dw_write_subreg(DW_REG_TX_CAL, DW_SUBREG_TC_PGTEST, DW_SUBLEN_TC_PGTEST,
            &tc_pgtest_val);

  uint8_t fs_xtalt_val = 0x6F;
  dw_write_subreg(DW_REG_FS_CTRL, DW_SUBREG_FS_XTALT, DW_SUBLEN_FS_XTALT,
                  &fs_xtalt_val);
}
/**
 * \brief This is used adjust the crystal frequency.
 *
 * \param[in]   value - crystal trim value (in range 0x0 to 0x1F) 31 steps 
 *              (~1.5ppm per step).
 *              Based on the "void dwt_xtaltrim(uint8 value)" function in the 
 *              DecaRanging app.
 */
void 
dw_fs_xtalt(uint8_t value)
{
  uint32_t ec_ctrl_val = dw_read_reg_32(DW_REG_EC_CTRL, DW_LEN_EC_CTRL);
  /* enable the Clock PLL lock detect tune 
      Required when using the Crystal Trim Setting 
      Enable reliability of the Clock PLL Lock bit in the SYS STATUS*/
  ec_ctrl_val |= (0x01 << DW_PLLLDT) & DW_PLLLDT_MASK;

  dw_write_reg(DW_REG_EC_CTRL, DW_LEN_EC_CTRL, (uint8_t *) &ec_ctrl_val);

  uint8_t fs_xtalt_val = 0;
  /* Configure the Crystal Trim Setting
    Bits 7:5 must always be set to binary “011”. */
  fs_xtalt_val = DW_FS_XTAL_RESERVED_MASK | 
                (value & DW_XTALT_MASK);
  dw_write_subreg(DW_REG_FS_CTRL, DW_SUBREG_FS_XTALT, DW_SUBLEN_FS_XTALT,
                  (uint8_t *) &fs_xtalt_val);
}