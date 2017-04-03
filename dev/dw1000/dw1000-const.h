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
 *         Constants are from the Decawave DW10000 User Manual that at one
 *      point in time could be found at
 *      http://www.decawave.com/support
 *
 *          Based on the work of Hasan Derhamy & Kim Albertsson
 * \author
 *         Charlier Maximilien  <maximilien-charlier@outlook.com>
 *         Hasan Derhamy        <hasan.derhamy@ltu.se>
 *         Kim Albertsson       <kim.albertsson@ltu.se>
 */

#ifndef DW1000_CONST_H
#define DW1000_CONST_H

#define DW_REG_DEV_ID   0x00 /* \brief Register Device Identifier, address */
#define DW_LEN_DEV_ID   4    /* \brief Register Device Identifier, length  */

#define DW_REG_EID      0x01 /* \brief Register Extended Unique Identifier, address */
#define DW_LEN_EID      8    /* \brief Register Extended Unique Identifier, length  */

#define DW_REG_PANADR   0x03 /* \brief Register PAN Identifier and Short Address, address */
#define DW_LEN_PANADR   4    /* \brief Register PAN Identifier and Short Address, length  */

#define DW_REG_SYS_CFG  0x04 /* \brief Register System Configuration bitmap, address */
#define DW_LEN_SYS_CFG  4    /* \brief Register System Configuration bitmap, length  */

#define DW_REG_SYS_TIME     0x06        /* \brief dummy */
#define DW_LEN_SYS_TIME     5           /* \brief dummy */

#define DW_REG_TX_FCTRL     0x08        /* \brief dummy */
#define DW_LEN_TX_FCTRL     5           /* \brief dummy */
#define DW_SUBREG_TX_FCTRL_LOW  0x00    /* \brief dummy */
#define DW_SUBLEN_TX_FCTRL_LOW  4       /* \brief dummy */
#define DW_SUBREG_TX_FCTRL_HIGH 0x04    /* \brief dummy */
#define DW_SUBLEN_TX_FCTRL_HIGH 4       /* \brief dummy */

#define DW_REG_TX_BUFFER    0x09        /* \brief dummy */
#define DW_LEN_TX_BUFFER    1024        /* \brief dummy */

#define DW_REG_DX_TIME      0x0A        /* \brief dummy */
#define DW_LEN_DX_TIME      5           /* \brief dummy */

#define DW_REG_RX_FWTO      0x0C        /* \brief dummy */
#define DW_LEN_RX_FWTO      2           /* \brief dummy */

#define DW_REG_SYS_CTRL     0x0D        /* \brief dummy */
#define DW_LEN_SYS_CTRL     4           /* \brief dummy */

#define DW_REG_SYS_MASK     0x0E        /* \brief dummy */
#define DW_LEN_SYS_MASK     4           /* \brief dummy */

#define DW_REG_SYS_STATUS   0x0F        /* \brief dummy */
#define DW_LEN_SYS_STATUS   5           /* \brief dummy */

#define DW_REG_RX_FINFO     0x10        /* \brief dummy */
#define DW_LEN_RX_FINFO     4           /* \brief dummy */

#define DW_REG_RX_BUFFER    0x11        /* \brief dummy */
#define DW_LEN_RX_BUFFER    1024        /* \brief dummy */

#define DW_REG_RX_FQUAL     0x12        /* \brief dummy */
#define DW_LEN_RX_FQUAL     8           /* \brief dummy */
#define DW_SUBREG_FP_AMPL2  0x02        /* \brief dummy */
#define DW_SUBLEN_FP_AMPL2  2           /* \brief dummy */
#define DW_SUBREG_FP_AMPL3  0x04        /* \brief dummy */
#define DW_SUBLEN_FP_AMPL3  2           /* \brief dummy */
#define DW_SUBREG_CIR_PWR   0x06        /* \brief dummy */
#define DW_SUBLEN_CIR_PWR   2           /* \brief dummy */

#define DW_REG_RX_TTCKI     0x13        /* \brief dummy */
#define DW_LEN_RX_TTCKI     4           /* \brief dummy */

#define DW_REG_RX_TTCKO     0x14        /* \brief dummy */
#define DW_LEN_RX_TTCKO     5           /* \brief dummy */

#define DW_REG_ACK_RESP     0x1A        /* \brief dummy */
#define DW_LEN_ACK_RESP     4           /* \brief dummy */

#define DW_REG_TX_POWER     0x1E        /* \brief dummy */
#define DW_LEN_TX_POWER     4           /* \brief dummy */

#define DW_REG_CHAN_CTRL    0x1F        /* \brief dummy */
#define DW_LEN_CHAN_CTRL    4           /* \brief dummy */

#define DW_REG_AGC_CTRL     0x23        /* \brief dummy */
#define DW_LEN_AGC_CTRL     32          /* \brief dummy */
#define DW_SUBREG_AGC_CTRL1 0x02        /* \brief dummy */
#define DW_SUBLEN_AGC_CTRL1 2           /* \brief dummy */
#define DW_SUBREG_AGC_TUNE1 0x04        /* \brief dummy */
#define DW_SUBLEN_AGC_TUNE1 2           /* \brief dummy */
#define DW_SUBREG_AGC_TUNE2 0x0C        /* \brief dummy */
#define DW_SUBLEN_AGC_TUNE2 4           /* \brief dummy */
#define DW_SUBREG_AGC_TUNE3 0x12        /* \brief dummy */
#define DW_SUBLEN_AGC_TUNE3 2           /* \brief dummy */

#define DW_REG_RX_TIME      0x15        /* \brief dummy */
#define DW_LEN_RX_TIME      14          /* \brief dummy */
#define DW_SUBREG_RX_STAMP  0x00        /* \brief dummy */
#define DW_SUBLEN_RX_STAMP  5           /* \brief dummy */
#define DW_SUBREG_FP_INDEX  0x05        /* \brief dummy */
#define DW_SUBLEN_FP_INDEX  2           /* \brief dummy */
#define DW_SUBREG_FP_AMPL1  0x07        /* \brief dummy */
#define DW_SUBLEN_FP_AMPL1  2           /* \brief dummy */
#define DW_SUBREG_RX_RAWST  0x09        /* \brief dummy */
#define DW_SUBLEN_RX_RAWST  5           /* \brief dummy */

#define DW_REG_TX_TIME      0x17        /* \brief dummy */
#define DW_LEN_TX_TIME      10          /* \brief dummy */
#define DW_SUBREG_TX_STAMP  0x00        /* \brief dummy */
#define DW_SUBLEN_TX_STAMP  5           /* \brief dummy */
#define DW_SUBREG_TX_RAWST  0x05        /* \brief dummy */
#define DW_SUBLEN_TX_RAWST  5           /* \brief dummy */

#define DW_REG_TX_ANTD      0x18        /* \brief dummy */
#define DW_LEN_TX_ANTD      2           /* \brief dummy */

#define DW_REG_GPIO_CTRL    0x26        /* \brief dummy */
#define DW_LEN_GPIO_CTRL    4           /* \brief dummy */
#define DW_SUBREG_GPIO_MODE 0x00        /* \brief dummy */
#define DW_SUBLEN_GPIO_MODE 4           /* \brief dummy */
#define DW_SUBREG_GPIO_DIR  0x08        /* \brief dummy */
#define DW_SUBLEN_GPIO_DIR  4           /* \brief dummy */
#define DW_SUBREG_GPIO_DOUT 0x0C        /* \brief dummy */
#define DW_SUBLEN_GPIO_DOUT 4           /* \brief dummy */

#define DW_REG_DRX_CONF        0x27       /* \brief dummy */
#define DW_SUBREG_DRX_TUNE0b   0x02       /* \brief dummy */
#define DW_SUBLEN_DRX_TUNE0b   2          /* \brief dummy */
#define DW_SUBREG_DRX_TUNE1a   0x04       /* \brief dummy */
#define DW_SUBLEN_DRX_TUNE1a   2          /* \brief dummy */
#define DW_SUBREG_DRX_TUNE1b   0x06       /* \brief dummy */
#define DW_SUBLEN_DRX_TUNE1b   2          /* \brief dummy */
#define DW_SUBREG_DRX_TUNE2    0x08       /* \brief dummy */
#define DW_SUBLEN_DRX_TUNE2    4          /* \brief dummy */
#define DW_SUBREG_DRX_SFDTOC   0x20       /* \brief dummy */
#define DW_SUBLEN_DRX_SFDTOC   2          /* \brief dummy */
#define DW_SUBREG_DRX_TUNE4h   0x26       /* \brief dummy */
#define DW_SUBLEN_DRX_TUNE4h   2          /* \brief dummy */
#define DW_SUBREG_RXPACC_NOSAT 0x2C       /* \brief dummy */
#define DW_SUBLEN_RXPACC_NOSAT 2          /* \brief dummy */

#define DW_REG_RF_CONF       0x28       /* \brief dummy */
#define DW_SUBREG_RF_RXCTRLH 0x0B       /* \brief dummy */
#define DW_SUBLEN_RF_RXCTRLH 1          /* \brief dummy */
#define DW_SUBREG_RF_TXCTRL  0x0C       /* \brief dummy */
#define DW_SUBLEN_RF_TXCTRL  4          /* \brief dummy */

#define DW_REG_TX_CAL        0x2A       /* \brief dummy */
#define DW_SUBREG_TC_SARC    0x00       /* \brief dummy */
#define DW_SUBLEN_TC_SARC    3          /* \brief dummy */
#define DW_SUBREG_TC_SARL    0x03       /* \brief dummy */
#define DW_SUBLEN_TC_SARL    3          /* \brief dummy */
#define DW_SUBREG_TC_SARW    0x06       /* \brief dummy */
#define DW_SUBLEN_TC_SARW    2          /* \brief dummy */
#define DW_SUBREG_TC_PGDELAY 0x0B       /* \brief dummy */
#define DW_SUBLEN_TC_PGDELAY 1          /* \brief dummy */

#define DW_REG_FS_CTRL       0x2B       /* \brief dummy */
#define DW_SUBREG_FS_PLLCFG  0x07       /* \brief dummy */
#define DW_SUBLEN_FS_PLLCFG  4          /* \brief dummy */
#define DW_SUBREG_FS_PLLTUNE 0x0B       /* \brief dummy */
#define DW_SUBLEN_FS_PLLTUNE 1          /* \brief dummy */
#define DW_SUBREG_FS_XTALT   0x0E       /* \brief dummy */
#define DW_SUBLEN_FS_XTALT   1          /* \brief dummy */

#define DW_REG_AON_WCFG      0x2C       /* \brief dummy */
#define DW_SUBREG_AON_WCFG   0x00       /* \brief dummy */
#define DW_SUBLEN_AON_WCFG   2          /* \brief dummy */
#define DW_SUBREG_AON_CTRL   0x0B       /* \brief dummy */
#define DW_SUBLEN_AON_CTRL   1          /* \brief dummy */
#define DW_SUBREG_AON_RDAT   0x0B       /* \brief dummy */
#define DW_SUBLEN_AON_RDAT   1          /* \brief dummy */
#define DW_SUBREG_AON_ADDR   0x0B       /* \brief dummy */
#define DW_SUBLEN_AON_ADDR   1          /* \brief dummy */
#define DW_SUBREG_AON_CFG0   0x0B       /* \brief dummy */
#define DW_SUBLEN_AON_CFG0   4          /* \brief dummy */
#define DW_SUBREG_AON_CFG1   0x0B       /* \brief dummy */
#define DW_SUBLEN_AON_CFG1   2          /* \brief dummy */

#define DW_REG_OTP_IF       0x2D        /* \brief dummy */
#define DW_SUBREG_OTP_WDAT  0x00        /* \brief dummy */
#define DW_SUBLEN_OTP_WDAT  4           /* \brief dummy */
#define DW_SUBREG_OTP_ADDR  0x04        /* \brief dummy */
#define DW_SUBLEN_OTP_ADDR  2           /* \brief dummy */
#define DW_SUBREG_OTP_CTRL  0x06        /* \brief dummy */
#define DW_SUBLEN_OTP_CTRL  2           /* \brief dummy */
#define DW_SUBREG_OTP_STAT  0x08        /* \brief dummy */
#define DW_SUBLEN_OTP_STAT  2           /* \brief dummy */
#define DW_SUBREG_OTP_RDAT  0x0A        /* \brief dummy */
#define DW_SUBLEN_OTP_RDAT  4           /* \brief dummy */
#define DW_SUBREG_OTP_SRDAT 0x0E        /* \brief dummy */
#define DW_SUBLEN_OTP_SRDAT 4           /* \brief dummy */
#define DW_SUBREG_OTP_SF    0x12        /* \brief dummy */
#define DW_SUBLEN_OTP_SF    1           /* \brief dummy */

#define DW_REG_LDE_IF        0x2E       /* \brief dummy */
#define DW_SUBREG_LDE_CFG1   0x0806     /* \brief dummy */
#define DW_SUBLEN_LDE_CFG1   1          /* \brief dummy */
#define DW_SUBREG_LDE_RXANTD 0x1804     /* \brief dummy */
#define DW_SUBLEN_LDE_RXANTD 2          /* \brief dummy */
#define DW_SUBREG_LDE_CFG2   0x1806     /* \brief dummy */
#define DW_SUBLEN_LDE_CFG2   2          /* \brief dummy */
#define DW_SUBREG_LDE_REPC   0x2804     /* \brief dummy */
#define DW_SUBLEN_LDE_REPC   2          /* \brief dummy */

#define DW_REG_EVC_CTRL      0x2F        /* \brief dummy */
#define DW_LEN_EVC_CTRL      4           /* \brief dummy */
#define DW_SUBREG_EVC_PHE    0x4         /* \brief dummy */
#define DW_SUBLEN_EVC_PHE    2           /* \brief dummy */
#define DW_SUBREG_EVC_RSE    0x6         /* \brief dummy */
#define DW_SUBLEN_EVC_RSE    2           /* \brief dummy */
#define DW_SUBREG_EVC_FCG    0x8         /* \brief dummy */
#define DW_SUBLEN_EVC_FCG    2           /* \brief dummy */
#define DW_SUBREG_EVC_FCE    0xA         /* \brief dummy */
#define DW_SUBLEN_EVC_FCE    2           /* \brief dummy */
#define DW_SUBREG_EVC_FFR    0xC         /* \brief dummy */
#define DW_SUBLEN_EVC_FFR    2           /* \brief dummy */
#define DW_SUBREG_EVC_OVR    0xE         /* \brief dummy */
#define DW_SUBLEN_EVC_OVR    2           /* \brief dummy */
#define DW_SUBREG_EVC_STO    0x10        /* \brief dummy */
#define DW_SUBLEN_EVC_STO    2           /* \brief dummy */
#define DW_SUBREG_EVC_PTO    0x12        /* \brief dummy */
#define DW_SUBLEN_EVC_PTO    2           /* \brief dummy */
#define DW_SUBREG_EVC_FWTO   0x14        /* \brief dummy */
#define DW_SUBLEN_EVC_FWTO   2           /* \brief dummy */
#define DW_SUBREG_EVC_TXFS   0x16        /* \brief dummy */
#define DW_SUBLEN_EVC_TXFS   2           /* \brief dummy */
#define DW_SUBREG_EVC_HPW    0x18        /* \brief dummy */
#define DW_SUBLEN_EVC_HPW    2           /* \brief dummy */
#define DW_SUBREG_EVC_TPW    0x1A        /* \brief dummy */
#define DW_SUBLEN_EVC_TPW    2           /* \brief dummy */

#define DW_REG_PMSC          0x36        /* \brief dummy */
#define DW_SUBREG_PMSC_CTRL0 0x00        /* \brief dummy */
#define DW_SUBLEN_PMSC_CTRL0 0x04        /* \brief dummy */

#define DW_SUBREG_PMSC_LEDC  0x28        /* \brief dummy */
#define DW_SUBLEN_PMSC_LEDC  0x04        /* \brief dummy */

/*=================================================
   =========== Bitfields
   =================================================*/

/* DW_REG_SYS_CFG 0x04 */
#define DW_FFEN          0              /* \brief dummy */
#define DW_FFEN_MASK     (0x1UL << 0)   /* \brief dummy */
#define DW_FFBC          1              /* \brief dummy */
#define DW_FFBC_MASK     (0x1UL << 1)   /* \brief dummy */
#define DW_FFAB          2              /* \brief dummy */
#define DW_FFAB_MASK     (0x1UL << 2)   /* \brief dummy */
#define DW_FFAD          3              /* \brief dummy */
#define DW_FFAD_MASK     (0x1UL << 3)   /* \brief dummy */
#define DW_FFAA          4              /* \brief dummy */
#define DW_FFAA_MASK     (0x1UL << 4)   /* \brief dummy */
#define DW_FFAM          5              /* \brief dummy */
#define DW_FFAM_MASK     (0x1UL << 5)   /* \brief dummy */
#define DW_DIS_DRXB      12             /* \brief dummy */
#define DW_DIS_DRXB_MASK (0x1UL << 12)  /* \brief dummy */
#define DW_DIS_PHE       13             /* \brief dummy */
#define DW_DIS_PHE_MASK  (0x1UL << 13)  /* \brief dummy */
#define DW_PHR_MODE      16             /* \brief dummy */
#define DW_PHR_MODE_MASK (0x3UL << 16)  /* \brief dummy */
#define DW_DIS_STXP      18             /* \brief dummy */
#define DW_DIS_STXP_MASK (0x1UL << 18)  /* \brief dummy */
#define DW_RXM110K       22             /* \brief dummy */
#define DW_RXM110K_MASK  (0x1UL << 22)  /* \brief dummy */
#define DW_RXWTOE        28             /* \brief dummy */
#define DW_RXWTOE_MASK   (0x1UL << 28)  /* \brief dummy */
#define DW_RXAUTR        29             /* \brief dummy */
#define DW_RXAUTR_MASK   (0x1UL << 29)  /* \brief dummy */
#define DW_AUTOACK       30             /* \brief dummy */
#define DW_AUTOACK_MASK  (0x1UL << 30)  /* \brief dummy */

/* DW_REG_TX_FCTRL 0x08 */
/* DW_SUBREG_TX_FCTRL_LOW 0x00 */
#define DW_TFLEN         0                  /* \brief dummy */
#define DW_TFLEN_MASK    (0x7FUL << 0)      /* \brief dummy */
#define DW_TFLE          7                  /* \brief dummy */
#define DW_TFLE_MASK     (0x07UL << 7)      /* \brief dummy */
#define DW_TXBR          13                 /* \brief dummy */
#define DW_TXBR_MASK     (0x3UL << 13)      /* \brief dummy */
#define DW_TR            15                 /* \brief dummy */
#define DW_TR_MASK       (0x1UL << 15)      /* \brief dummy */
#define DW_TXPRF         16                 /* \brief dummy */
#define DW_TXPRF_MASK    (0x3UL << 16)      /* \brief dummy */
#define DW_TXPSR         18                 /* \brief dummy */
#define DW_TXPSR_MASK    (0x3UL << 18)      /* \brief dummy */
#define DW_PE            20                 /* \brief dummy */
#define DW_PE_MASK       (0x3UL << 20)      /* \brief dummy */
#define DW_TXBOFFS       22                 /* \brief dummy */
#define DW_TXBOFFS_MASK  (0x3FFUL << 22)    /* \brief dummy */
/* DW_SUBREG_TX_FCTRL_HIGH 0x04 */
#define DW_IFSDELAY      0                  /* \brief dummy */
#define DW_IFSDELAY_MASK (0xFFUL << 0)      /* \brief dummy */

/* DW_REG_SYS_CTRL 0x0D */
#define DW_SFCST          0             /* \brief dummy */
#define DW_SFCST_MASK     (0x1UL << 0)  /* \brief dummy */
#define DW_TXSTRT         1             /* \brief dummy */
#define DW_TXSTRT_MASK    (0x1UL << 1)  /* \brief dummy */
#define DW_TXDLYS         2             /* \brief dummy */
#define DW_TXDLYS_MASK    (0x1UL << 2)  /* \brief dummy */
#define DW_TRXOFF         6             /* \brief dummy */
#define DW_TRXOFF_MASK    (0x1UL << 6)  /* \brief dummy */
#define DW_WAIT4RESP      7             /* \brief dummy */
#define DW_WAIT4RESP_MASK (0x1UL << 7)  /* \brief dummy */
#define DW_RXENAB         8             /* \brief dummy */
#define DW_RXENAB_MASK    (0x1UL << 8)  /* \brief dummy */
#define DW_RXDLYE         9             /* \brief dummy */
#define DW_RXDLYE_MASK    (0x1UL << 9)  /* \brief dummy */
#define DW_HRBPT          24            /* \brief dummy */
#define DW_HRBPT_MASK     (0x1UL << 24) /* \brief dummy */

/* DW_REG_SYS_MASK 0x0E */
#define DW_MTXFRB        4              /* \brief dummy */
#define DW_MTXFRB_MASK   (0x1UL << 4)   /* \brief dummy */
#define DW_MTXPRS        5              /* \brief dummy */
#define DW_MTXPRS_MASK   (0x1UL << 5)   /* \brief dummy */
#define DW_MTXPHS        6              /* \brief dummy */
#define DW_MTXPHS_MASK   (0x1UL << 6)   /* \brief dummy */
#define DW_MTXFRS        7              /* \brief dummy */
#define DW_MTXFRS_MASK   (0x1UL << 7)   /* \brief dummy */
#define DW_MRXPRD        8              /* \brief dummy */
#define DW_MRXPRD_MASK   (0x1UL << 8)   /* \brief dummy */
#define DW_MRXSFDD       9              /* \brief dummy */
#define DW_MRXSFDD_MASK  (0x1UL << 9)   /* \brief dummy */
#define DW_MLDEDONE      10             /* \brief dummy */
#define DW_MLDEDONE_MASK (0x1UL << 10)  /* \brief dummy */
#define DW_MRXPHD        11             /* \brief dummy */
#define DW_MRXPHD_MASK   (0x1UL << 11)  /* \brief dummy */
#define DW_MRXPHE        12             /* \brief dummy */
#define DW_MRXPHE_MASK   (0x1UL << 12)  /* \brief dummy */
#define DW_MRXDFR        13             /* \brief dummy */
#define DW_MRXDFR_MASK   (0x1UL << 13)  /* \brief dummy */
#define DW_MRXFCG        14             /* \brief dummy */
#define DW_MRXFCG_MASK   (0x1UL << 14)  /* \brief dummy */
#define DW_MRXFCE        15             /* \brief dummy */
#define DW_MRXFCE_MASK   (0x1UL << 15)  /* \brief dummy */
#define DW_MRXRFSL       16             /* \brief dummy */
#define DW_MRXRFSL_MASK  (0x1UL << 16)  /* \brief dummy */
#define DW_MRXRFTO       17             /* \brief dummy */
#define DW_MRXRFTO_MASK  (0x1UL << 17)  /* \brief dummy */
#define DW_MLDEERR       18             /* \brief dummy */
#define DW_MLDEERR_MASK  (0x1UL << 18)  /* \brief dummy */
#define DW_MRXOVRR       20             /* \brief dummy */
#define DW_MRXOVRR_MASK  (0x1UL << 20)  /* \brief dummy */
#define DW_MRXPTO        21             /* \brief dummy */
#define DW_MRXPTO_MASK   (0x1UL << 21)  /* \brief dummy */
#define DW_MRXSFDTO      26             /* \brief dummy */
#define DW_MRXSFDTO_MASK (0x1UL << 26)  /* \brief dummy */
#define DW_MHPDWARN      27             /* \brief dummy */
#define DW_MHPDWARN_MASK (0x1UL << 27)  /* \brief dummy */
#define DW_MAFFREJ       29             /* \brief dummy */
#define DW_MAFFREJ_MASK  (0x1UL << 29)  /* \brief dummy */

/* DW_REG_SYS_STATUS 0x0F LOW */
#define DW_IRQS          0              /* \brief dummy */
#define DW_IRQS_MASK     (0x1UL << 0)   /* \brief dummy */
#define DW_TXFRB         4              /* \brief dummy */
#define DW_TXFRB_MASK    (0x1UL << 4)   /* \brief dummy */
#define DW_TXPRS         5              /* \brief dummy */
#define DW_TXPRS_MASK    (0x1UL << 5)   /* \brief dummy */
#define DW_TXPHS         6              /* \brief dummy */
#define DW_TXPHS_MASK    (0x1UL << 6)   /* \brief dummy */
#define DW_TXFRS         7              /* \brief dummy */
#define DW_TXFRS_MASK    (0x1UL << 7)   /* \brief dummy */
#define DW_RXPRD         8              /* \brief dummy */
#define DW_RXPRD_MASK    (0x1UL << 8)   /* \brief dummy */
#define DW_RXSFDD        9              /* \brief dummy */
#define DW_RXSFDD_MASK   (0x1UL << 9)   /* \brief dummy */
#define DW_LDEDONE       10             /* \brief dummy */
#define DW_LDEDONE_MASK  (0x1UL << 10)  /* \brief dummy */
#define DW_RXPHD         11             /* \brief dummy */
#define DW_RXPHD_MASK    (0x1UL << 11)  /* \brief dummy */
#define DW_RXPHE         12             /* \brief dummy */
#define DW_RXPHE_MASK    (0x1UL << 12)  /* \brief dummy */
#define DW_RXDFR         13             /* \brief dummy */
#define DW_RXDFR_MASK    (0x1UL << 13)  /* \brief dummy */
#define DW_RXFCG         14             /* \brief dummy */
#define DW_RXFCG_MASK    (0x1UL << 14)  /* \brief dummy */
#define DW_RXFCE         15             /* \brief dummy */
#define DW_RXFCE_MASK    (0x1UL << 15)  /* \brief dummy */
#define DW_RXRFSL        16             /* \brief dummy */
#define DW_RXRFSL_MASK   (0x1UL << 16)  /* \brief dummy */
#define DW_RXRFTO        17             /* \brief dummy */
#define DW_RXRFTO_MASK   (0x1UL << 17)  /* \brief dummy */
#define DW_LDEERR        18             /* \brief dummy */
#define DW_LDEERR_MASK   (0x1UL << 18)  /* \brief dummy */
#define DW_RXOVRR        20             /* \brief dummy */
#define DW_RXOVRR_MASK   (0x1UL << 20)  /* \brief dummy */
#define DW_RXPTO         21             /* \brief dummy */
#define DW_RXPTO_MASK    (0x1UL << 21)  /* \brief dummy */
#define DW_GPIOIRQ       22             /* \brief dummy */
#define DW_GPIOIRQ_MASK  (0x1UL << 22)  /* \brief dummy */
#define DW_SLP2INIT      23             /* \brief dummy */
#define DW_SLP2INIT_MASK (0x1UL << 23)  /* \brief dummy */
#define DW_RFPLL_LL      24             /* \brief dummy */
#define DW_RFPLL_LL_MASK (0x1UL << 24)  /* \brief dummy */
#define DW_CLKPLL_LL      25             /* \brief dummy */
#define DW_CLKPLL_LL_MASK (0x1UL << 25)  /* \brief dummy */
#define DW_RXSFDTO       26             /* \brief dummy */
#define DW_RXSFDTO_MASK  (0x1UL << 26)  /* \brief dummy */
#define DW_HPDWARN       27             /* \brief dummy */
#define DW_HPDWARN_MASK  (0x1UL << 27)  /* \brief dummy */
#define DW_TXBERR        28             /* \brief dummy */
#define DW_TXBERR_MASK   (0x1UL << 28)  /* \brief dummy */
#define DW_AFFREJ        29             /* \brief dummy */
#define DW_AFFREJ_MASK   (0x1UL << 29)  /* \brief dummy */
#define DW_HSRBP         30             /* \brief dummy */
#define DW_HSRBP_MASK    (0x1UL << 30)  /* \brief dummy */
#define DW_ICRBP         31             /* \brief dummy */
#define DW_ICRBP_MASK    (0x1UL << 31)  /* \brief dummy */
/* DW_REG_SYS_STATUS 0x0F HIGH */
#define DW_RXRSCS      0                /* \brief dummy */
#define DW_RXRSCS_MASK (0x1ULL << 0)    /* \brief dummy */
#define DW_RXPREJ      1                /* \brief dummy */
#define DW_RXPREJ_MASK (0x1ULL << 1)    /* \brief dummy */
#define DW_TXPUTE      2                /* \brief dummy */
#define DW_TXPUTE_MASK (0x1ULL << 2)    /* \brief dummy */

/* DW_REG_RX_FINFO 0x10 */
#define DW_RXFLEN      0                /* \brief dummy */
#define DW_RXFLEN_MASK (0x7FUL << 0)    /* \brief dummy */
#define DW_RXFLE       7                /* \brief dummy */
#define DW_RXFLE_MASK  (0x07UL << 7)    /* \brief dummy */
#define DW_RXNSPL      11               /* \brief dummy */
#define DW_RXNSPL_MASK (0x3UL << 11)    /* \brief dummy */
#define DW_RXBR        13               /* \brief dummy */
#define DW_RXBR_MASK   (0x3UL << 13)    /* \brief dummy */
#define DW_RNG         15               /* \brief dummy */
#define DW_RNG_MASK    (0x1UL << 15)    /* \brief dummy */
#define DW_RXPRFR      16               /* \brief dummy */
#define DW_RXPRFR_MASK (0x3UL << 16)    /* \brief dummy */
#define DW_RXPSR       18               /* \brief dummy */
#define DW_RXPSR_MASK  (0x3UL << 18)    /* \brief dummy */
#define DW_RXPACC      20               /* \brief dummy */
#define DW_RXPACC_MASK (0xFFFUL << 20)  /* \brief dummy */

/* DW_REG_ACK_RESPL 0x1A */
#define DW_W4R_TIM      0                   /* \brief dummy */
#define DW_W4R_TIM_MASK (0xFFFFFULL << 0)   /* \brief dummy */
#define DW_ACK_TIM      24                  /* \brief dummy */
#define DW_ACK_TIM_MASK (0xFFULL << 24)     /* \brief dummy */

/* DW_REG_RX_FQUAL 0x12 */
#define DW_STD_NOISE       0                 /* \brief dummy */
#define DW_STD_NOISE_MASK  (0xFFFFULL << 0)  /* \brief dummy */
#define DW_FP_AMPL2        16                /* \brief dummy */
#define DW_FP_AMPL2_MASK   (0xFFFFULL << 16) /* \brief dummy */
#define DW_FP_AMPL3        32                /* \brief dummy */
#define DW_FP_AMPL3_MASK   (0xFFFFULL << 32) /* \brief dummy */
#define DW_CIR_PWR         48                /* \brief dummy */
#define DW_CIR_PWR_MASK    (0xFFFFULL << 48) /* \brief dummy */

/* DW_REG_RX_TTCKI 0x13 */
#define DW_RXTTCKI         0                    /* \brief dummy */
#define DW_RXTTCKI_MASK    (0xFFFFFFFFULL << 0) /* \brief dummy */

/* DW_REG_RX_TTCKO 0x14 */
#define DW_RXTOFS         0                 /* \brief dummy */
#define DW_RXTOFS_MASK    (0x7FFFFULL << 0)   /* \brief dummy */

/* DW_REG_CHAN_CTRL 0x1F */
#define DW_TXCHAN        0              /* \brief dummy */
#define DW_TXCHAN_MASK   (0x1FUL << 0)  /* \brief dummy */  
#define DW_RXCHAN        5              /* \brief dummy */
#define DW_RXCHAN_MASK   (0x1FUL << 5)  /* \brief dummy */
#define DW_DWSFD         17             /* \brief dummy */
#define DW_DWSFD_MASK    (0x1UL << 17)  /* \brief dummy */
#define DW_RXPRF         18             /* \brief dummy */
#define DW_RXPRF_MASK    (0x03UL << 18) /* \brief dummy */
#define DW_TNSSFD        20             /* \brief dummy */
#define DW_TNSSFD_MASK   (0x1UL << 20)  /* \brief dummy */
#define DW_RNSSFD        21             /* \brief dummy */
#define DW_RNSSFD_MASK   (0x1UL << 21)  /* \brief dummy */
#define DW_TX_PCODE      22             /* \brief dummy */
#define DW_TX_PCODE_MASK (0x1FUL << 22) /* \brief dummy */
#define DW_RX_PCODE      27             /* \brief dummy */
#define DW_RX_PCODE_MASK (0x1FUL << 27) /* \brief dummy */

/* DW_REG_GPIO_CTRL 0x26 */
/* DW_SUBREG_GPIO_MODE 0x26:00*/
#define DW_MSGP8_MASK (0x3 << 22)   /* \brief dummy */
#define DW_MSGP8      22            /* \brief dummy */
#define DW_MSGP7_MASK (0x3 << 20)   /* \brief dummy */
#define DW_MSGP7      20            /* \brief dummy */
#define DW_MSGP6_MASK (0x3 << 18)   /* \brief dummy */
#define DW_MSGP6      18            /* \brief dummy */
#define DW_MSGP5_MASK (0x3 << 16)   /* \brief dummy */
#define DW_MSGP5      16            /* \brief dummy */
#define DW_MSGP4_MASK (0x3 << 14)   /* \brief dummy */
#define DW_MSGP4      14            /* \brief dummy */
#define DW_MSGP3_MASK (0x3 << 12)   /* \brief dummy */
#define DW_MSGP3      12            /* \brief dummy */
#define DW_MSGP2_MASK (0x3 << 10)   /* \brief dummy */
#define DW_MSGP2      10            /* \brief dummy */
#define DW_MSGP1_MASK (0x3 << 8)    /* \brief dummy */
#define DW_MSGP1      8             /* \brief dummy */
#define DW_MSGP0_MASK (0x3 << 6)    /* \brief dummy */
#define DW_MSGP0      6             /* \brief dummy */

/* DW_REG_GPIO_CTRL 0x26 */
/* DW_SUBREG_GPIO_DIR  0x26:08 */
#define DW_GDP0        0          /* \brief dummy */
#define DW_GDP1        1          /* \brief dummy */
#define DW_GDP2        2          /* \brief dummy */
#define DW_GDP3        3          /* \brief dummy */
#define DW_GDP4        8          /* \brief dummy */
#define DW_GDP5        9          /* \brief dummy */
#define DW_GDP6        10         /* \brief dummy */
#define DW_GDP7        11         /* \brief dummy */
#define DW_GDP8        16         /* \brief dummy */
#define DW_GDM0        4          /* \brief dummy */
#define DW_GDM1        5          /* \brief dummy */
#define DW_GDM2        6          /* \brief dummy */
#define DW_GDM3        7          /* \brief dummy */
#define DW_GDM4        12         /* \brief dummy */
#define DW_GDM5        13         /* \brief dummy */
#define DW_GDM6        14         /* \brief dummy */
#define DW_GDM7        15         /* \brief dummy */
#define DW_GDM8        20         /* \brief dummy */

/* DW_REG_GPIO_CTRL 0x26 */
/* DW_REG_DW_SUBREG_GPIO_DOUT 0x26:0C */
#define DW_GOP0        0          /* \brief dummy */
#define DW_GOP1        1          /* \brief dummy */
#define DW_GOP2        2          /* \brief dummy */
#define DW_GOP3        3          /* \brief dummy */
#define DW_GOP4        8          /* \brief dummy */
#define DW_GOP5        9          /* \brief dummy */
#define DW_GOP6        10         /* \brief dummy */
#define DW_GOP7        11         /* \brief dummy */
#define DW_GOP8        16         /* \brief dummy */
#define DW_GOM0        4          /* \brief dummy */
#define DW_GOM1        5          /* \brief dummy */
#define DW_GOM2        6          /* \brief dummy */
#define DW_GOM3        7          /* \brief dummy */
#define DW_GOM4        12         /* \brief dummy */
#define DW_GOM5        13         /* \brief dummy */
#define DW_GOM6        14         /* \brief dummy */
#define DW_GOM7        15         /* \brief dummy */
#define DW_GOM8        20         /* \brief dummy */

/* DW_REG_TX_CAL 0x2A */
/* DW_SUBREG_SARC 0x2A:00 */
#define DW_SAR_CTRL       0             /* \brief dummy */
#define DW_SAR_CTRL_MASK  (0x1UL << 0)  /* \brief dummy */
/* DW_SUBREG_SARL 0x2A:03 */
#define DW_SAR_LTEMP      0             /* \brief dummy */
#define DW_SAR_LTEMP_MASK (0xFFUL << 0) /* \brief dummy */
#define DW_SAR_LVBAT      8             /* \brief dummy */
#define DW_SAR_LVBAT_MASK (0xFFUL << 8) /* \brief dummy */
/* DW_SUBREG_SARW 0x2A:06 */
#define DW_SAR_WTEMP      0             /* \brief dummy */
#define DW_SAR_WTEMP_MASK (0xFFUL << 0) /* \brief dummy */
#define DW_SAR_WVBAT      8             /* \brief dummy */
#define DW_SAR_WVBAT_MASK (0xFFUL << 8) /* \brief dummy */

/* DW_REG_FS_CTRL       0x2B */
/* DW_SUBREG_FS_XTALT   0x0E */
#define DW_FS_XTAL_RESERVED_MASK (0x60) /* \brief dummy */
#define DW_XTALT          0             /* \brief dummy */
#define DW_XTALT_MASK     (0x1FUL << 0) /* \brief dummy */

/* DW_REG_AON_WCFG      0x2C */
/* DW_SUBREG_AON_WCFG   0x2C:00 */
#define DW_ONW_LLDE      11               /* \brief dummy */
#define DW_ONW_LLDE_MASK (0x1UL << 11)  /* \brief dummy */


/* DW_REG_OTP_IF 0x2D */
/* DW_SUBREG_OTP_WDAT  0x2D:00 */
/* DW_SUBREG_OTP_ADDR  0x2D:04 */
#define DW_OTPADDR      0               /* \brief dummy */
#define DW_OTPADDR_MASK (0x7FFUL << 0)  /* \brief dummy */
/* DW_SUBREG_OTP_CTRL  0x2D:06 */
#define DW_OTPRDEN      0               /* \brief dummy */
#define DW_OTPRDEN_MASK (0x1UL << 0)    /* \brief dummy */
#define DW_OTPREAD      1               /* \brief dummy */
#define DW_OTPREAD_MASK (0x1UL << 1)    /* \brief dummy */
#define DW_OTPMRWR      3               /* \brief dummy */
#define DW_OTPMRWR_MASK (0x1UL << 3)    /* \brief dummy */
#define DW_OTPPROG      6               /* \brief dummy */
#define DW_OTPPROG_MASK (0x1UL << 6)    /* \brief dummy */
#define DW_OTPMR        7               /* \brief dummy */
#define DW_OTPMR_MASK   (0xFUL << 7)    /* \brief dummy */
#define DW_LDELOAD      15              /* \brief dummy */
#define DW_LDELOAD_MASK (0x1UL << 15)   /* \brief dummy */
/* DW_SUBREG_OTP_STAT  0x2D:08 */
#define DW_OTPPRGD      0               /* \brief dummy */
#define DW_OTPPRGD_MASK (0x1UL << 0)    /* \brief dummy */
#define DW_OTPVPOK      1               /* \brief dummy */
#define DW_OTPVPOK_MASK (0x1UL << 1)    /* \brief dummy */
/* DW_SUBREG_OTP_RDAT  0x2D:0A */
/* DW_SUBREG_OTP_SRDAT 0x2D:0E */
/* DW_SUBREG_OTP_SF    0x2D:12 */
#define DW_OPS_KICK      0              /* \brief dummy */
#define DW_OPS_KICK_MASK (0x1UL << 0)   /* \brief dummy */
#define DW_OPS_SEL       5              /* \brief dummy */
#define DW_OPS_SEL_MASK  (0x3UL << 5)   /* \brief dummy */

/* DW_SUBREG_LDE_CFG1        0x2E:0806 */
#define DW_NTM            0              /* \brief dummy */
#define DW_NTM_MASK       (0x1FUL << 0)   /* \brief dummy */
#define DW_PMULT          5              /* \brief dummy */
#define DW_PMULT_MASK     (0x7UL << 5)   /* \brief dummy */

/* DW_REG_EVC_CTRL      0x2F:00        */
#define DW_EVC_EN            0              /* \brief dummy */
#define DW_EVC_EN_MASK       (0x1UL << 0)   /* \brief dummy */
#define DW_EVC_CLR           1              /* \brief dummy */
#define DW_EVC_CLR_MASK      (0x1UL << 1)   /* \brief dummy */
#define DW_REG_EVC_CTRL_MASK (0x3UL << 0)
/* DW_SUBREG_EVC_PHE    0x2F:04         */
#define DW_EVC_PHE            0              /* \brief dummy */
#define DW_EVC_PHE_MASK       (0xFFFUL << 0) /* \brief dummy */
/* DW_SUBREG_EVC_RSE    0x2F:06         */
#define DW_EVC_RSE            0              /* \brief dummy */
#define DW_EVC_RSE_MASK       (0xFFFUL << 0) /* \brief dummy */
/* DW_SUBREG_EVC_FCG    0x2F:08         */
#define DW_EVC_FCG            0              /* \brief dummy */
#define DW_EVC_FCG_MASK       (0xFFFUL << 0) /* \brief dummy */
/* DW_SUBREG_EVC_FCE    0x2F:0A         */
#define DW_EVC_FCE            0              /* \brief dummy */
#define DW_EVC_FCE_MASK       (0xFFFUL << 0) /* \brief dummy */
/* DW_SUBREG_EVC_FFR    0x2F:0C         */
#define DW_EVC_FFR            0              /* \brief dummy */
#define DW_EVC_FFR_MASK       (0xFFFUL << 0) /* \brief dummy */
/* DW_SUBREG_EVC_OVR    0x2F:0E         */
#define DW_EVC_OVR            0              /* \brief dummy */
#define DW_EVC_OVR_MASK       (0xFFFUL << 0) /* \brief dummy */
/* DW_SUBREG_EVC_STO    0x2F:10         */
#define DW_EVC_STO            0              /* \brief dummy */
#define DW_EVC_STO_MASK       (0xFFFUL << 0) /* \brief dummy */
/* DW_SUBREG_EVC_PTO    0x2F:12         */
#define DW_EVC_PTO            0              /* \brief dummy */
#define DW_EVC_PTO_MASK       (0xFFFUL << 0) /* \brief dummy */
/* DW_SUBREG_EVC_FWTO   0x2F:14         */
#define DW_EVC_FWTO           0              /* \brief dummy */
#define DW_EVC_FWTO_MASK      (0xFFFUL << 0) /* \brief dummy */
/* DW_SUBREG_EVC_TXFS   0x2F:14         */
#define DW_EVC_TXFS           0              /* \brief dummy */
#define DW_EVC_TXFS_MASK      (0xFFFUL << 0) /* \brief dummy */
/* DW_SUBREG_EVC_HPW    0x2F:18         */
#define DW_EVC_HPW            0              /* \brief dummy */
#define DW_EVC_HPW_MASK       (0xFFFUL << 0) /* \brief dummy */
/* DW_SUBREG_EVC_TPW    0x2F:1A         */
#define DW_EVC_TPW            0              /* \brief dummy */
#define DW_EVC_TPW_MASK       (0xFFFUL << 0) /* \brief dummy */

/* DW_REG_PMSC 0x36 */
/* DW_REG_PMSC_CTRL0 0x36:00 */
#define DW_SYS_CLKS          0              /* \brief dummy */
#define DW_SYS_CLKS_MASK     (0x3UL << 0)   /* \brief dummy */
#define DW_SYSCLKS           0              /* \brief dummy */
#define DW_SYSCLKS_MASK      (0x3UL << 0)   /* \brief dummy */
#define DW_RXCLKS            2              /* \brief dummy */
#define DW_RXCLKS_MASK       (0x3UL << 2)   /* \brief dummy */
#define DW_TXCLKS            4              /* \brief dummy */
#define DW_TXCLKS_MASK       (0x3UL << 4)   /* \brief dummy */
#define DW_ADCCE             10             /* \brief dummy */
#define DW_ADCCE_MASK        (0x1UL << 10)  /* \brief dummy */
#define DW_GPCE              16             /* \brief dummy */
#define DW_GPCE_MASK         (0x1UL << 16)  /* \brief dummy */
#define DW_GPRN              17             /* \brief dummy */
#define DW_GPRN_MASK         (0x1UL << 17)  /* \brief dummy */
#define DW_GPDCE             18             /* \brief dummy */
#define DW_GPDCE_MASK        (0x1UL << 18)  /* \brief dummy */
#define DW_KHZCLKEN          23             /* \brief dummy */
#define DW_KHZCLKEN_MASK     (0x1UL << 23)  /* \brief dummy */
#define DW_SOFTRESET         28             /* \brief dummy */
#define DW_SOFTRESET_MASK    (0xFUL << 28)  /* \brief dummy */

/* DW_REG_PMSC 0x36 */
/* DW_SUBREG_PMSC_LEDC  0x36:28 */
#define DW_BLINK_TIM         0              /* \brief dummy */
#define DW_BLINK_TIM_MASK    (0xFFUL << 0)  /* \brief dummy */
#define DW_BLNKEN            8              /* \brief dummy */
#define DW_BLNKEN_MASK       (0x01UL << 8)  /* \brief dummy */
#define DW_BLNKNOW           16             /* \brief dummy */
#define DW_BLNKNOW_MASK      (0xFUL << 16)  /* \brief dummy */

#endif /* DW1000_CONST_H */