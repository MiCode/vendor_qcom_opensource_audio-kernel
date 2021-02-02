/*
 * Copyright (c) 2018-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2021 XiaoMi, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _WCD937X_REGISTERS_H
#define _WCD937X_REGISTERS_H

#define WCD937X_BASE_ADDRESS 0x3000

#define WCD937X_REG(reg)  (reg - WCD937X_BASE_ADDRESS)

enum {
	REG_NO_ACCESS,
	RD_REG,
	WR_REG,
	RD_WR_REG
};

#define WCD937X_ANA_BIAS                            (WCD937X_BASE_ADDRESS+0x001)
#define WCD937X_ANA_RX_SUPPLIES                     (WCD937X_BASE_ADDRESS+0x008)
#define WCD937X_ANA_HPH                             (WCD937X_BASE_ADDRESS+0x009)
#define WCD937X_ANA_EAR                             (WCD937X_BASE_ADDRESS+0x00A)
#define WCD937X_ANA_EAR_COMPANDER_CTL               (WCD937X_BASE_ADDRESS+0x00B)
#define WCD937X_ANA_TX_CH1                          (WCD937X_BASE_ADDRESS+0x00E)
#define WCD937X_ANA_TX_CH2                          (WCD937X_BASE_ADDRESS+0x00F)
#define WCD937X_ANA_TX_CH3                          (WCD937X_BASE_ADDRESS+0x010)
#define WCD937X_ANA_TX_CH3_HPF                      (WCD937X_BASE_ADDRESS+0x011)
#define WCD937X_ANA_MICB1_MICB2_DSP_EN_LOGIC        (WCD937X_BASE_ADDRESS+0x012)
#define WCD937X_ANA_MICB3_DSP_EN_LOGIC              (WCD937X_BASE_ADDRESS+0x013)
#define WCD937X_ANA_MBHC_MECH                       (WCD937X_BASE_ADDRESS+0x014)
#define WCD937X_ANA_MBHC_ELECT                      (WCD937X_BASE_ADDRESS+0x015)
#define WCD937X_ANA_MBHC_ZDET                       (WCD937X_BASE_ADDRESS+0x016)
#define WCD937X_ANA_MBHC_RESULT_1                   (WCD937X_BASE_ADDRESS+0x017)
#define WCD937X_ANA_MBHC_RESULT_2                   (WCD937X_BASE_ADDRESS+0x018)
#define WCD937X_ANA_MBHC_RESULT_3                   (WCD937X_BASE_ADDRESS+0x019)
#define WCD937X_ANA_MBHC_BTN0                       (WCD937X_BASE_ADDRESS+0x01A)
#define WCD937X_ANA_MBHC_BTN1                       (WCD937X_BASE_ADDRESS+0x01B)
#define WCD937X_ANA_MBHC_BTN2                       (WCD937X_BASE_ADDRESS+0x01C)
#define WCD937X_ANA_MBHC_BTN3                       (WCD937X_BASE_ADDRESS+0x01D)
#define WCD937X_ANA_MBHC_BTN4                       (WCD937X_BASE_ADDRESS+0x01E)
#define WCD937X_ANA_MBHC_BTN5                       (WCD937X_BASE_ADDRESS+0x01F)
#define WCD937X_ANA_MBHC_BTN6                       (WCD937X_BASE_ADDRESS+0x020)
#define WCD937X_ANA_MBHC_BTN7                       (WCD937X_BASE_ADDRESS+0x021)
#define WCD937X_ANA_MICB1                           (WCD937X_BASE_ADDRESS+0x022)
#define WCD937X_ANA_MICB2                           (WCD937X_BASE_ADDRESS+0x023)
#define WCD937X_ANA_MICB2_RAMP                      (WCD937X_BASE_ADDRESS+0x024)
#define WCD937X_ANA_MICB3                           (WCD937X_BASE_ADDRESS+0x025)
#define WCD937X_BIAS_CTL                            (WCD937X_BASE_ADDRESS+0x028)
#define WCD937X_BIAS_VBG_FINE_ADJ                   (WCD937X_BASE_ADDRESS+0x029)
#define WCD937X_LDOL_VDDCX_ADJUST                   (WCD937X_BASE_ADDRESS+0x040)
#define WCD937X_LDOL_DISABLE_LDOL                   (WCD937X_BASE_ADDRESS+0x041)
#define WCD937X_MBHC_CTL_CLK                        (WCD937X_BASE_ADDRESS+0x056)
#define WCD937X_MBHC_CTL_ANA                        (WCD937X_BASE_ADDRESS+0x057)
#define WCD937X_MBHC_CTL_SPARE_1                    (WCD937X_BASE_ADDRESS+0x058)
#define WCD937X_MBHC_CTL_SPARE_2                    (WCD937X_BASE_ADDRESS+0x059)
#define WCD937X_MBHC_CTL_BCS                        (WCD937X_BASE_ADDRESS+0x05A)
#define WCD937X_MBHC_MOISTURE_DET_FSM_STATUS        (WCD937X_BASE_ADDRESS+0x05B)
#define WCD937X_MBHC_TEST_CTL                       (WCD937X_BASE_ADDRESS+0x05C)
#define WCD937X_LDOH_MODE                           (WCD937X_BASE_ADDRESS+0x067)
#define WCD937X_LDOH_BIAS                           (WCD937X_BASE_ADDRESS+0x068)
#define WCD937X_LDOH_STB_LOADS                      (WCD937X_BASE_ADDRESS+0x069)
#define WCD937X_LDOH_SLOWRAMP                       (WCD937X_BASE_ADDRESS+0x06A)
#define WCD937X_MICB1_TEST_CTL_1                    (WCD937X_BASE_ADDRESS+0x06B)
#define WCD937X_MICB1_TEST_CTL_2                    (WCD937X_BASE_ADDRESS+0x06C)
#define WCD937X_MICB1_TEST_CTL_3                    (WCD937X_BASE_ADDRESS+0x06D)
#define WCD937X_MICB2_TEST_CTL_1                    (WCD937X_BASE_ADDRESS+0x06E)
#define WCD937X_MICB2_TEST_CTL_2                    (WCD937X_BASE_ADDRESS+0x06F)
#define WCD937X_MICB2_TEST_CTL_3                    (WCD937X_BASE_ADDRESS+0x070)
#define WCD937X_MICB3_TEST_CTL_1                    (WCD937X_BASE_ADDRESS+0x071)
#define WCD937X_MICB3_TEST_CTL_2                    (WCD937X_BASE_ADDRESS+0x072)
#define WCD937X_MICB3_TEST_CTL_3                    (WCD937X_BASE_ADDRESS+0x073)
#define WCD937X_TX_COM_ADC_VCM                      (WCD937X_BASE_ADDRESS+0x077)
#define WCD937X_TX_COM_BIAS_ATEST                   (WCD937X_BASE_ADDRESS+0x078)
#define WCD937X_TX_COM_ADC_INT1_IB                  (WCD937X_BASE_ADDRESS+0x079)
#define WCD937X_TX_COM_ADC_INT2_IB                  (WCD937X_BASE_ADDRESS+0x07A)
#define WCD937X_TX_COM_TXFE_DIV_CTL                 (WCD937X_BASE_ADDRESS+0x07B)
#define WCD937X_TX_COM_TXFE_DIV_START               (WCD937X_BASE_ADDRESS+0x07C)
#define WCD937X_TX_COM_TXFE_DIV_STOP_9P6M           (WCD937X_BASE_ADDRESS+0x07D)
#define WCD937X_TX_COM_TXFE_DIV_STOP_12P288M        (WCD937X_BASE_ADDRESS+0x07E)
#define WCD937X_TX_1_2_TEST_EN                      (WCD937X_BASE_ADDRESS+0x07F)
#define WCD937X_TX_1_2_ADC_IB                       (WCD937X_BASE_ADDRESS+0x080)
#define WCD937X_TX_1_2_ATEST_REFCTL                 (WCD937X_BASE_ADDRESS+0x081)
#define WCD937X_TX_1_2_TEST_CTL                     (WCD937X_BASE_ADDRESS+0x082)
#define WCD937X_TX_1_2_TEST_BLK_EN                  (WCD937X_BASE_ADDRESS+0x083)
#define WCD937X_TX_1_2_TXFE_CLKDIV                  (WCD937X_BASE_ADDRESS+0x084)
#define WCD937X_TX_1_2_SAR2_ERR                     (WCD937X_BASE_ADDRESS+0x085)
#define WCD937X_TX_1_2_SAR1_ERR                     (WCD937X_BASE_ADDRESS+0x086)
#define WCD937X_TX_3_TEST_EN                        (WCD937X_BASE_ADDRESS+0x087)
#define WCD937X_TX_3_ADC_IB                         (WCD937X_BASE_ADDRESS+0x088)
#define WCD937X_TX_3_ATEST_REFCTL                   (WCD937X_BASE_ADDRESS+0x089)
#define WCD937X_TX_3_TEST_CTL                       (WCD937X_BASE_ADDRESS+0x08A)
#define WCD937X_TX_3_TEST_BLK_EN                    (WCD937X_BASE_ADDRESS+0x08B)
#define WCD937X_TX_3_TXFE_CLKDIV                    (WCD937X_BASE_ADDRESS+0x08C)
#define WCD937X_TX_3_SPARE_MONO                     (WCD937X_BASE_ADDRESS+0x08D)
#define WCD937X_TX_3_SAR1_ERR                       (WCD937X_BASE_ADDRESS+0x08E)
#define WCD937X_CLASSH_MODE_1                       (WCD937X_BASE_ADDRESS+0x097)
#define WCD937X_CLASSH_MODE_2                       (WCD937X_BASE_ADDRESS+0x098)
#define WCD937X_CLASSH_MODE_3                       (WCD937X_BASE_ADDRESS+0x099)
#define WCD937X_CLASSH_CTRL_VCL_1                   (WCD937X_BASE_ADDRESS+0x09A)
#define WCD937X_CLASSH_CTRL_VCL_2                   (WCD937X_BASE_ADDRESS+0x09B)
#define WCD937X_CLASSH_CTRL_CCL_1                   (WCD937X_BASE_ADDRESS+0x09C)
#define WCD937X_CLASSH_CTRL_CCL_2                   (WCD937X_BASE_ADDRESS+0x09D)
#define WCD937X_CLASSH_CTRL_CCL_3                   (WCD937X_BASE_ADDRESS+0x09E)
#define WCD937X_CLASSH_CTRL_CCL_4                   (WCD937X_BASE_ADDRESS+0x09F)
#define WCD937X_CLASSH_CTRL_CCL_5                   (WCD937X_BASE_ADDRESS+0x0A0)
#define WCD937X_CLASSH_BUCK_TMUX_A_D                (WCD937X_BASE_ADDRESS+0x0A1)
#define WCD937X_CLASSH_BUCK_SW_DRV_CNTL             (WCD937X_BASE_ADDRESS+0x0A2)
#define WCD937X_CLASSH_SPARE                        (WCD937X_BASE_ADDRESS+0x0A3)
#define WCD937X_FLYBACK_EN                          (WCD937X_BASE_ADDRESS+0x0A4)
#define WCD937X_FLYBACK_VNEG_CTRL_1                 (WCD937X_BASE_ADDRESS+0x0A5)
#define WCD937X_FLYBACK_VNEG_CTRL_2                 (WCD937X_BASE_ADDRESS+0x0A6)
#define WCD937X_FLYBACK_VNEG_CTRL_3                 (WCD937X_BASE_ADDRESS+0x0A7)
#define WCD937X_FLYBACK_VNEG_CTRL_4                 (WCD937X_BASE_ADDRESS+0x0A8)
#define WCD937X_FLYBACK_VNEG_CTRL_5                 (WCD937X_BASE_ADDRESS+0x0A9)
#define WCD937X_FLYBACK_VNEG_CTRL_6                 (WCD937X_BASE_ADDRESS+0x0AA)
#define WCD937X_FLYBACK_VNEG_CTRL_7                 (WCD937X_BASE_ADDRESS+0x0AB)
#define WCD937X_FLYBACK_VNEG_CTRL_8                 (WCD937X_BASE_ADDRESS+0x0AC)
#define WCD937X_FLYBACK_VNEG_CTRL_9                 (WCD937X_BASE_ADDRESS+0x0AD)
#define WCD937X_FLYBACK_VNEGDAC_CTRL_1              (WCD937X_BASE_ADDRESS+0x0AE)
#define WCD937X_FLYBACK_VNEGDAC_CTRL_2              (WCD937X_BASE_ADDRESS+0x0AF)
#define WCD937X_FLYBACK_VNEGDAC_CTRL_3              (WCD937X_BASE_ADDRESS+0x0B0)
#define WCD937X_FLYBACK_CTRL_1                      (WCD937X_BASE_ADDRESS+0x0B1)
#define WCD937X_FLYBACK_TEST_CTL                    (WCD937X_BASE_ADDRESS+0x0B2)
#define WCD937X_RX_AUX_SW_CTL                       (WCD937X_BASE_ADDRESS+0x0B3)
#define WCD937X_RX_PA_AUX_IN_CONN                   (WCD937X_BASE_ADDRESS+0x0B4)
#define WCD937X_RX_TIMER_DIV                        (WCD937X_BASE_ADDRESS+0x0B5)
#define WCD937X_RX_OCP_CTL                          (WCD937X_BASE_ADDRESS+0x0B6)
#define WCD937X_RX_OCP_COUNT                        (WCD937X_BASE_ADDRESS+0x0B7)
#define WCD937X_RX_BIAS_EAR_DAC                     (WCD937X_BASE_ADDRESS+0x0B8)
#define WCD937X_RX_BIAS_EAR_AMP                     (WCD937X_BASE_ADDRESS+0x0B9)
#define WCD937X_RX_BIAS_HPH_LDO                     (WCD937X_BASE_ADDRESS+0x0BA)
#define WCD937X_RX_BIAS_HPH_PA                      (WCD937X_BASE_ADDRESS+0x0BB)
#define WCD937X_RX_BIAS_HPH_RDACBUFF_CNP2           (WCD937X_BASE_ADDRESS+0x0BC)
#define WCD937X_RX_BIAS_HPH_RDAC_LDO                (WCD937X_BASE_ADDRESS+0x0BD)
#define WCD937X_RX_BIAS_HPH_CNP1                    (WCD937X_BASE_ADDRESS+0x0BE)
#define WCD937X_RX_BIAS_HPH_LOWPOWER                (WCD937X_BASE_ADDRESS+0x0BF)
#define WCD937X_RX_BIAS_AUX_DAC                     (WCD937X_BASE_ADDRESS+0x0C0)
#define WCD937X_RX_BIAS_AUX_AMP                     (WCD937X_BASE_ADDRESS+0x0C1)
#define WCD937X_RX_BIAS_VNEGDAC_BLEEDER             (WCD937X_BASE_ADDRESS+0x0C2)
#define WCD937X_RX_BIAS_MISC                        (WCD937X_BASE_ADDRESS+0x0C3)
#define WCD937X_RX_BIAS_BUCK_RST                    (WCD937X_BASE_ADDRESS+0x0C4)
#define WCD937X_RX_BIAS_BUCK_VREF_ERRAMP            (WCD937X_BASE_ADDRESS+0x0C5)
#define WCD937X_RX_BIAS_FLYB_ERRAMP                 (WCD937X_BASE_ADDRESS+0x0C6)
#define WCD937X_RX_BIAS_FLYB_BUFF                   (WCD937X_BASE_ADDRESS+0x0C7)
#define WCD937X_RX_BIAS_FLYB_MID_RST                (WCD937X_BASE_ADDRESS+0x0C8)
#define WCD937X_HPH_L_STATUS                        (WCD937X_BASE_ADDRESS+0x0C9)
#define WCD937X_HPH_R_STATUS                        (WCD937X_BASE_ADDRESS+0x0CA)
#define WCD937X_HPH_CNP_EN                          (WCD937X_BASE_ADDRESS+0x0CB)
#define WCD937X_HPH_CNP_WG_CTL                      (WCD937X_BASE_ADDRESS+0x0CC)
#define WCD937X_HPH_CNP_WG_TIME                     (WCD937X_BASE_ADDRESS+0x0CD)
#define WCD937X_HPH_OCP_CTL                         (WCD937X_BASE_ADDRESS+0x0CE)
#define WCD937X_HPH_AUTO_CHOP                       (WCD937X_BASE_ADDRESS+0x0CF)
#define WCD937X_HPH_CHOP_CTL                        (WCD937X_BASE_ADDRESS+0x0D0)
#define WCD937X_HPH_PA_CTL1                         (WCD937X_BASE_ADDRESS+0x0D1)
#define WCD937X_HPH_PA_CTL2                         (WCD937X_BASE_ADDRESS+0x0D2)
#define WCD937X_HPH_L_EN                            (WCD937X_BASE_ADDRESS+0x0D3)
#define WCD937X_HPH_L_TEST                          (WCD937X_BASE_ADDRESS+0x0D4)
#define WCD937X_HPH_L_ATEST                         (WCD937X_BASE_ADDRESS+0x0D5)
#define WCD937X_HPH_R_EN                            (WCD937X_BASE_ADDRESS+0x0D6)
#define WCD937X_HPH_R_TEST                          (WCD937X_BASE_ADDRESS+0x0D7)
#define WCD937X_HPH_R_ATEST                         (WCD937X_BASE_ADDRESS+0x0D8)
#define WCD937X_HPH_RDAC_CLK_CTL1                   (WCD937X_BASE_ADDRESS+0x0D9)
#define WCD937X_HPH_RDAC_CLK_CTL2                   (WCD937X_BASE_ADDRESS+0x0DA)
#define WCD937X_HPH_RDAC_LDO_CTL                    (WCD937X_BASE_ADDRESS+0x0DB)
#define WCD937X_HPH_RDAC_CHOP_CLK_LP_CTL            (WCD937X_BASE_ADDRESS+0x0DC)
#define WCD937X_HPH_REFBUFF_UHQA_CTL                (WCD937X_BASE_ADDRESS+0x0DD)
#define WCD937X_HPH_REFBUFF_LP_CTL                  (WCD937X_BASE_ADDRESS+0x0DE)
#define WCD937X_HPH_L_DAC_CTL                       (WCD937X_BASE_ADDRESS+0x0DF)
#define WCD937X_HPH_R_DAC_CTL                       (WCD937X_BASE_ADDRESS+0x0E0)
#define WCD937X_HPH_SURGE_HPHLR_SURGE_COMP_SEL      (WCD937X_BASE_ADDRESS+0x0E1)
#define WCD937X_HPH_SURGE_HPHLR_SURGE_EN            (WCD937X_BASE_ADDRESS+0x0E2)
#define WCD937X_HPH_SURGE_HPHLR_SURGE_MISC1         (WCD937X_BASE_ADDRESS+0x0E3)
#define WCD937X_HPH_SURGE_HPHLR_SURGE_STATUS        (WCD937X_BASE_ADDRESS+0x0E4)
#define WCD937X_EAR_EAR_EN_REG                      (WCD937X_BASE_ADDRESS+0x0E9)
#define WCD937X_EAR_EAR_PA_CON                      (WCD937X_BASE_ADDRESS+0x0EA)
#define WCD937X_EAR_EAR_SP_CON                      (WCD937X_BASE_ADDRESS+0x0EB)
#define WCD937X_EAR_EAR_DAC_CON                     (WCD937X_BASE_ADDRESS+0x0EC)
#define WCD937X_EAR_EAR_CNP_FSM_CON                 (WCD937X_BASE_ADDRESS+0x0ED)
#define WCD937X_EAR_TEST_CTL                        (WCD937X_BASE_ADDRESS+0x0EE)
#define WCD937X_EAR_STATUS_REG_1                    (WCD937X_BASE_ADDRESS+0x0EF)
#define WCD937X_EAR_STATUS_REG_2                    (WCD937X_BASE_ADDRESS+0x0F0)
#define WCD937X_ANA_NEW_PAGE_REGISTER               (WCD937X_BASE_ADDRESS+0x100)
#define WCD937X_HPH_NEW_ANA_HPH2                    (WCD937X_BASE_ADDRESS+0x101)
#define WCD937X_HPH_NEW_ANA_HPH3                    (WCD937X_BASE_ADDRESS+0x102)
#define WCD937X_SLEEP_CTL                           (WCD937X_BASE_ADDRESS+0x103)
#define WCD937X_SLEEP_WATCHDOG_CTL                  (WCD937X_BASE_ADDRESS+0x104)
#define WCD937X_MBHC_NEW_ELECT_REM_CLAMP_CTL        (WCD937X_BASE_ADDRESS+0x11F)
#define WCD937X_MBHC_NEW_CTL_1                      (WCD937X_BASE_ADDRESS+0x120)
#define WCD937X_MBHC_NEW_CTL_2                      (WCD937X_BASE_ADDRESS+0x121)
#define WCD937X_MBHC_NEW_PLUG_DETECT_CTL            (WCD937X_BASE_ADDRESS+0x122)
#define WCD937X_MBHC_NEW_ZDET_ANA_CTL               (WCD937X_BASE_ADDRESS+0x123)
#define WCD937X_MBHC_NEW_ZDET_RAMP_CTL              (WCD937X_BASE_ADDRESS+0x124)
#define WCD937X_MBHC_NEW_FSM_STATUS                 (WCD937X_BASE_ADDRESS+0x125)
#define WCD937X_MBHC_NEW_ADC_RESULT                 (WCD937X_BASE_ADDRESS+0x126)
#define WCD937X_TX_NEW_TX_CH2_SEL                   (WCD937X_BASE_ADDRESS+0x127)
#define WCD937X_AUX_AUXPA                           (WCD937X_BASE_ADDRESS+0x128)
#define WCD937X_LDORXTX_MODE                        (WCD937X_BASE_ADDRESS+0x129)
#define WCD937X_LDORXTX_CONFIG                      (WCD937X_BASE_ADDRESS+0x12A)
#define WCD937X_DIE_CRACK_DIE_CRK_DET_EN            (WCD937X_BASE_ADDRESS+0x12C)
#define WCD937X_DIE_CRACK_DIE_CRK_DET_OUT           (WCD937X_BASE_ADDRESS+0x12D)
#define WCD937X_HPH_NEW_INT_RDAC_GAIN_CTL           (WCD937X_BASE_ADDRESS+0x132)
#define WCD937X_HPH_NEW_INT_RDAC_HD2_CTL_L          (WCD937X_BASE_ADDRESS+0x133)
#define WCD937X_HPH_NEW_INT_RDAC_VREF_CTL           (WCD937X_BASE_ADDRESS+0x134)
#define WCD937X_HPH_NEW_INT_RDAC_OVERRIDE_CTL       (WCD937X_BASE_ADDRESS+0x135)
#define WCD937X_HPH_NEW_INT_RDAC_HD2_CTL_R          (WCD937X_BASE_ADDRESS+0x136)
#define WCD937X_HPH_NEW_INT_PA_MISC1                (WCD937X_BASE_ADDRESS+0x137)
#define WCD937X_HPH_NEW_INT_PA_MISC2                (WCD937X_BASE_ADDRESS+0x138)
#define WCD937X_HPH_NEW_INT_PA_RDAC_MISC            (WCD937X_BASE_ADDRESS+0x139)
#define WCD937X_HPH_NEW_INT_HPH_TIMER1              (WCD937X_BASE_ADDRESS+0x13A)
#define WCD937X_HPH_NEW_INT_HPH_TIMER2              (WCD937X_BASE_ADDRESS+0x13B)
#define WCD937X_HPH_NEW_INT_HPH_TIMER3              (WCD937X_BASE_ADDRESS+0x13C)
#define WCD937X_HPH_NEW_INT_HPH_TIMER4              (WCD937X_BASE_ADDRESS+0x13D)
#define WCD937X_HPH_NEW_INT_PA_RDAC_MISC2           (WCD937X_BASE_ADDRESS+0x13E)
#define WCD937X_HPH_NEW_INT_PA_RDAC_MISC3           (WCD937X_BASE_ADDRESS+0x13F)
#define WCD937X_RX_NEW_INT_HPH_RDAC_BIAS_LOHIFI     (WCD937X_BASE_ADDRESS+0x145)
#define WCD937X_RX_NEW_INT_HPH_RDAC_BIAS_ULP        (WCD937X_BASE_ADDRESS+0x146)
#define WCD937X_RX_NEW_INT_HPH_RDAC_LDO_LP          (WCD937X_BASE_ADDRESS+0x147)
#define WCD937X_MBHC_NEW_INT_MOISTURE_DET_DC_CTRL   (WCD937X_BASE_ADDRESS+0x1AF)
#define WCD937X_MBHC_NEW_INT_MOISTURE_DET_POLLING_CTRL \
						    (WCD937X_BASE_ADDRESS+0x1B0)
#define WCD937X_MBHC_NEW_INT_MECH_DET_CURRENT       (WCD937X_BASE_ADDRESS+0x1B1)
#define WCD937X_MBHC_NEW_INT_SPARE_2                (WCD937X_BASE_ADDRESS+0x1B2)
#define WCD937X_EAR_INT_NEW_EAR_CHOPPER_CON         (WCD937X_BASE_ADDRESS+0x1B7)
#define WCD937X_EAR_INT_NEW_CNP_VCM_CON1            (WCD937X_BASE_ADDRESS+0x1B8)
#define WCD937X_EAR_INT_NEW_CNP_VCM_CON2            (WCD937X_BASE_ADDRESS+0x1B9)
#define WCD937X_EAR_INT_NEW_EAR_DYNAMIC_BIAS        (WCD937X_BASE_ADDRESS+0x1BA)
#define WCD937X_AUX_INT_EN_REG                      (WCD937X_BASE_ADDRESS+0x1BD)
#define WCD937X_AUX_INT_PA_CTRL                     (WCD937X_BASE_ADDRESS+0x1BE)
#define WCD937X_AUX_INT_SP_CTRL                     (WCD937X_BASE_ADDRESS+0x1BF)
#define WCD937X_AUX_INT_DAC_CTRL                    (WCD937X_BASE_ADDRESS+0x1C0)
#define WCD937X_AUX_INT_CLK_CTRL                    (WCD937X_BASE_ADDRESS+0x1C1)
#define WCD937X_AUX_INT_TEST_CTRL                   (WCD937X_BASE_ADDRESS+0x1C2)
#define WCD937X_AUX_INT_STATUS_REG                  (WCD937X_BASE_ADDRESS+0x1C3)
#define WCD937X_AUX_INT_MISC                        (WCD937X_BASE_ADDRESS+0x1C4)
#define WCD937X_LDORXTX_INT_BIAS                    (WCD937X_BASE_ADDRESS+0x1C5)
#define WCD937X_LDORXTX_INT_STB_LOADS_DTEST         (WCD937X_BASE_ADDRESS+0x1C6)
#define WCD937X_LDORXTX_INT_TEST0                   (WCD937X_BASE_ADDRESS+0x1C7)
#define WCD937X_LDORXTX_INT_STARTUP_TIMER           (WCD937X_BASE_ADDRESS+0x1C8)
#define WCD937X_LDORXTX_INT_TEST1                   (WCD937X_BASE_ADDRESS+0x1C9)
#define WCD937X_LDORXTX_INT_STATUS                  (WCD937X_BASE_ADDRESS+0x1CA)
#define WCD937X_SLEEP_INT_WATCHDOG_CTL_1            (WCD937X_BASE_ADDRESS+0x1D0)
#define WCD937X_SLEEP_INT_WATCHDOG_CTL_2            (WCD937X_BASE_ADDRESS+0x1D1)
#define WCD937X_DIE_CRACK_INT_DIE_CRK_DET_INT1      (WCD937X_BASE_ADDRESS+0x1D3)
#define WCD937X_DIE_CRACK_INT_DIE_CRK_DET_INT2      (WCD937X_BASE_ADDRESS+0x1D4)
#define WCD937X_DIGITAL_PAGE_REGISTER               (WCD937X_BASE_ADDRESS+0x400)
#define WCD937X_DIGITAL_CHIP_ID0                    (WCD937X_BASE_ADDRESS+0x401)
#define WCD937X_DIGITAL_CHIP_ID1                    (WCD937X_BASE_ADDRESS+0x402)
#define WCD937X_DIGITAL_CHIP_ID2                    (WCD937X_BASE_ADDRESS+0x403)
#define WCD937X_DIGITAL_CHIP_ID3                    (WCD937X_BASE_ADDRESS+0x404)
#define WCD937X_DIGITAL_CDC_RST_CTL                 (WCD937X_BASE_ADDRESS+0x406)
#define WCD937X_DIGITAL_TOP_CLK_CFG                 (WCD937X_BASE_ADDRESS+0x407)
#define WCD937X_DIGITAL_CDC_ANA_CLK_CTL             (WCD937X_BASE_ADDRESS+0x408)
#define WCD937X_DIGITAL_CDC_DIG_CLK_CTL             (WCD937X_BASE_ADDRESS+0x409)
#define WCD937X_DIGITAL_SWR_RST_EN                  (WCD937X_BASE_ADDRESS+0x40A)
#define WCD937X_DIGITAL_CDC_PATH_MODE               (WCD937X_BASE_ADDRESS+0x40B)
#define WCD937X_DIGITAL_CDC_RX_RST                  (WCD937X_BASE_ADDRESS+0x40C)
#define WCD937X_DIGITAL_CDC_RX0_CTL                 (WCD937X_BASE_ADDRESS+0x40D)
#define WCD937X_DIGITAL_CDC_RX1_CTL                 (WCD937X_BASE_ADDRESS+0x40E)
#define WCD937X_DIGITAL_CDC_RX2_CTL                 (WCD937X_BASE_ADDRESS+0x40F)
#define WCD937X_DIGITAL_DEM_BYPASS_DATA0            (WCD937X_BASE_ADDRESS+0x410)
#define WCD937X_DIGITAL_DEM_BYPASS_DATA1            (WCD937X_BASE_ADDRESS+0x411)
#define WCD937X_DIGITAL_DEM_BYPASS_DATA2            (WCD937X_BASE_ADDRESS+0x412)
#define WCD937X_DIGITAL_DEM_BYPASS_DATA3            (WCD937X_BASE_ADDRESS+0x413)
#define WCD937X_DIGITAL_CDC_COMP_CTL_0              (WCD937X_BASE_ADDRESS+0x414)
#define WCD937X_DIGITAL_CDC_RX_DELAY_CTL            (WCD937X_BASE_ADDRESS+0x417)
#define WCD937X_DIGITAL_CDC_HPH_DSM_A1_0            (WCD937X_BASE_ADDRESS+0x418)
#define WCD937X_DIGITAL_CDC_HPH_DSM_A1_1            (WCD937X_BASE_ADDRESS+0x419)
#define WCD937X_DIGITAL_CDC_HPH_DSM_A2_0            (WCD937X_BASE_ADDRESS+0x41A)
#define WCD937X_DIGITAL_CDC_HPH_DSM_A2_1            (WCD937X_BASE_ADDRESS+0x41B)
#define WCD937X_DIGITAL_CDC_HPH_DSM_A3_0            (WCD937X_BASE_ADDRESS+0x41C)
#define WCD937X_DIGITAL_CDC_HPH_DSM_A3_1            (WCD937X_BASE_ADDRESS+0x41D)
#define WCD937X_DIGITAL_CDC_HPH_DSM_A4_0            (WCD937X_BASE_ADDRESS+0x41E)
#define WCD937X_DIGITAL_CDC_HPH_DSM_A4_1            (WCD937X_BASE_ADDRESS+0x41F)
#define WCD937X_DIGITAL_CDC_HPH_DSM_A5_0            (WCD937X_BASE_ADDRESS+0x420)
#define WCD937X_DIGITAL_CDC_HPH_DSM_A5_1            (WCD937X_BASE_ADDRESS+0x421)
#define WCD937X_DIGITAL_CDC_HPH_DSM_A6_0            (WCD937X_BASE_ADDRESS+0x422)
#define WCD937X_DIGITAL_CDC_HPH_DSM_A7_0            (WCD937X_BASE_ADDRESS+0x423)
#define WCD937X_DIGITAL_CDC_HPH_DSM_C_0             (WCD937X_BASE_ADDRESS+0x424)
#define WCD937X_DIGITAL_CDC_HPH_DSM_C_1             (WCD937X_BASE_ADDRESS+0x425)
#define WCD937X_DIGITAL_CDC_HPH_DSM_C_2             (WCD937X_BASE_ADDRESS+0x426)
#define WCD937X_DIGITAL_CDC_HPH_DSM_C_3             (WCD937X_BASE_ADDRESS+0x427)
#define WCD937X_DIGITAL_CDC_HPH_DSM_R1              (WCD937X_BASE_ADDRESS+0x428)
#define WCD937X_DIGITAL_CDC_HPH_DSM_R2              (WCD937X_BASE_ADDRESS+0x429)
#define WCD937X_DIGITAL_CDC_HPH_DSM_R3              (WCD937X_BASE_ADDRESS+0x42A)
#define WCD937X_DIGITAL_CDC_HPH_DSM_R4              (WCD937X_BASE_ADDRESS+0x42B)
#define WCD937X_DIGITAL_CDC_HPH_DSM_R5              (WCD937X_BASE_ADDRESS+0x42C)
#define WCD937X_DIGITAL_CDC_HPH_DSM_R6              (WCD937X_BASE_ADDRESS+0x42D)
#define WCD937X_DIGITAL_CDC_HPH_DSM_R7              (WCD937X_BASE_ADDRESS+0x42E)
#define WCD937X_DIGITAL_CDC_AUX_DSM_A1_0            (WCD937X_BASE_ADDRESS+0x42F)
#define WCD937X_DIGITAL_CDC_AUX_DSM_A1_1            (WCD937X_BASE_ADDRESS+0x430)
#define WCD937X_DIGITAL_CDC_AUX_DSM_A2_0            (WCD937X_BASE_ADDRESS+0x431)
#define WCD937X_DIGITAL_CDC_AUX_DSM_A2_1            (WCD937X_BASE_ADDRESS+0x432)
#define WCD937X_DIGITAL_CDC_AUX_DSM_A3_0            (WCD937X_BASE_ADDRESS+0x433)
#define WCD937X_DIGITAL_CDC_AUX_DSM_A3_1            (WCD937X_BASE_ADDRESS+0x434)
#define WCD937X_DIGITAL_CDC_AUX_DSM_A4_0            (WCD937X_BASE_ADDRESS+0x435)
#define WCD937X_DIGITAL_CDC_AUX_DSM_A4_1            (WCD937X_BASE_ADDRESS+0x436)
#define WCD937X_DIGITAL_CDC_AUX_DSM_A5_0            (WCD937X_BASE_ADDRESS+0x437)
#define WCD937X_DIGITAL_CDC_AUX_DSM_A5_1            (WCD937X_BASE_ADDRESS+0x438)
#define WCD937X_DIGITAL_CDC_AUX_DSM_A6_0            (WCD937X_BASE_ADDRESS+0x439)
#define WCD937X_DIGITAL_CDC_AUX_DSM_A7_0            (WCD937X_BASE_ADDRESS+0x43A)
#define WCD937X_DIGITAL_CDC_AUX_DSM_C_0             (WCD937X_BASE_ADDRESS+0x43B)
#define WCD937X_DIGITAL_CDC_AUX_DSM_C_1             (WCD937X_BASE_ADDRESS+0x43C)
#define WCD937X_DIGITAL_CDC_AUX_DSM_C_2             (WCD937X_BASE_ADDRESS+0x43D)
#define WCD937X_DIGITAL_CDC_AUX_DSM_C_3             (WCD937X_BASE_ADDRESS+0x43E)
#define WCD937X_DIGITAL_CDC_AUX_DSM_R1              (WCD937X_BASE_ADDRESS+0x43F)
#define WCD937X_DIGITAL_CDC_AUX_DSM_R2              (WCD937X_BASE_ADDRESS+0x440)
#define WCD937X_DIGITAL_CDC_AUX_DSM_R3              (WCD937X_BASE_ADDRESS+0x441)
#define WCD937X_DIGITAL_CDC_AUX_DSM_R4              (WCD937X_BASE_ADDRESS+0x442)
#define WCD937X_DIGITAL_CDC_AUX_DSM_R5              (WCD937X_BASE_ADDRESS+0x443)
#define WCD937X_DIGITAL_CDC_AUX_DSM_R6              (WCD937X_BASE_ADDRESS+0x444)
#define WCD937X_DIGITAL_CDC_AUX_DSM_R7              (WCD937X_BASE_ADDRESS+0x445)
#define WCD937X_DIGITAL_CDC_HPH_GAIN_RX_0           (WCD937X_BASE_ADDRESS+0x446)
#define WCD937X_DIGITAL_CDC_HPH_GAIN_RX_1           (WCD937X_BASE_ADDRESS+0x447)
#define WCD937X_DIGITAL_CDC_HPH_GAIN_DSD_0          (WCD937X_BASE_ADDRESS+0x448)
#define WCD937X_DIGITAL_CDC_HPH_GAIN_DSD_1          (WCD937X_BASE_ADDRESS+0x449)
#define WCD937X_DIGITAL_CDC_HPH_GAIN_DSD_2          (WCD937X_BASE_ADDRESS+0x44A)
#define WCD937X_DIGITAL_CDC_AUX_GAIN_DSD_0          (WCD937X_BASE_ADDRESS+0x44B)
#define WCD937X_DIGITAL_CDC_AUX_GAIN_DSD_1          (WCD937X_BASE_ADDRESS+0x44C)
#define WCD937X_DIGITAL_CDC_AUX_GAIN_DSD_2          (WCD937X_BASE_ADDRESS+0x44D)
#define WCD937X_DIGITAL_CDC_HPH_GAIN_CTL            (WCD937X_BASE_ADDRESS+0x44E)
#define WCD937X_DIGITAL_CDC_AUX_GAIN_CTL            (WCD937X_BASE_ADDRESS+0x44F)
#define WCD937X_DIGITAL_CDC_EAR_PATH_CTL            (WCD937X_BASE_ADDRESS+0x450)
#define WCD937X_DIGITAL_CDC_SWR_CLH                 (WCD937X_BASE_ADDRESS+0x451)
#define WCD937X_DIGITAL_SWR_CLH_BYP                 (WCD937X_BASE_ADDRESS+0x452)
#define WCD937X_DIGITAL_CDC_TX0_CTL                 (WCD937X_BASE_ADDRESS+0x453)
#define WCD937X_DIGITAL_CDC_TX1_CTL                 (WCD937X_BASE_ADDRESS+0x454)
#define WCD937X_DIGITAL_CDC_TX2_CTL                 (WCD937X_BASE_ADDRESS+0x455)
#define WCD937X_DIGITAL_CDC_TX_RST                  (WCD937X_BASE_ADDRESS+0x456)
#define WCD937X_DIGITAL_CDC_REQ_CTL                 (WCD937X_BASE_ADDRESS+0x457)
#define WCD937X_DIGITAL_CDC_AMIC_CTL                (WCD937X_BASE_ADDRESS+0x45A)
#define WCD937X_DIGITAL_CDC_DMIC_CTL                (WCD937X_BASE_ADDRESS+0x45B)
#define WCD937X_DIGITAL_CDC_DMIC1_CTL               (WCD937X_BASE_ADDRESS+0x45C)
#define WCD937X_DIGITAL_CDC_DMIC2_CTL               (WCD937X_BASE_ADDRESS+0x45D)
#define WCD937X_DIGITAL_CDC_DMIC3_CTL               (WCD937X_BASE_ADDRESS+0x45E)
#define WCD937X_DIGITAL_EFUSE_CTL                   (WCD937X_BASE_ADDRESS+0x45F)
#define WCD937X_DIGITAL_EFUSE_PRG_CTL               (WCD937X_BASE_ADDRESS+0x460)
#define WCD937X_DIGITAL_EFUSE_TEST_CTL_0            (WCD937X_BASE_ADDRESS+0x461)
#define WCD937X_DIGITAL_EFUSE_TEST_CTL_1            (WCD937X_BASE_ADDRESS+0x462)
#define WCD937X_DIGITAL_EFUSE_T_DATA_0              (WCD937X_BASE_ADDRESS+0x463)
#define WCD937X_DIGITAL_EFUSE_T_DATA_1              (WCD937X_BASE_ADDRESS+0x464)
#define WCD937X_DIGITAL_PDM_WD_CTL0                 (WCD937X_BASE_ADDRESS+0x465)
#define WCD937X_DIGITAL_PDM_WD_CTL1                 (WCD937X_BASE_ADDRESS+0x466)
#define WCD937X_DIGITAL_PDM_WD_CTL2                 (WCD937X_BASE_ADDRESS+0x467)
#define WCD937X_DIGITAL_INTR_MODE                   (WCD937X_BASE_ADDRESS+0x46A)
#define WCD937X_DIGITAL_INTR_MASK_0                 (WCD937X_BASE_ADDRESS+0x46B)
#define WCD937X_DIGITAL_INTR_MASK_1                 (WCD937X_BASE_ADDRESS+0x46C)
#define WCD937X_DIGITAL_INTR_MASK_2                 (WCD937X_BASE_ADDRESS+0x46D)
#define WCD937X_DIGITAL_INTR_STATUS_0               (WCD937X_BASE_ADDRESS+0x46E)
#define WCD937X_DIGITAL_INTR_STATUS_1               (WCD937X_BASE_ADDRESS+0x46F)
#define WCD937X_DIGITAL_INTR_STATUS_2               (WCD937X_BASE_ADDRESS+0x470)
#define WCD937X_DIGITAL_INTR_CLEAR_0                (WCD937X_BASE_ADDRESS+0x471)
#define WCD937X_DIGITAL_INTR_CLEAR_1                (WCD937X_BASE_ADDRESS+0x472)
#define WCD937X_DIGITAL_INTR_CLEAR_2                (WCD937X_BASE_ADDRESS+0x473)
#define WCD937X_DIGITAL_INTR_LEVEL_0                (WCD937X_BASE_ADDRESS+0x474)
#define WCD937X_DIGITAL_INTR_LEVEL_1                (WCD937X_BASE_ADDRESS+0x475)
#define WCD937X_DIGITAL_INTR_LEVEL_2                (WCD937X_BASE_ADDRESS+0x476)
#define WCD937X_DIGITAL_INTR_SET_0                  (WCD937X_BASE_ADDRESS+0x477)
#define WCD937X_DIGITAL_INTR_SET_1                  (WCD937X_BASE_ADDRESS+0x478)
#define WCD937X_DIGITAL_INTR_SET_2                  (WCD937X_BASE_ADDRESS+0x479)
#define WCD937X_DIGITAL_INTR_TEST_0                 (WCD937X_BASE_ADDRESS+0x47A)
#define WCD937X_DIGITAL_INTR_TEST_1                 (WCD937X_BASE_ADDRESS+0x47B)
#define WCD937X_DIGITAL_INTR_TEST_2                 (WCD937X_BASE_ADDRESS+0x47C)
#define WCD937X_DIGITAL_CDC_CONN_RX0_CTL            (WCD937X_BASE_ADDRESS+0x47F)
#define WCD937X_DIGITAL_CDC_CONN_RX1_CTL            (WCD937X_BASE_ADDRESS+0x480)
#define WCD937X_DIGITAL_CDC_CONN_RX2_CTL            (WCD937X_BASE_ADDRESS+0x481)
#define WCD937X_DIGITAL_CDC_CONN_TX_CTL             (WCD937X_BASE_ADDRESS+0x482)
#define WCD937X_DIGITAL_LOOP_BACK_MODE              (WCD937X_BASE_ADDRESS+0x483)
#define WCD937X_DIGITAL_SWR_DAC_TEST                (WCD937X_BASE_ADDRESS+0x484)
#define WCD937X_DIGITAL_SWR_HM_TEST_RX_0            (WCD937X_BASE_ADDRESS+0x485)
#define WCD937X_DIGITAL_SWR_HM_TEST_TX_0            (WCD937X_BASE_ADDRESS+0x491)
#define WCD937X_DIGITAL_SWR_HM_TEST_RX_1            (WCD937X_BASE_ADDRESS+0x492)
#define WCD937X_DIGITAL_SWR_HM_TEST_TX_1            (WCD937X_BASE_ADDRESS+0x493)
#define WCD937X_DIGITAL_SWR_HM_TEST                 (WCD937X_BASE_ADDRESS+0x494)
#define WCD937X_DIGITAL_PAD_CTL_PDM_RX0             (WCD937X_BASE_ADDRESS+0x495)
#define WCD937X_DIGITAL_PAD_CTL_PDM_RX1             (WCD937X_BASE_ADDRESS+0x496)
#define WCD937X_DIGITAL_PAD_CTL_PDM_TX0             (WCD937X_BASE_ADDRESS+0x497)
#define WCD937X_DIGITAL_PAD_CTL_PDM_TX1             (WCD937X_BASE_ADDRESS+0x498)
#define WCD937X_DIGITAL_PAD_INP_DIS_0               (WCD937X_BASE_ADDRESS+0x499)
#define WCD937X_DIGITAL_PAD_INP_DIS_1               (WCD937X_BASE_ADDRESS+0x49A)
#define WCD937X_DIGITAL_DRIVE_STRENGTH_0            (WCD937X_BASE_ADDRESS+0x49B)
#define WCD937X_DIGITAL_DRIVE_STRENGTH_1            (WCD937X_BASE_ADDRESS+0x49C)
#define WCD937X_DIGITAL_DRIVE_STRENGTH_2            (WCD937X_BASE_ADDRESS+0x49D)
#define WCD937X_DIGITAL_RX_DATA_EDGE_CTL            (WCD937X_BASE_ADDRESS+0x49E)
#define WCD937X_DIGITAL_TX_DATA_EDGE_CTL            (WCD937X_BASE_ADDRESS+0x49F)
#define WCD937X_DIGITAL_GPIO_MODE                   (WCD937X_BASE_ADDRESS+0x4A0)
#define WCD937X_DIGITAL_PIN_CTL_OE                  (WCD937X_BASE_ADDRESS+0x4A1)
#define WCD937X_DIGITAL_PIN_CTL_DATA_0              (WCD937X_BASE_ADDRESS+0x4A2)
#define WCD937X_DIGITAL_PIN_CTL_DATA_1              (WCD937X_BASE_ADDRESS+0x4A3)
#define WCD937X_DIGITAL_PIN_STATUS_0                (WCD937X_BASE_ADDRESS+0x4A4)
#define WCD937X_DIGITAL_PIN_STATUS_1                (WCD937X_BASE_ADDRESS+0x4A5)
#define WCD937X_DIGITAL_DIG_DEBUG_CTL               (WCD937X_BASE_ADDRESS+0x4A6)
#define WCD937X_DIGITAL_DIG_DEBUG_EN                (WCD937X_BASE_ADDRESS+0x4A7)
#define WCD937X_DIGITAL_ANA_CSR_DBG_ADD             (WCD937X_BASE_ADDRESS+0x4A8)
#define WCD937X_DIGITAL_ANA_CSR_DBG_CTL             (WCD937X_BASE_ADDRESS+0x4A9)
#define WCD937X_DIGITAL_SSP_DBG                     (WCD937X_BASE_ADDRESS+0x4AA)
#define WCD937X_DIGITAL_MODE_STATUS_0               (WCD937X_BASE_ADDRESS+0x4AB)
#define WCD937X_DIGITAL_MODE_STATUS_1               (WCD937X_BASE_ADDRESS+0x4AC)
#define WCD937X_DIGITAL_SPARE_0                     (WCD937X_BASE_ADDRESS+0x4AD)
#define WCD937X_DIGITAL_SPARE_1                     (WCD937X_BASE_ADDRESS+0x4AE)
#define WCD937X_DIGITAL_SPARE_2                     (WCD937X_BASE_ADDRESS+0x4AF)
#define WCD937X_DIGITAL_EFUSE_REG_0                 (WCD937X_BASE_ADDRESS+0x4B0)
#define WCD937X_DIGITAL_EFUSE_REG_1                 (WCD937X_BASE_ADDRESS+0x4B1)
#define WCD937X_DIGITAL_EFUSE_REG_2                 (WCD937X_BASE_ADDRESS+0x4B2)
#define WCD937X_DIGITAL_EFUSE_REG_3                 (WCD937X_BASE_ADDRESS+0x4B3)
#define WCD937X_DIGITAL_EFUSE_REG_4                 (WCD937X_BASE_ADDRESS+0x4B4)
#define WCD937X_DIGITAL_EFUSE_REG_5                 (WCD937X_BASE_ADDRESS+0x4B5)
#define WCD937X_DIGITAL_EFUSE_REG_6                 (WCD937X_BASE_ADDRESS+0x4B6)
#define WCD937X_DIGITAL_EFUSE_REG_7                 (WCD937X_BASE_ADDRESS+0x4B7)
#define WCD937X_DIGITAL_EFUSE_REG_8                 (WCD937X_BASE_ADDRESS+0x4B8)
#define WCD937X_DIGITAL_EFUSE_REG_9                 (WCD937X_BASE_ADDRESS+0x4B9)
#define WCD937X_DIGITAL_EFUSE_REG_10                (WCD937X_BASE_ADDRESS+0x4BA)
#define WCD937X_DIGITAL_EFUSE_REG_11                (WCD937X_BASE_ADDRESS+0x4BB)
#define WCD937X_DIGITAL_EFUSE_REG_12                (WCD937X_BASE_ADDRESS+0x4BC)
#define WCD937X_DIGITAL_EFUSE_REG_13                (WCD937X_BASE_ADDRESS+0x4BD)
#define WCD937X_DIGITAL_EFUSE_REG_14                (WCD937X_BASE_ADDRESS+0x4BE)
#define WCD937X_DIGITAL_EFUSE_REG_15                (WCD937X_BASE_ADDRESS+0x4BF)
#define WCD937X_DIGITAL_EFUSE_REG_16                (WCD937X_BASE_ADDRESS+0x4C0)
#define WCD937X_DIGITAL_EFUSE_REG_17                (WCD937X_BASE_ADDRESS+0x4C1)
#define WCD937X_DIGITAL_EFUSE_REG_18                (WCD937X_BASE_ADDRESS+0x4C2)
#define WCD937X_DIGITAL_EFUSE_REG_19                (WCD937X_BASE_ADDRESS+0x4C3)
#define WCD937X_DIGITAL_EFUSE_REG_20                (WCD937X_BASE_ADDRESS+0x4C4)
#define WCD937X_DIGITAL_EFUSE_REG_21                (WCD937X_BASE_ADDRESS+0x4C5)
#define WCD937X_DIGITAL_EFUSE_REG_22                (WCD937X_BASE_ADDRESS+0x4C6)
#define WCD937X_DIGITAL_EFUSE_REG_23                (WCD937X_BASE_ADDRESS+0x4C7)
#define WCD937X_DIGITAL_EFUSE_REG_24                (WCD937X_BASE_ADDRESS+0x4C8)
#define WCD937X_DIGITAL_EFUSE_REG_25                (WCD937X_BASE_ADDRESS+0x4C9)
#define WCD937X_DIGITAL_EFUSE_REG_26                (WCD937X_BASE_ADDRESS+0x4CA)
#define WCD937X_DIGITAL_EFUSE_REG_27                (WCD937X_BASE_ADDRESS+0x4CB)
#define WCD937X_DIGITAL_EFUSE_REG_28                (WCD937X_BASE_ADDRESS+0x4CC)
#define WCD937X_DIGITAL_EFUSE_REG_29                (WCD937X_BASE_ADDRESS+0x4CD)
#define WCD937X_DIGITAL_EFUSE_REG_30                (WCD937X_BASE_ADDRESS+0x4CE)
#define WCD937X_DIGITAL_EFUSE_REG_31                (WCD937X_BASE_ADDRESS+0x4CF)

#define WCD937X_REGISTERS_MAX_SIZE (WCD937X_BASE_ADDRESS+0x4D0)
#define WCD937X_MAX_REGISTER (WCD937X_REGISTERS_MAX_SIZE - 1)

#endif
