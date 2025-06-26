/** Filename: tfa9865_tfaFieldnames_A2.h
 *  This file was generated automatically on 09/08/22 at 09:53:17. 
 *  Source file: TFA9865_GPA1_N1A2_I2C_RegisterMap.xlsx
 */

#ifndef _TFA9865_TFAFIELDNAMES_A2_H
#define _TFA9865_TFAFIELDNAMES_A2_H


#define TFA9865A2_I2CVERSION    3

typedef enum tfa9865A2BfEnumList {
    TFA9865A2_BF_PWDN  = 0x0000,    /*!< Powerdown selection                                */
    TFA9865A2_BF_I2CR  = 0x0010,    /*!< I2C reset - auto clear                             */
    TFA9865A2_BF_AMPE  = 0x0030,    /*!< Activate amplifier                                 */
    TFA9865A2_BF_DCA   = 0x0040,    /*!< Activate DC-to-DC converter                        */
    TFA9865A2_BF_BYPDYUVP= 0x0060,    /*!< Enable dynamic uvp                                 */
    TFA9865A2_BF_INTP  = 0x0071,    /*!< Interrupt config                                   */
    TFA9865A2_BF_QALARM= 0x0090,    /*!< Device response to Qpump OK flag                   */
    TFA9865A2_BF_BYPUVP= 0x00a0,    /*!< Bypass UVP                                         */
    TFA9865A2_BF_BYPOCP= 0x00b0,    /*!< Bypass OCP                                         */
    TFA9865A2_BF_ENPLLSYNC= 0x00e0,    /*!< Manager control for enabling synchronisation with PLL FS */
    TFA9865A2_BF_COORHYS= 0x00f0,    /*!< Select hysteresis for clock range detector         */
    TFA9865A2_BF_MANSCONF= 0x0120,    /*!< Configuration setting if I2C settings are uploaded by the host */
    TFA9865A2_BF_MUTETO= 0x0160,    /*!< Time out mute sequence                             */
    TFA9865A2_BF_MANROBOD= 0x0170,    /*!< Controls the reaction of the device on BOD         */
    TFA9865A2_BF_MANEDCTH= 0x01d0,    /*!< Device response to too high DC level flag (DCTH = 1) */
    TFA9865A2_BF_AUDFS = 0x0203,    /*!< Sample rate (Fs)                                   */
    TFA9865A2_BF_FRACTDEL= 0x0256,    /*!< V/I Fractional delay                               */
    TFA9865A2_BF_REV   = 0x030f,    /*!< product                                            */
    TFA9865A2_BF_REFCKEXT= 0x0400,    /*!< PLL external ref clock                             */
    TFA9865A2_BF_BYTDMGLF= 0x0420,    /*!< Bypass TDM FS/BCK/DATAI giltch filter              */
    TFA9865A2_BF_MANAOOSC= 0x0460,    /*!< Internal oscillator control during power down mode */
    TFA9865A2_BF_FSSYNCEN= 0x0480,    /*!< Enable FS synchronisation for clock divider        */
    TFA9865A2_BF_CLKREFSYNCEN= 0x0490,    /*!< Enable PLL reference clock synchronisation for clock divider */
    TFA9865A2_BF_PWMFREQ= 0x04a0,    /*!< PWM output frequency select                        */
    TFA9865A2_BF_CGUSYNCDCG= 0x0500,    /*!< Clock gating control for CGU synchronisation module */
    TFA9865A2_BF_FRCCLKSPKR= 0x0510,    /*!< Force active the speaker sub-system clock when in idle power */
    TFA9865A2_BF_DEVCAT= 0x0607,    /*!< product category                                   */
    TFA9865A2_BF_DEVREV= 0x0687,    /*!< version                                            */
    TFA9865A2_BF_VDDS  = 0x1000,    /*!< POR (sticky flag, clear on write a '1')            */
    TFA9865A2_BF_OTDS  = 0x1010,    /*!< OTP alarm (sticky flag,  clear on write a '1')     */
    TFA9865A2_BF_UVDS  = 0x1020,    /*!< UVP alarm (sticky flag,  clear on write a '1')     */
    TFA9865A2_BF_OVDS  = 0x1030,    /*!< OVP alarm (sticky flag,  clear on write a '1')     */
    TFA9865A2_BF_OCDS  = 0x1040,    /*!< OCP amplifier (sticky flag,  clear on write a '1') */
    TFA9865A2_BF_NOCLK = 0x1050,    /*!< Lost clock (sticky flag,  clear on write a '1')    */
    TFA9865A2_BF_CLKOOR= 0x1060,    /*!< External clock status (sticky flag,  clear on write a '1') */
    TFA9865A2_BF_DCOCPOK= 0x1070,    /*!< DCDC OCP nmos (sticky flag,  clear on write a '1') */
    TFA9865A2_BF_OCPOAP= 0x10b0,    /*!< OCPOK pmos A (sticky flag,  clear on write a '1')  */
    TFA9865A2_BF_OCPOAN= 0x10c0,    /*!< OCPOK nmos A (sticky flag,  clear on write a '1')  */
    TFA9865A2_BF_OCPOBP= 0x10d0,    /*!< OCPOK pmos B (sticky flag,  clear on write a '1')  */
    TFA9865A2_BF_OCPOBN= 0x10e0,    /*!< OCPOK nmos B (sticky flag,  clear on write a '1')  */
    TFA9865A2_BF_DCTH  = 0x10f0,    /*!< DC level on audio input stream too high (sticky flag,  clear on write a '1') */
    TFA9865A2_BF_CLKS  = 0x1100,    /*!< Clocks stable                                      */
    TFA9865A2_BF_OTPB  = 0x1110,    /*!< EFUSE busy                                         */
    TFA9865A2_BF_TDMERR= 0x1120,    /*!< TDM error                                          */
    TFA9865A2_BF_PLLS  = 0x1170,    /*!< PLL lock                                           */
    TFA9865A2_BF_TDMLUTER= 0x1180,    /*!< TDM LUT error                                      */
    TFA9865A2_BF_SWS   = 0x1190,    /*!< Amplifier engage                                   */
    TFA9865A2_BF_AMPS  = 0x11a0,    /*!< Amplifier enable                                   */
    TFA9865A2_BF_AREFS = 0x11b0,    /*!< References enable                                  */
    TFA9865A2_BF_CLIPS = 0x11c0,    /*!< Amplifier clipping                                 */
    TFA9865A2_BF_MANSTATE= 0x1203,    /*!< Device manager status                              */
    TFA9865A2_BF_AMPSTE= 0x1243,    /*!< Amplifier control status                           */
    TFA9865A2_BF_TDMSTAT= 0x1282,    /*!< TDM status bits                                    */
    TFA9865A2_BF_WAITSYNC= 0x12d0,    /*!< CGU and PLL synchronisation status flag from CGU   */
    TFA9865A2_BF_BODNOK= 0x1300,    /*!< BOD Flag VDD NOT OK (sticky flag,  clear on write a '1') */
    TFA9865A2_BF_QPFAIL= 0x1310,    /*!< QPUMP Fail Flag (sticky flag,  clear on write a '1') */
    TFA9865A2_BF_DCLD  = 0x140c,    /*!< DC level detected by DC protection module (2s complement) */
    TFA9865A2_BF_BATS  = 0x1509,    /*!< Battery voltage (V)                                */
    TFA9865A2_BF_TEMPS = 0x1608,    /*!< IC Temperature (C)                                 */
    TFA9865A2_BF_VDDPS = 0x1709,    /*!< IC VDDP voltage (V)                                */
    TFA9865A2_BF_TDME  = 0x2000,    /*!< Enable interface                                   */
    TFA9865A2_BF_AMPINSEL= 0x2011,    /*!< Amplifier input selection                          */
    TFA9865A2_BF_INPLEV= 0x2030,    /*!< TDM output attenuation                             */
    TFA9865A2_BF_TDMCLINV= 0x2040,    /*!< Reception data to BCK clock                        */
    TFA9865A2_BF_TDMFSPOL= 0x2050,    /*!< FS polarity                                        */
    TFA9865A2_BF_TDMSLOTS= 0x2061,    /*!< N-slots in Frame                                   */
    TFA9865A2_BF_TDMSLLN= 0x2081,    /*!< N-bits in slot                                     */
    TFA9865A2_BF_TDMSSIZE= 0x20a1,    /*!< Sample size per slot                               */
    TFA9865A2_BF_TDMNBCK= 0x20c3,    /*!< N-BCK's in FS                                      */
    TFA9865A2_BF_TDMDEL= 0x2100,    /*!< Data delay to FS                                   */
    TFA9865A2_BF_TDMADJ= 0x2110,    /*!< Data adjustment                                    */
    TFA9865A2_BF_TDMSPKE= 0x2120,    /*!< Control audio TDM channel in 0                     */
    TFA9865A2_BF_TDMDCE= 0x2130,    /*!< Control audio TDM channel in 1                     */
    TFA9865A2_BF_TDMSRC0E= 0x2140,    /*!< Enable TDM source0 data channel                    */
    TFA9865A2_BF_TDMSRC1E= 0x2150,    /*!< Enable TDM source1 data channel                    */
    TFA9865A2_BF_TDMSRC2E= 0x2160,    /*!< Enable TDM source2 data channel                    */
    TFA9865A2_BF_TDMSPKS= 0x2183,    /*!< TDM slot for sink 0                                */
    TFA9865A2_BF_TDMDCS= 0x21c3,    /*!< TDM slot for sink 1                                */
    TFA9865A2_BF_TDMSRC0S= 0x2203,    /*!< Slot Position of TDM source0 channel data          */
    TFA9865A2_BF_TDMSRC1S= 0x2243,    /*!< Slot Position of TDM source1 channel data          */
    TFA9865A2_BF_TDMSRC2S= 0x2283,    /*!< Slot Position of TDM source2 channel data          */
    TFA9865A2_BF_ISTVDDS= 0x4000,    /*!< Interrupt status POR                               */
    TFA9865A2_BF_ISTBSTOC= 0x4010,    /*!< Interrupt status DCDC OCP alarm                    */
    TFA9865A2_BF_ISTOTDS= 0x4020,    /*!< Interrupt status OTP alarm                         */
    TFA9865A2_BF_ISTOCPR= 0x4030,    /*!< Interrupt status OCP alarm                         */
    TFA9865A2_BF_ISTUVDS= 0x4040,    /*!< Interrupt status UVP alarm                         */
    TFA9865A2_BF_ISTTDMER= 0x4050,    /*!< Interrupt status TDM error                         */
    TFA9865A2_BF_ISTNOCLK= 0x4060,    /*!< Interrupt status lost clock                        */
    TFA9865A2_BF_ISTDCTH= 0x4070,    /*!< Interrupt status dc too high                       */
    TFA9865A2_BF_ISTBODNOK= 0x4080,    /*!< Interrupt status brown out detected                */
    TFA9865A2_BF_ISTCOOR= 0x4090,    /*!< Interrupt status clock out of range                */
    TFA9865A2_BF_ISTOVDS= 0x40a0,    /*!< Interrupt status OVP alarm                         */
    TFA9865A2_BF_ISTQPFAIL= 0x40b0,    /*!< Interrupt status qpump failure                     */
    TFA9865A2_BF_ICLVDDS= 0x4400,    /*!< Clear interrupt status POR                         */
    TFA9865A2_BF_ICLBSTOC= 0x4410,    /*!< Clear interrupt status DCDC OCP                    */
    TFA9865A2_BF_ICLOTDS= 0x4420,    /*!< Clear interrupt status OTP alarm                   */
    TFA9865A2_BF_ICLOCPR= 0x4430,    /*!< Clear interrupt status OCP alarm                   */
    TFA9865A2_BF_ICLUVDS= 0x4440,    /*!< Clear interrupt status UVP alarm                   */
    TFA9865A2_BF_ICLTDMER= 0x4450,    /*!< Clear interrupt status TDM error                   */
    TFA9865A2_BF_ICLNOCLK= 0x4460,    /*!< Clear interrupt status lost clk                    */
    TFA9865A2_BF_ICLDCTH= 0x4470,    /*!< Clear interrupt status dc too high                 */
    TFA9865A2_BF_ICLBODNOK= 0x4480,    /*!< Clear interrupt status brown out detected          */
    TFA9865A2_BF_ICLCOOR= 0x4490,    /*!< Clear interrupt status clock out of range          */
    TFA9865A2_BF_ICLOVDS= 0x44a0,    /*!< Clear interrupt status OVP alarm                   */
    TFA9865A2_BF_ICLQPFAIL= 0x44b0,    /*!< Clear interrupt status qpump failure               */
    TFA9865A2_BF_IEVDDS= 0x4800,    /*!< Enable interrupt POR                               */
    TFA9865A2_BF_IEBSTOC= 0x4810,    /*!< Enable interrupt DCDC OCP                          */
    TFA9865A2_BF_IEOTDS= 0x4820,    /*!< Enable interrupt OTP alarm                         */
    TFA9865A2_BF_IEOCPR= 0x4830,    /*!< Enable interrupt OCP alarm                         */
    TFA9865A2_BF_IEUVDS= 0x4840,    /*!< Enable interrupt UVP alarm                         */
    TFA9865A2_BF_IETDMER= 0x4850,    /*!< Enable interrupt TDM error                         */
    TFA9865A2_BF_IENOCLK= 0x4860,    /*!< Enable interrupt lost clk                          */
    TFA9865A2_BF_IEDCTH= 0x4870,    /*!< Enable interrupt dc too high                       */
    TFA9865A2_BF_IEBODNOK= 0x4880,    /*!< Enable interrupt brown out detect                  */
    TFA9865A2_BF_IECOOR= 0x4890,    /*!< Enable interrupt clock out of range                */
    TFA9865A2_BF_IEOVDS= 0x48a0,    /*!< Enable interrupt OVP alarm                         */
    TFA9865A2_BF_IEQPFAIL= 0x48b0,    /*!< Enable interrupt qpump failure                     */
    TFA9865A2_BF_IPOVDDS= 0x4c00,    /*!< Interrupt polarity POR                             */
    TFA9865A2_BF_IPOBSTOC= 0x4c10,    /*!< Interrupt polarity DCDC OCP                        */
    TFA9865A2_BF_IPOOTDS= 0x4c20,    /*!< Interrupt polarity OTP alarm                       */
    TFA9865A2_BF_IPOOCPR= 0x4c30,    /*!< Interrupt polarity OCP alarm                       */
    TFA9865A2_BF_IPOUVDS= 0x4c40,    /*!< Interrupt polarity UVP alarm                       */
    TFA9865A2_BF_IPOTDMER= 0x4c50,    /*!< Interrupt polarity TDM error                       */
    TFA9865A2_BF_IPONOCLK= 0x4c60,    /*!< Interrupt polarity lost clk                        */
    TFA9865A2_BF_IPODCTH= 0x4c70,    /*!< Interrupt polarity dc too high                     */
    TFA9865A2_BF_IPOBODNOK= 0x4c80,    /*!< Interrupt polarity brown out detect                */
    TFA9865A2_BF_IPOCOOR= 0x4c90,    /*!< Interrupt polarity clock out of range              */
    TFA9865A2_BF_IPOOVDS= 0x4ca0,    /*!< Interrupt polarity OVP alarm                       */
    TFA9865A2_BF_IPOQPFAIL= 0x4cb0,    /*!< Interrupt polarity qpump failure                   */
    TFA9865A2_BF_BSSCLRST= 0x50d0,    /*!< Reset clipper : auto-clear                         */
    TFA9865A2_BF_BSSR  = 0x50e0,    /*!< Battery voltage read out                           */
    TFA9865A2_BF_BSSBY = 0x50f0,    /*!< Bypass battery safeguard                           */
    TFA9865A2_BF_DCPTC = 0x5111,    /*!< Duration of DC level detection                     */
    TFA9865A2_BF_DCPL  = 0x5131,    /*!< DC-level detection                                 */
    TFA9865A2_BF_HPFBYP= 0x5150,    /*!< Bypass HPF                                         */
    TFA9865A2_BF_AMPGAIN= 0x5257,    /*!< Amplifier gain                                     */
    TFA9865A2_BF_BYPDLYLINE= 0x52f0,    /*!< Bypass the interpolator delay line                 */
    TFA9865A2_BF_BSSMVBS= 0x5a00,    /*!< min_vbat source select:                            */
    TFA9865A2_BF_DCMCC = 0x5a13,    /*!< DCMCC initial value                                */
    TFA9865A2_BF_BSST  = 0x5a53,    /*!< BSS threshold Vbat value (reducing gain)           */
    TFA9865A2_BF_BSSAR = 0x5a91,    /*!< BSS attack rate (reducing gain)                    */
    TFA9865A2_BF_BSSRR = 0x5ab3,    /*!< BSS release rate  (reducing gain)                  */
    TFA9865A2_BF_BSSHT = 0x5b03,    /*!< BSS hold time (reducing gain)                      */
    TFA9865A2_BF_BSSS  = 0x5b41,    /*!< BSS steepness (reducing gain)                      */
    TFA9865A2_BF_BSSRL = 0x5b62,    /*!< BSS maximum reduction level (reducing gain)        */
    TFA9865A2_BF_BSSDCT= 0x5b93,    /*!< BSS threshold Vbat value (reducing DCMCC)          */
    TFA9865A2_BF_BSSDCAR= 0x5bd1,    /*!< BSS attack rate  (reducing DCMCC)                  */
    TFA9865A2_BF_BSSDCRR= 0x5c03,    /*!< BSS release rate  (reducing DCMCC)                 */
    TFA9865A2_BF_BSSDCHT= 0x5c43,    /*!< BSS hold time (reducing DCMCC)                     */
    TFA9865A2_BF_BSSDCS= 0x5c81,    /*!< Battery supply safeguard steepness (reducing DCMCC) */
    TFA9865A2_BF_DCLIPFLS= 0x5cd1,    /*!< select source digital clip flag                    */
    TFA9865A2_BF_TDMSPKG= 0x5f63,    /*!< Total gain depending on INPLEV setting (channel 0) */
    TFA9865A2_BF_IPM   = 0x60e1,    /*!< Idle power mode control                            */
    TFA9865A2_BF_TDMSRCMAP= 0x6802,    /*!< TDM source mapping                                 */
    TFA9865A2_BF_TDMSRCAS= 0x6842,    /*!< Sensed value A                                     */
    TFA9865A2_BF_TDMSRCBS= 0x6872,    /*!< Sensed value B                                     */
    TFA9865A2_BF_TDMSRCACLIP= 0x68a1,    /*!< Clip flag information with combined clip flag      */
    TFA9865A2_BF_LP0   = 0x6e00,    /*!< Idle power mode                                    */
    TFA9865A2_BF_LVLCLPPWM= 0x6f72,    /*!< clip detect threshold control                      */
    TFA9865A2_BF_DCIE  = 0x7060,    /*!< Adaptive boost mode                                */
    TFA9865A2_BF_DCNS  = 0x7400,    /*!< Disable control of noise shaper in DCDC control    */
    TFA9865A2_BF_DCNSRST= 0x7410,    /*!< Disable control of reset of noise shaper when 8 bit value for dcdc control occurs */
    TFA9865A2_BF_DCTRIP= 0x7444,    /*!< Headroom for 1st Adaptive boost trip level, effective only when DCIE is set to 1 */
    TFA9865A2_BF_DCHOLD= 0x7494,    /*!< Hysteresis for tracking mode of DCDC booster, effective only when DCIE is set to 1 */
    TFA9865A2_BF_DCINT = 0x74e0,    /*!< Selection of data for adaptive boost algorithm, effective only when DCIE is set to 1 */
    TFA9865A2_BF_DCTRIPT= 0x7584,    /*!< Headroom for Tracking Adaptive boost trip level, effective only when DCIE is 1 and DCTRACK is 1 */
    TFA9865A2_BF_BYPDCLPF= 0x75d0,    /*!< Bypass control of DCDC control low pass filter for quantization noise suppression */
    TFA9865A2_BF_DCTRIPHYSTE= 0x75f0,    /*!< Enable hysteresis on booster trip levels           */
    TFA9865A2_BF_DCVOS = 0x7687,    /*!< Second boost voltage level                         */
    TFA9865A2_BF_EFUSEK= 0xa107,    /*!< EFUSE KEY2 register                                */
    TFA9865A2_BF_KEY1LOCKED= 0xa200,    /*!< Indicates KEY1 is locked                           */
    TFA9865A2_BF_KEY2LOCKED= 0xa210,    /*!< Indicates KEY2 is locked                           */
    TFA9865A2_BF_PLLPDIV= 0xc934,    /*!< PLL PDIV in PLL direct control mode only, use_direct_pll_ctrl set to 1 */
    TFA9865A2_BF_PLLNDIV= 0xc987,    /*!< PLL NDIV in PLL direct control mode only, use_direct_pll_ctrl set to 1 */
    TFA9865A2_BF_PLLMDIV= 0xca0f,    /*!< PLL MDIV in PLL direct control mode only, use_direct_pll_ctrl set to 1 */
    TFA9865A2_BF_PLLSTRTM= 0xce42,    /*!< PLL startup time selection control                 */
    TFA9865A2_BF_DCDIS = 0xd190,    /*!< DCDC on/off                                        */
    TFA9865A2_BF_SWPROFIL= 0xe00f,    /*!< Software profile data                              */
    TFA9865A2_BF_SWVSTEP= 0xe10f,    /*!< Software vstep information                         */
} tfa9865A2BfEnumList_t;
#define TFA9865A2_NAMETABLE static tfaBfName_t Tfa9865A2DatasheetNames[]= {\
   { 0x0, "PWDN"},    /* Powerdown selection                               , */\
   { 0x10, "I2CR"},    /* I2C reset - auto clear                            , */\
   { 0x30, "AMPE"},    /* Activate amplifier                                , */\
   { 0x40, "DCA"},    /* Activate DC-to-DC converter                       , */\
   { 0x60, "BYPDYUVP"},    /* Enable dynamic uvp                                , */\
   { 0x71, "INTP"},    /* Interrupt config                                  , */\
   { 0x90, "QALARM"},    /* Device response to Qpump OK flag                  , */\
   { 0xa0, "BYPUVP"},    /* Bypass UVP                                        , */\
   { 0xb0, "BYPOCP"},    /* Bypass OCP                                        , */\
   { 0xe0, "ENPLLSYNC"},    /* Manager control for enabling synchronisation with PLL FS, */\
   { 0xf0, "COORHYS"},    /* Select hysteresis for clock range detector        , */\
   { 0x120, "MANSCONF"},    /* Configuration setting if I2C settings are uploaded by the host, */\
   { 0x160, "MUTETO"},    /* Time out mute sequence                            , */\
   { 0x170, "MANROBOD"},    /* Controls the reaction of the device on BOD        , */\
   { 0x1d0, "MANEDCTH"},    /* Device response to too high DC level flag (DCTH = 1), */\
   { 0x203, "AUDFS"},    /* Sample rate (Fs)                                  , */\
   { 0x256, "FRACTDEL"},    /* V/I Fractional delay                              , */\
   { 0x30f, "REV"},    /* product                                           , */\
   { 0x400, "REFCKEXT"},    /* PLL external ref clock                            , */\
   { 0x420, "BYTDMGLF"},    /* Bypass TDM FS/BCK/DATAI giltch filter             , */\
   { 0x460, "MANAOOSC"},    /* Internal oscillator control during power down mode, */\
   { 0x480, "FSSYNCEN"},    /* Enable FS synchronisation for clock divider       , */\
   { 0x490, "CLKREFSYNCEN"},    /* Enable PLL reference clock synchronisation for clock divider, */\
   { 0x4a0, "PWMFREQ"},    /* PWM output frequency select                       , */\
   { 0x500, "CGUSYNCDCG"},    /* Clock gating control for CGU synchronisation module, */\
   { 0x510, "FRCCLKSPKR"},    /* Force active the speaker sub-system clock when in idle power, */\
   { 0x607, "DEVCAT"},    /* product category                                  , */\
   { 0x687, "DEVREV"},    /* version                                           , */\
   { 0x1000, "VDDS"},    /* POR (sticky flag, clear on write a '1')           , */\
   { 0x1010, "OTDS"},    /* OTP alarm (sticky flag,  clear on write a '1')    , */\
   { 0x1020, "UVDS"},    /* UVP alarm (sticky flag,  clear on write a '1')    , */\
   { 0x1030, "OVDS"},    /* OVP alarm (sticky flag,  clear on write a '1')    , */\
   { 0x1040, "OCDS"},    /* OCP amplifier (sticky flag,  clear on write a '1'), */\
   { 0x1050, "NOCLK"},    /* Lost clock (sticky flag,  clear on write a '1')   , */\
   { 0x1060, "CLKOOR"},    /* External clock status (sticky flag,  clear on write a '1'), */\
   { 0x1070, "DCOCPOK"},    /* DCDC OCP nmos (sticky flag,  clear on write a '1'), */\
   { 0x10b0, "OCPOAP"},    /* OCPOK pmos A (sticky flag,  clear on write a '1') , */\
   { 0x10c0, "OCPOAN"},    /* OCPOK nmos A (sticky flag,  clear on write a '1') , */\
   { 0x10d0, "OCPOBP"},    /* OCPOK pmos B (sticky flag,  clear on write a '1') , */\
   { 0x10e0, "OCPOBN"},    /* OCPOK nmos B (sticky flag,  clear on write a '1') , */\
   { 0x10f0, "DCTH"},    /* DC level on audio input stream too high (sticky flag,  clear on write a '1'), */\
   { 0x1100, "CLKS"},    /* Clocks stable                                     , */\
   { 0x1110, "OTPB"},    /* EFUSE busy                                        , */\
   { 0x1120, "TDMERR"},    /* TDM error                                         , */\
   { 0x1170, "PLLS"},    /* PLL lock                                          , */\
   { 0x1180, "TDMLUTER"},    /* TDM LUT error                                     , */\
   { 0x1190, "SWS"},    /* Amplifier engage                                  , */\
   { 0x11a0, "AMPS"},    /* Amplifier enable                                  , */\
   { 0x11b0, "AREFS"},    /* References enable                                 , */\
   { 0x11c0, "CLIPS"},    /* Amplifier clipping                                , */\
   { 0x1203, "MANSTATE"},    /* Device manager status                             , */\
   { 0x1243, "AMPSTE"},    /* Amplifier control status                          , */\
   { 0x1282, "TDMSTAT"},    /* TDM status bits                                   , */\
   { 0x12d0, "WAITSYNC"},    /* CGU and PLL synchronisation status flag from CGU  , */\
   { 0x1300, "BODNOK"},    /* BOD Flag VDD NOT OK (sticky flag,  clear on write a '1'), */\
   { 0x1310, "QPFAIL"},    /* QPUMP Fail Flag (sticky flag,  clear on write a '1'), */\
   { 0x140c, "DCLD"},    /* DC level detected by DC protection module (2s complement), */\
   { 0x1509, "BATS"},    /* Battery voltage (V)                               , */\
   { 0x1608, "TEMPS"},    /* IC Temperature (C)                                , */\
   { 0x1709, "VDDPS"},    /* IC VDDP voltage (V)                               , */\
   { 0x2000, "TDME"},    /* Enable interface                                  , */\
   { 0x2011, "AMPINSEL"},    /* Amplifier input selection                         , */\
   { 0x2030, "INPLEV"},    /* TDM output attenuation                            , */\
   { 0x2040, "TDMCLINV"},    /* Reception data to BCK clock                       , */\
   { 0x2050, "TDMFSPOL"},    /* FS polarity                                       , */\
   { 0x2061, "TDMSLOTS"},    /* N-slots in Frame                                  , */\
   { 0x2081, "TDMSLLN"},    /* N-bits in slot                                    , */\
   { 0x20a1, "TDMSSIZE"},    /* Sample size per slot                              , */\
   { 0x20c3, "TDMNBCK"},    /* N-BCK's in FS                                     , */\
   { 0x2100, "TDMDEL"},    /* Data delay to FS                                  , */\
   { 0x2110, "TDMADJ"},    /* Data adjustment                                   , */\
   { 0x2120, "TDMSPKE"},    /* Control audio TDM channel in 0                    , */\
   { 0x2130, "TDMDCE"},    /* Control audio TDM channel in 1                    , */\
   { 0x2140, "TDMSRC0E"},    /* Enable TDM source0 data channel                   , */\
   { 0x2150, "TDMSRC1E"},    /* Enable TDM source1 data channel                   , */\
   { 0x2160, "TDMSRC2E"},    /* Enable TDM source2 data channel                   , */\
   { 0x2183, "TDMSPKS"},    /* TDM slot for sink 0                               , */\
   { 0x21c3, "TDMDCS"},    /* TDM slot for sink 1                               , */\
   { 0x2203, "TDMSRC0S"},    /* Slot Position of TDM source0 channel data         , */\
   { 0x2243, "TDMSRC1S"},    /* Slot Position of TDM source1 channel data         , */\
   { 0x2283, "TDMSRC2S"},    /* Slot Position of TDM source2 channel data         , */\
   { 0x4000, "ISTVDDS"},    /* Interrupt status POR                              , */\
   { 0x4010, "ISTBSTOC"},    /* Interrupt status DCDC OCP alarm                   , */\
   { 0x4020, "ISTOTDS"},    /* Interrupt status OTP alarm                        , */\
   { 0x4030, "ISTOCPR"},    /* Interrupt status OCP alarm                        , */\
   { 0x4040, "ISTUVDS"},    /* Interrupt status UVP alarm                        , */\
   { 0x4050, "ISTTDMER"},    /* Interrupt status TDM error                        , */\
   { 0x4060, "ISTNOCLK"},    /* Interrupt status lost clock                       , */\
   { 0x4070, "ISTDCTH"},    /* Interrupt status dc too high                      , */\
   { 0x4080, "ISTBODNOK"},    /* Interrupt status brown out detected               , */\
   { 0x4090, "ISTCOOR"},    /* Interrupt status clock out of range               , */\
   { 0x40a0, "ISTOVDS"},    /* Interrupt status OVP alarm                        , */\
   { 0x40b0, "ISTQPFAIL"},    /* Interrupt status qpump failure                    , */\
   { 0x4400, "ICLVDDS"},    /* Clear interrupt status POR                        , */\
   { 0x4410, "ICLBSTOC"},    /* Clear interrupt status DCDC OCP                   , */\
   { 0x4420, "ICLOTDS"},    /* Clear interrupt status OTP alarm                  , */\
   { 0x4430, "ICLOCPR"},    /* Clear interrupt status OCP alarm                  , */\
   { 0x4440, "ICLUVDS"},    /* Clear interrupt status UVP alarm                  , */\
   { 0x4450, "ICLTDMER"},    /* Clear interrupt status TDM error                  , */\
   { 0x4460, "ICLNOCLK"},    /* Clear interrupt status lost clk                   , */\
   { 0x4470, "ICLDCTH"},    /* Clear interrupt status dc too high                , */\
   { 0x4480, "ICLBODNOK"},    /* Clear interrupt status brown out detected         , */\
   { 0x4490, "ICLCOOR"},    /* Clear interrupt status clock out of range         , */\
   { 0x44a0, "ICLOVDS"},    /* Clear interrupt status OVP alarm                  , */\
   { 0x44b0, "ICLQPFAIL"},    /* Clear interrupt status qpump failure              , */\
   { 0x4800, "IEVDDS"},    /* Enable interrupt POR                              , */\
   { 0x4810, "IEBSTOC"},    /* Enable interrupt DCDC OCP                         , */\
   { 0x4820, "IEOTDS"},    /* Enable interrupt OTP alarm                        , */\
   { 0x4830, "IEOCPR"},    /* Enable interrupt OCP alarm                        , */\
   { 0x4840, "IEUVDS"},    /* Enable interrupt UVP alarm                        , */\
   { 0x4850, "IETDMER"},    /* Enable interrupt TDM error                        , */\
   { 0x4860, "IENOCLK"},    /* Enable interrupt lost clk                         , */\
   { 0x4870, "IEDCTH"},    /* Enable interrupt dc too high                      , */\
   { 0x4880, "IEBODNOK"},    /* Enable interrupt brown out detect                 , */\
   { 0x4890, "IECOOR"},    /* Enable interrupt clock out of range               , */\
   { 0x48a0, "IEOVDS"},    /* Enable interrupt OVP alarm                        , */\
   { 0x48b0, "IEQPFAIL"},    /* Enable interrupt qpump failure                    , */\
   { 0x4c00, "IPOVDDS"},    /* Interrupt polarity POR                            , */\
   { 0x4c10, "IPOBSTOC"},    /* Interrupt polarity DCDC OCP                       , */\
   { 0x4c20, "IPOOTDS"},    /* Interrupt polarity OTP alarm                      , */\
   { 0x4c30, "IPOOCPR"},    /* Interrupt polarity OCP alarm                      , */\
   { 0x4c40, "IPOUVDS"},    /* Interrupt polarity UVP alarm                      , */\
   { 0x4c50, "IPOTDMER"},    /* Interrupt polarity TDM error                      , */\
   { 0x4c60, "IPONOCLK"},    /* Interrupt polarity lost clk                       , */\
   { 0x4c70, "IPODCTH"},    /* Interrupt polarity dc too high                    , */\
   { 0x4c80, "IPOBODNOK"},    /* Interrupt polarity brown out detect               , */\
   { 0x4c90, "IPOCOOR"},    /* Interrupt polarity clock out of range             , */\
   { 0x4ca0, "IPOOVDS"},    /* Interrupt polarity OVP alarm                      , */\
   { 0x4cb0, "IPOQPFAIL"},    /* Interrupt polarity qpump failure                  , */\
   { 0x50d0, "BSSCLRST"},    /* Reset clipper : auto-clear                        , */\
   { 0x50e0, "BSSR"},    /* Battery voltage read out                          , */\
   { 0x50f0, "BSSBY"},    /* Bypass battery safeguard                          , */\
   { 0x5111, "DCPTC"},    /* Duration of DC level detection                    , */\
   { 0x5131, "DCPL"},    /* DC-level detection                                , */\
   { 0x5150, "HPFBYP"},    /* Bypass HPF                                        , */\
   { 0x5257, "AMPGAIN"},    /* Amplifier gain                                    , */\
   { 0x52f0, "BYPDLYLINE"},    /* Bypass the interpolator delay line                , */\
   { 0x5a00, "BSSMVBS"},    /* min_vbat source select:                           , */\
   { 0x5a13, "DCMCC"},    /* DCMCC initial value                               , */\
   { 0x5a53, "BSST"},    /* BSS threshold Vbat value (reducing gain)          , */\
   { 0x5a91, "BSSAR"},    /* BSS attack rate (reducing gain)                   , */\
   { 0x5ab3, "BSSRR"},    /* BSS release rate  (reducing gain)                 , */\
   { 0x5b03, "BSSHT"},    /* BSS hold time (reducing gain)                     , */\
   { 0x5b41, "BSSS"},    /* BSS steepness (reducing gain)                     , */\
   { 0x5b62, "BSSRL"},    /* BSS maximum reduction level (reducing gain)       , */\
   { 0x5b93, "BSSDCT"},    /* BSS threshold Vbat value (reducing DCMCC)         , */\
   { 0x5bd1, "BSSDCAR"},    /* BSS attack rate  (reducing DCMCC)                 , */\
   { 0x5c03, "BSSDCRR"},    /* BSS release rate  (reducing DCMCC)                , */\
   { 0x5c43, "BSSDCHT"},    /* BSS hold time (reducing DCMCC)                    , */\
   { 0x5c81, "BSSDCS"},    /* Battery supply safeguard steepness (reducing DCMCC), */\
   { 0x5cd1, "DCLIPFLS"},    /* select source digital clip flag                   , */\
   { 0x5f63, "TDMSPKG"},    /* Total gain depending on INPLEV setting (channel 0), */\
   { 0x60e1, "IPM"},    /* Idle power mode control                           , */\
   { 0x6802, "TDMSRCMAP"},    /* TDM source mapping                                , */\
   { 0x6842, "TDMSRCAS"},    /* Sensed value A                                    , */\
   { 0x6872, "TDMSRCBS"},    /* Sensed value B                                    , */\
   { 0x68a1, "TDMSRCACLIP"},    /* Clip flag information with combined clip flag     , */\
   { 0x6e00, "LP0"},    /* Idle power mode                                   , */\
   { 0x6f72, "LVLCLPPWM"},    /* clip detect threshold control                     , */\
   { 0x7060, "DCIE"},    /* Adaptive boost mode                               , */\
   { 0x7400, "DCNS"},    /* Disable control of noise shaper in DCDC control   , */\
   { 0x7410, "DCNSRST"},    /* Disable control of reset of noise shaper when 8 bit value for dcdc control occurs, */\
   { 0x7444, "DCTRIP"},    /* Headroom for 1st Adaptive boost trip level, effective only when DCIE is set to 1, */\
   { 0x7494, "DCHOLD"},    /* Hysteresis for tracking mode of DCDC booster, effective only when DCIE is set to 1, */\
   { 0x74e0, "DCINT"},    /* Selection of data for adaptive boost algorithm, effective only when DCIE is set to 1, */\
   { 0x7584, "DCTRIPT"},    /* Headroom for Tracking Adaptive boost trip level, effective only when DCIE is 1 and DCTRACK is 1, */\
   { 0x75d0, "BYPDCLPF"},    /* Bypass control of DCDC control low pass filter for quantization noise suppression, */\
   { 0x75f0, "DCTRIPHYSTE"},    /* Enable hysteresis on booster trip levels          , */\
   { 0x7687, "DCVOS"},    /* Second boost voltage level                        , */\
   { 0xa107, "EFUSEK"},    /* EFUSE KEY2 register                               , */\
   { 0xa200, "KEY1LOCKED"},    /* Indicates KEY1 is locked                          , */\
   { 0xa210, "KEY2LOCKED"},    /* Indicates KEY2 is locked                          , */\
   { 0xc934, "PLLPDIV"},    /* PLL PDIV in PLL direct control mode only, use_direct_pll_ctrl set to 1, */\
   { 0xc987, "PLLNDIV"},    /* PLL NDIV in PLL direct control mode only, use_direct_pll_ctrl set to 1, */\
   { 0xca0f, "PLLMDIV"},    /* PLL MDIV in PLL direct control mode only, use_direct_pll_ctrl set to 1, */\
   { 0xce42, "PLLSTRTM"},    /* PLL startup time selection control                , */\
   { 0xd190, "DCDIS"},    /* DCDC on/off                                       , */\
   { 0xe00f, "SWPROFIL"},    /* Software profile data                             , */\
   { 0xe10f, "SWVSTEP"},    /* Software vstep information                        , */\
   { 0xffff,"Unknown bitfield enum" }   /* not found */\
};

#define TFA9865A2_BITNAMETABLE static tfaBfName_t Tfa9865A2BitNames[]= {\
   { 0x0, "powerdown"},    /* Powerdown selection                               , */\
   { 0x10, "reset"},    /* I2C reset - auto clear                            , */\
   { 0x30, "enbl_amplifier"},    /* Activate amplifier                                , */\
   { 0x40, "enbl_boost"},    /* Activate DC-to-DC converter                       , */\
   { 0x60, "enable_dynamic_uvp"},    /* Enable dynamic uvp                                , */\
   { 0x71, "int_pad_io"},    /* Interrupt config                                  , */\
   { 0x90, "man_enbl_qpump_ok"},    /* Device response to Qpump OK flag                  , */\
   { 0xa0, "bypass_uvp"},    /* Bypass UVP                                        , */\
   { 0xb0, "bypass_ocp"},    /* Bypass OCP                                        , */\
   { 0xe0, "enbl_pll_synchronisation"},    /* Manager control for enabling synchronisation with PLL FS, */\
   { 0xf0, "sel_hysteresis"},    /* Select hysteresis for clock range detector        , */\
   { 0x120, "src_set_configured"},    /* Configuration setting if I2C settings are uploaded by the host, */\
   { 0x160, "disable_mute_time_out"},    /* Time out mute sequence                            , */\
   { 0x170, "man_enbl_brown"},    /* Controls the reaction of the device on BOD        , */\
   { 0x1d0, "man_enbl_dc_too_high"},    /* Device response to too high DC level flag (DCTH = 1), */\
   { 0x203, "audio_fs"},    /* Sample rate (Fs)                                  , */\
   { 0x256, "cs_frac_delay"},    /* V/I Fractional delay                              , */\
   { 0x30f, "product"},    /* product                                           , */\
   { 0x400, "pll_clkin_sel"},    /* PLL external ref clock                            , */\
   { 0x420, "bypass_tdm_glitch_filter"},    /* Bypass TDM FS/BCK/DATAI giltch filter             , */\
   { 0x460, "enbl_osc_auto_off"},    /* Internal oscillator control during power down mode, */\
   { 0x480, "enbl_fs_sync"},    /* Enable FS synchronisation for clock divider       , */\
   { 0x490, "enbl_clkref_sync"},    /* Enable PLL reference clock synchronisation for clock divider, */\
   { 0x4a0, "sel_pwm_freq"},    /* PWM output frequency select                       , */\
   { 0x500, "disable_cgu_sync_cgate"},    /* Clock gating control for CGU synchronisation module, */\
   { 0x510, "force_spkr_clk"},    /* Force active the speaker sub-system clock when in idle power, */\
   { 0x607, "category"},    /* product category                                  , */\
   { 0x687, "version"},    /* version                                           , */\
   { 0x802, "ctrl_on2off_criterion"},    /* Amplifier on-off criteria for shutdown            , */\
   { 0xf0f, "hidden_code"},    /* Hidden code to enable access to key registers     , */\
   { 0x1000, "flag_por"},    /* POR (sticky flag, clear on write a '1')           , */\
   { 0x1010, "flag_otpok"},    /* OTP alarm (sticky flag,  clear on write a '1')    , */\
   { 0x1020, "flag_uvpok"},    /* UVP alarm (sticky flag,  clear on write a '1')    , */\
   { 0x1030, "flag_ovpok"},    /* OVP alarm (sticky flag,  clear on write a '1')    , */\
   { 0x1040, "flag_ocp_alarm"},    /* OCP amplifier (sticky flag,  clear on write a '1'), */\
   { 0x1050, "flag_lost_clk"},    /* Lost clock (sticky flag,  clear on write a '1')   , */\
   { 0x1060, "flag_clk_out_of_range"},    /* External clock status (sticky flag,  clear on write a '1'), */\
   { 0x1070, "flag_bst_ocpok"},    /* DCDC OCP nmos (sticky flag,  clear on write a '1'), */\
   { 0x10b0, "flag_ocpokap"},    /* OCPOK pmos A (sticky flag,  clear on write a '1') , */\
   { 0x10c0, "flag_ocpokan"},    /* OCPOK nmos A (sticky flag,  clear on write a '1') , */\
   { 0x10d0, "flag_ocpokbp"},    /* OCPOK pmos B (sticky flag,  clear on write a '1') , */\
   { 0x10e0, "flag_ocpokbn"},    /* OCPOK nmos B (sticky flag,  clear on write a '1') , */\
   { 0x10f0, "flag_dc_too_high"},    /* DC level on audio input stream too high (sticky flag,  clear on write a '1'), */\
   { 0x1100, "flag_clocks_stable"},    /* Clocks stable                                     , */\
   { 0x1110, "flag_efuse_busy"},    /* EFUSE busy                                        , */\
   { 0x1120, "flag_tdm_error"},    /* TDM error                                         , */\
   { 0x1170, "flag_pll_lock"},    /* PLL lock                                          , */\
   { 0x1180, "flag_tdm_lut_error"},    /* TDM LUT error                                     , */\
   { 0x1190, "flag_engage"},    /* Amplifier engage                                  , */\
   { 0x11a0, "flag_enbl_amp"},    /* Amplifier enable                                  , */\
   { 0x11b0, "flag_enbl_ref"},    /* References enable                                 , */\
   { 0x11c0, "flag_clip"},    /* Amplifier clipping                                , */\
   { 0x1203, "man_state"},    /* Device manager status                             , */\
   { 0x1243, "amp_ctrl_state"},    /* Amplifier control status                          , */\
   { 0x1282, "flag_tdm_status"},    /* TDM status bits                                   , */\
   { 0x12d0, "flag_waiting_for_sync"},    /* CGU and PLL synchronisation status flag from CGU  , */\
   { 0x1300, "flag_bod_vddd_nok"},    /* BOD Flag VDD NOT OK (sticky flag,  clear on write a '1'), */\
   { 0x1310, "flag_qpump_fail"},    /* QPUMP Fail Flag (sticky flag,  clear on write a '1'), */\
   { 0x140c, "dc_level_detect"},    /* DC level detected by DC protection module (2s complement), */\
   { 0x1509, "bat_adc"},    /* Battery voltage (V)                               , */\
   { 0x1608, "temp_adc"},    /* IC Temperature (C)                                , */\
   { 0x1709, "vddp_adc"},    /* IC VDDP voltage (V)                               , */\
   { 0x1800, "pll_mdiv_mode"},    /* feedback divider mode selection signal, for testing, */\
   { 0x1817, "pll_dpll_itrim"},    /* dpll trim cco current code , for testing                                                                                  , */\
   { 0x1894, "pll_mdiv_ps"},    /* prescaler threshold value in feedback divider, for testing                                others =  (pll_mdiv_s - pll_mdiv)/pll_mdiv_p, */\
   { 0x190b, "pll_mdiv_p"},    /* program counter threshold value in feedback divider, for testing                   others =  (pll_mdiv_s - pll_mdiv)/pll_mdiv_ps, */\
   { 0x19c3, "pll_mdiv_s"},    /* swallow counter threshold value in feedback divider, for testing                   others =  (pll_mdiv - pll_mdiv_ps*pll_mdiv_p, */\
   { 0x1a0f, "pll_dpll_comp_val"},    /* comparison value in dpll, for testing             , */\
   { 0x2000, "tdm_enable"},    /* Enable interface                                  , */\
   { 0x2011, "tdm_vamp_sel"},    /* Amplifier input selection                         , */\
   { 0x2030, "tdm_input_level"},    /* TDM output attenuation                            , */\
   { 0x2040, "tdm_clk_inversion"},    /* Reception data to BCK clock                       , */\
   { 0x2050, "tdm_fs_ws_polarity"},    /* FS polarity                                       , */\
   { 0x2061, "tdm_nb_of_slots"},    /* N-slots in Frame                                  , */\
   { 0x2081, "tdm_slot_length"},    /* N-bits in slot                                    , */\
   { 0x20a1, "tdm_sample_size"},    /* Sample size per slot                              , */\
   { 0x20c3, "tdm_nbck"},    /* N-BCK's in FS                                     , */\
   { 0x2100, "tdm_data_delay"},    /* Data delay to FS                                  , */\
   { 0x2110, "tdm_data_adjustment"},    /* Data adjustment                                   , */\
   { 0x2120, "tdm_sink0_enable"},    /* Control audio TDM channel in 0                    , */\
   { 0x2130, "tdm_sink1_enable"},    /* Control audio TDM channel in 1                    , */\
   { 0x2140, "tdm_source0_enable"},    /* Enable TDM source0 data channel                   , */\
   { 0x2150, "tdm_source1_enable"},    /* Enable TDM source1 data channel                   , */\
   { 0x2160, "tdm_source2_enable"},    /* Enable TDM source2 data channel                   , */\
   { 0x2183, "tdm_sink0_slot"},    /* TDM slot for sink 0                               , */\
   { 0x21c3, "tdm_sink1_slot"},    /* TDM slot for sink 1                               , */\
   { 0x2203, "tdm_source0_slot"},    /* Slot Position of TDM source0 channel data         , */\
   { 0x2243, "tdm_source1_slot"},    /* Slot Position of TDM source1 channel data         , */\
   { 0x2283, "tdm_source2_slot"},    /* Slot Position of TDM source2 channel data         , */\
   { 0x4000, "int_out_flag_por"},    /* Interrupt status POR                              , */\
   { 0x4010, "int_out_flag_bst_ocpok"},    /* Interrupt status DCDC OCP alarm                   , */\
   { 0x4020, "int_out_flag_otpok"},    /* Interrupt status OTP alarm                        , */\
   { 0x4030, "int_out_flag_ocp_alarm"},    /* Interrupt status OCP alarm                        , */\
   { 0x4040, "int_out_flag_uvpok"},    /* Interrupt status UVP alarm                        , */\
   { 0x4050, "int_out_flag_tdm_error"},    /* Interrupt status TDM error                        , */\
   { 0x4060, "int_out_flag_lost_clk"},    /* Interrupt status lost clock                       , */\
   { 0x4070, "int_out_flag_dc_too_high"},    /* Interrupt status dc too high                      , */\
   { 0x4080, "int_out_flag_bod_vddd_nok"},    /* Interrupt status brown out detected               , */\
   { 0x4090, "int_out_flag_clk_out_of_range"},    /* Interrupt status clock out of range               , */\
   { 0x40a0, "int_out_flag_ovpok"},    /* Interrupt status OVP alarm                        , */\
   { 0x40b0, "int_out_flag_qpump_fail"},    /* Interrupt status qpump failure                    , */\
   { 0x4400, "int_in_flag_por"},    /* Clear interrupt status POR                        , */\
   { 0x4410, "int_in_flag_bst_ocpok"},    /* Clear interrupt status DCDC OCP                   , */\
   { 0x4420, "int_in_flag_otpok"},    /* Clear interrupt status OTP alarm                  , */\
   { 0x4430, "int_in_flag_ocp_alarm"},    /* Clear interrupt status OCP alarm                  , */\
   { 0x4440, "int_in_flag_uvpok"},    /* Clear interrupt status UVP alarm                  , */\
   { 0x4450, "int_in_flag_tdm_error"},    /* Clear interrupt status TDM error                  , */\
   { 0x4460, "int_in_flag_lost_clk"},    /* Clear interrupt status lost clk                   , */\
   { 0x4470, "int_in_flag_dc_too_high"},    /* Clear interrupt status dc too high                , */\
   { 0x4480, "int_in_flag_bod_vddd_nok"},    /* Clear interrupt status brown out detected         , */\
   { 0x4490, "int_in_flag_clk_out_of_range"},    /* Clear interrupt status clock out of range         , */\
   { 0x44a0, "int_in_flag_ovpok"},    /* Clear interrupt status OVP alarm                  , */\
   { 0x44b0, "int_in_flag_qpump_fail"},    /* Clear interrupt status qpump failure              , */\
   { 0x4800, "int_enable_flag_por"},    /* Enable interrupt POR                              , */\
   { 0x4810, "int_enable_flag_bst_ocpok"},    /* Enable interrupt DCDC OCP                         , */\
   { 0x4820, "int_enable_flag_otpok"},    /* Enable interrupt OTP alarm                        , */\
   { 0x4830, "int_enable_flag_ocp_alarm"},    /* Enable interrupt OCP alarm                        , */\
   { 0x4840, "int_enable_flag_uvpok"},    /* Enable interrupt UVP alarm                        , */\
   { 0x4850, "int_enable_flag_tdm_error"},    /* Enable interrupt TDM error                        , */\
   { 0x4860, "int_enable_flag_lost_clk"},    /* Enable interrupt lost clk                         , */\
   { 0x4870, "int_enable_flag_dc_too_high"},    /* Enable interrupt dc too high                      , */\
   { 0x4880, "int_enable_flag_bod_vddd_nok"},    /* Enable interrupt brown out detect                 , */\
   { 0x4890, "int_enable_flag_clk_out_of_range"},    /* Enable interrupt clock out of range               , */\
   { 0x48a0, "int_enable_flag_ovpok"},    /* Enable interrupt OVP alarm                        , */\
   { 0x48b0, "int_enable_flag_qpump_fail"},    /* Enable interrupt qpump failure                    , */\
   { 0x4c00, "int_polarity_flag_por"},    /* Interrupt polarity POR                            , */\
   { 0x4c10, "int_polarity_flag_bst_ocpok"},    /* Interrupt polarity DCDC OCP                       , */\
   { 0x4c20, "int_polarity_flag_otpok"},    /* Interrupt polarity OTP alarm                      , */\
   { 0x4c30, "int_polarity_flag_ocp_alarm"},    /* Interrupt polarity OCP alarm                      , */\
   { 0x4c40, "int_polarity_flag_uvpok"},    /* Interrupt polarity UVP alarm                      , */\
   { 0x4c50, "int_polarity_flag_tdm_error"},    /* Interrupt polarity TDM error                      , */\
   { 0x4c60, "int_polarity_flag_lost_clk"},    /* Interrupt polarity lost clk                       , */\
   { 0x4c70, "int_polarity_flag_dc_too_high"},    /* Interrupt polarity dc too high                    , */\
   { 0x4c80, "int_polarity_flag_bod_vddd_nok"},    /* Interrupt polarity brown out detect               , */\
   { 0x4c90, "int_polarity_flag_clk_out_of_range"},    /* Interrupt polarity clock out of range             , */\
   { 0x4ca0, "int_polarity_flag_ovpok"},    /* Interrupt polarity OVP alarm                      , */\
   { 0x4cb0, "int_polarity_flag_qpump_fail"},    /* Interrupt polarity qpump failure                  , */\
   { 0x50d0, "rst_min_vbat"},    /* Reset clipper : auto-clear                        , */\
   { 0x50e0, "sel_vbat"},    /* Battery voltage read out                          , */\
   { 0x50f0, "bypass_clipper"},    /* Bypass battery safeguard                          , */\
   { 0x5111, "dc_prot_time_constant"},    /* Duration of DC level detection                    , */\
   { 0x5131, "dc_prot_level"},    /* DC-level detection                                , */\
   { 0x5150, "bypass_hp"},    /* Bypass HPF                                        , */\
   { 0x5166, "audio_delay"},    /* Set the audio data delay time before send to spkr , */\
   { 0x5257, "gain"},    /* Amplifier gain                                    , */\
   { 0x52f0, "bypass_dly_line"},    /* Bypass the interpolator delay line                , */\
   { 0x5300, "bypass_lp_temp"},    /* Bypass the low pass filter inside temperature sensor, */\
   { 0x5452, "amp_drive"},    /* others : drive setting of AMP powerstage          , */\
   { 0x5810, "hard_mute"},    /* Hard mute - PWM                                   , */\
   { 0x5820, "sel_reson_freq_10k8"},    /* selection of resonance frequency                  , */\
   { 0x58b4, "ssm_depth"},    /* spread spectrum                                   , */\
   { 0x5900, "enable_btl"},    /* PWM modulator configuration                       , */\
   { 0x5a00, "bss_minvbat_select"},    /* min_vbat source select:                           , */\
   { 0x5a13, "bss_dcmcc"},    /* DCMCC initial value                               , */\
   { 0x5a53, "bss_threshold"},    /* BSS threshold Vbat value (reducing gain)          , */\
   { 0x5a91, "bss_attact_rate"},    /* BSS attack rate (reducing gain)                   , */\
   { 0x5ab3, "bss_release_rate"},    /* BSS release rate  (reducing gain)                 , */\
   { 0x5b03, "bss_hold_time"},    /* BSS hold time (reducing gain)                     , */\
   { 0x5b41, "bss_steepness"},    /* BSS steepness (reducing gain)                     , */\
   { 0x5b62, "bss_reduction_limit"},    /* BSS maximum reduction level (reducing gain)       , */\
   { 0x5b93, "bssdc_threshold"},    /* BSS threshold Vbat value (reducing DCMCC)         , */\
   { 0x5bd1, "bssdc_attact_rate"},    /* BSS attack rate  (reducing DCMCC)                 , */\
   { 0x5c03, "bssdc_release_rate"},    /* BSS release rate  (reducing DCMCC)                , */\
   { 0x5c43, "bssdc_hold_time"},    /* BSS hold time (reducing DCMCC)                    , */\
   { 0x5c81, "bssdc_steepness"},    /* Battery supply safeguard steepness (reducing DCMCC), */\
   { 0x5ca2, "bssdc_reduction_limit"},    /* BSS reduction level (reducing DCMCC)              , */\
   { 0x5cd1, "bss_flag_select"},    /* select source digital clip flag                   , */\
   { 0x5f63, "ctrl_attr"},    /* Total gain depending on INPLEV setting (channel 0), */\
   { 0x6005, "idle_power_cal_offset"},    /* Idle power mode detector ctrl cal_offset from gain module , */\
   { 0x6065, "idle_power_zero_lvl"},    /* IIdle power mode zero crossing detection level    , */\
   { 0x60e1, "idle_power_mode"},    /* Idle power mode control                           , */\
   { 0x6105, "idle_power_threshold_lvl"},    /* Idle power mode amplitude trigger level           , */\
   { 0x6165, "idle_power_hold_time"},    /* Idle power mode detector ctrl hold time before low audio is reckoned to be low audio, */\
   { 0x61c0, "disable_idle_power_mode"},    /* Idle power mode detector control                  , */\
   { 0x6802, "tdm_source_mapping"},    /* TDM source mapping                                , */\
   { 0x6842, "tdm_sourcea_frame_sel"},    /* Sensed value A                                    , */\
   { 0x6872, "tdm_sourceb_frame_sel"},    /* Sensed value B                                    , */\
   { 0x68a1, "tdm_source0_clip_sel"},    /* Clip flag information with combined clip flag     , */\
   { 0x68f0, "tdm_enable_loopback"},    /* TDM loopback test                                 , */\
   { 0x6902, "tdm_sourcec_frame_sel"},    /* Sensed value C                                    , */\
   { 0x6932, "tdm_sourced_frame_sel"},    /* Sensed value D                                    , */\
   { 0x6962, "tdm_sourcee_frame_sel"},    /* Sensed value E                                    , */\
   { 0x6b00, "disable_auto_engage"},    /* Disable auto engage                               , */\
   { 0x6b10, "disable_engage"},    /* Disable engage                                    , */\
   { 0x6c69, "spare_out"},    /* Spare control bits for future use                 , */\
   { 0x6d09, "spare_in"},    /* Spare control bit - read only                     , */\
   { 0x6e00, "flag_idle_power_mode"},    /* Idle power mode                                   , */\
   { 0x6f72, "pwm_clip_lvl"},    /* clip detect threshold control                     , */\
   { 0x7060, "boost_intel"},    /* Adaptive boost mode                               , */\
   { 0x7103, "bst_drive"},    /* drive setting of BST powerstage                   , */\
   { 0x7400, "dcdc_disable_ns"},    /* Disable control of noise shaper in DCDC control   , */\
   { 0x7410, "dcdc_disable_mod8bit"},    /* Disable control of reset of noise shaper when 8 bit value for dcdc control occurs, */\
   { 0x7444, "boost_trip_lvl_1st"},    /* Headroom for 1st Adaptive boost trip level, effective only when DCIE is set to 1, */\
   { 0x7494, "boost_track_hysteresis"},    /* Hysteresis for tracking mode of DCDC booster, effective only when DCIE is set to 1, */\
   { 0x74e0, "sel_dcdc_envelope_8fs"},    /* Selection of data for adaptive boost algorithm, effective only when DCIE is set to 1, */\
   { 0x7584, "boost_trip_lvl_track"},    /* Headroom for Tracking Adaptive boost trip level, effective only when DCIE is 1 and DCTRACK is 1, */\
   { 0x75d0, "bypass_dcdc_lpf"},    /* Bypass control of DCDC control low pass filter for quantization noise suppression, */\
   { 0x75f0, "enbl_trip_hyst"},    /* Enable hysteresis on booster trip levels          , */\
   { 0x7687, "scnd_boost_voltage"},    /* Second boost voltage level                        , */\
   { 0x7709, "qpump_aux_en_threshold"},    /* threshold for qpump aux enable, compare with VDDP, default value is 3.6V, */\
   { 0x77a2, "qpump_drive_bst_active"},    /* qpump drvie value when da_bst_active = 1, also used when qpump_use_direct_ctrls=1, */\
   { 0x77d2, "qpump_drive_bst_follow"},    /* qpump drvie value when da_bst_active = 0          , */\
   { 0x7800, "enbl_bst_freqmin"},    /* enable blpa_bst_minfreqreg module                 , */\
   { 0x8050, "cs_gain_control"},    /* Current sense gain control                        , */\
   { 0x8060, "cs_bypass_gc"},    /* Bypasses the CS gain correction                   , */\
   { 0x8087, "cs_gain"},    /* Current sense gain                                , */\
   { 0x8305, "cs_ktemp"},    /* Current sense temperature compensation trimming (1 - VALUE*TEMP)*signal, */\
   { 0x8790, "enbl_dc_filter"},    /* Control for enabling the DC blocking filter for voltage and current sense, */\
   { 0x87a0, "enbl_ana_pre"},    /* Control for enabling the pre-empasis filter for voltage and current sense decimator, */\
   { 0x8850, "vs_gain_control"},    /* Voltage sense VS gain control                     , */\
   { 0x8860, "vs_bypass_gc"},    /* Bypasses the VS gain correction                   , */\
   { 0x8887, "vs_gain"},    /* Voltage sense VS gain                             , */\
   { 0xa007, "key1"},    /* 5Ah, 90d To access KEY1_protected registers (default for engineering), */\
   { 0xa107, "key2"},    /* EFUSE KEY2 register                               , */\
   { 0xa200, "key01_locked"},    /* Indicates KEY1 is locked                          , */\
   { 0xa210, "key02_locked"},    /* Indicates KEY2 is locked                          , */\
   { 0xa220, "efuse_all_0"},    /* Indicates shadow register all 0                   , */\
   { 0xa303, "efuse_man_address_in"},    /* EFUSE address from I2C register for read/writing efuse in manual single word mode, */\
   { 0xa340, "man_copy_efuse_to_iic"},    /* Start copying single word from efuse to I2C efuse register, */\
   { 0xa350, "man_copy_iic_to_efuse"},    /* Start copying single word from I2C efuse register to efuse, */\
   { 0xa360, "auto_copy_efuse_to_iic"},    /* Start copying all the data from efuse to I2C efuse registers, */\
   { 0xa370, "auto_copy_iic_to_efuse"},    /* Start copying data from I2C efuse registers to efuse, */\
   { 0xa40f, "efuse_man_data_out"},    /* EFUSE manual read data                            , */\
   { 0xa50f, "efuse_man_data_in"},    /* write data for EFUSE manual write                 , */\
   { 0xa600, "efuse_vdd_ctrl"},    /* EFUSE AVDD enable                                 , */\
   { 0xb010, "bypass_ocpcounter"},    /* Bypass OCP Counter                                , */\
   { 0xb020, "bypass_glitchfilter"},    /* Bypass glitch filter                              , */\
   { 0xb030, "bypass_ovp"},    /* Bypass OVP                                        , */\
   { 0xb050, "bypass_otp"},    /* Bypass OTP                                        , */\
   { 0xb060, "bypass_lost_clk"},    /* Bypass lost clock detector                        , */\
   { 0xb070, "ctrl_vpalarm"},    /* vpalarm (uvp ovp handling)                        , */\
   { 0xb087, "ocp_threshold"},    /* OCP threshold level                               , */\
   { 0xc000, "rgu_use_direct_ctrls"},    /* Direct control to overrule several functions for testing, */\
   { 0xc010, "rst_datapath"},    /* Direct control for datapath reset                 , */\
   { 0xc020, "rst_cgu"},    /* Direct control for cgu reset                      , */\
   { 0xc030, "fro_use_direct_ctrls"},    /* Direct control to overrule several functions for testing, */\
   { 0xc040, "pll_use_direct_ctrls"},    /* Direct control to overrule several functions for testing, */\
   { 0xc050, "amp_use_direct_ctrls"},    /* Direct control to overrule several functions for testing, */\
   { 0xc060, "bst_use_direct_ctrls"},    /* Direct control to overrule several functions for testing, */\
   { 0xc070, "cvs_use_direct_ctrls"},    /* Direct control to overrule several functions for testing, */\
   { 0xc080, "adc11_use_direct_ctrls"},    /* Direct control to overrule several functions for testing, */\
   { 0xc090, "ref_use_direct_ctrls"},    /* Direct control to overrule several functions for testing, */\
   { 0xc0a0, "atb_use_direct_ctrls"},    /* Direct control to overrule several functions for testing, */\
   { 0xc0b0, "qpump_use_direct_ctrls"},    /* Direct control to overrule several functions for testing (for qpump dynamic control only), */\
   { 0xc0d0, "enbl_ringo"},    /* Enable the ring oscillator for test purpose       , */\
   { 0xc0e0, "enbl_fro"},    /* Enables FRO8M in I2C direct control mode only     , */\
   { 0xc0f0, "atb_trigger"},    /* Trigger ATB controller, auto-clear                , */\
   { 0xc100, "clk_8m_sel"},    /* Select clk_8m(used for manger efuse_ctrl atb_ctrl) source (for testing):, */\
   { 0xc110, "clk_efuse_sel"},    /* Select clk_efuse source((for programming to choose an extranl clock):, */\
   { 0xc120, "clk_atb_sel"},    /* used to select the source of clk_atb(for testing) , */\
   { 0xc130, "enbl_efuse_ss"},    /* Sub-system EFUSE clock gate enable                , */\
   { 0xc20f, "abist_offset"},    /* Offset control for ABIST testing (two's complement), */\
   { 0xc311, "sourcep"},    /* Set OUTA to                                       , */\
   { 0xc331, "sourcen"},    /* Set OUTB to                                       , */\
   { 0xc350, "invertp"},    /* Invert pwma test signal                           , */\
   { 0xc360, "invertn"},    /* Invert pwmb test signal                           , */\
   { 0xc376, "pulselength"},    /* Pulse length setting test input for amplifier (PWM clock 2048/4096 Fs), */\
   { 0xc407, "digimuxa_sel"},    /* DigimuxA input selection control routed to DATAO (see Digimux list for details), */\
   { 0xc487, "digimuxb_sel"},    /* DigimuxB input selection control routed to INT (see Digimux list for details), */\
   { 0xc507, "digimuxc_sel"},    /* DigimuxC input selection control routed to ADS1 (see Digimux list for details), */\
   { 0xc601, "int_ds"},    /* INT pad output drive strength                     , */\
   { 0xc620, "hs_mode"},    /* I2C high speed mode control                       , */\
   { 0xc630, "io_flt_byp"},    /* Bypass the built-in filters of SCL and SDA        , */\
   { 0xc641, "sda_ds"},    /* SDA strength configutation for SDA                , */\
   { 0xc660, "datao_gpio_mode"},    /* DATAO GPIO mode control.                          , */\
   { 0xc670, "datao_oen"},    /* DATAO output control, only effective in GPIO mode., */\
   { 0xc680, "datao_pu"},    /* DATAO pull-up control, only effective in GPIO mode, */\
   { 0xc690, "datao_pd"},    /* DATAO pull-down control, only effective in GPIO mode, */\
   { 0xc6a0, "datao_out"},    /* DATAO output value, only effective in GPIO mode   , */\
   { 0xc6b0, "datao_ie"},    /* DATAO input control, only effective in GPIO mode  , */\
   { 0xc6c1, "datao_ds"},    /* DATAO pad output drive strength                   , */\
   { 0xc700, "enbl_pll"},    /* Enables PLL in PLL direct control mode, use_direct_pll_ctrl set to 1, */\
   { 0xc712, "pll_dpll_clkref_div"},    /* Used for i2c directly control rg_pl200m_dpll_clkref_div[2:0], pl200m dpll referenc clock division, */\
   { 0xc741, "pll_dpll_clkvco_div"},    /* Used for i2c directly control rg_pl200m_dpll_clkvco_div[1:0], pl200m dpll vco clock division, */\
   { 0xc762, "pll_cp_ichg_sel"},    /* PLL configuration:                                , */\
   { 0xc792, "pll_vco_gm_sel"},    /* PLL configuration:                                , */\
   { 0xc7c2, "pll_lf_c0_sel"},    /* PLL configuration:                                , */\
   { 0xc7f0, "pll_dpll_itrim_bypass"},    /* to bypass the DPLL                                , */\
   { 0xc801, "pll_lf_c1_sel"},    /* PLL configuration:                                , */\
   { 0xc821, "pll_lf_c2_sel"},    /* PLL configuration:                                , */\
   { 0xc842, "pll_lf_r0_sel"},    /* PLL configuration:                                , */\
   { 0xc872, "pll_lf_r2_sel"},    /* PLL configuration:                                , */\
   { 0xc934, "pll_pdiv"},    /* PLL PDIV in PLL direct control mode only, use_direct_pll_ctrl set to 1, */\
   { 0xc987, "pll_ndiv"},    /* PLL NDIV in PLL direct control mode only, use_direct_pll_ctrl set to 1, */\
   { 0xca0f, "pll_mdiv"},    /* PLL MDIV in PLL direct control mode only, use_direct_pll_ctrl set to 1, */\
   { 0xcb00, "pll_cp_en"},    /* Connect to rg_pl200m_cp_en,pl200m charge pump enable control bit, */\
   { 0xcb10, "pll_vco_en"},    /* Connect to rg_pl200m_vco_en, pl200m vco power down control bit, */\
   { 0xcb20, "pll_clkout_en"},    /* PLL final clock gate enable:                      , */\
   { 0xcb30, "pll_digital_test_en"},    /* Connect to rg_pl200m_analog_test_en,pl200m analog testing buffer enable, */\
   { 0xcb40, "pll_analog_test_en"},    /* Connect to rg_pl200m_analog_test_en,pl200m analog testing buffer enable, */\
   { 0xcb50, "pll_analog_test_buff_bypass"},    /* Connect to rg_pl200m_test_buff_bypass, pl200m testing buffer directly output, */\
   { 0xcb63, "pll_analog_test_signal_sel"},    /* Connect to rg_pl200m_test_signal_sel[3:0], pl200m testing signal channel selection, */\
   { 0xcba1, "pll_clkout_sel"},    /* Connect to rg_pl200m_clkout_sel[1:0], pl200m clock output channel selection, */\
   { 0xcbc1, "pll_cp_op_bw"},    /* Connect to rg_pl200m_cp_op_bw[1:0], pl200m charge pump opa bandwidth selection word, */\
   { 0xcbe1, "pll_cp_op_enhance"},    /* Charge pump opa output current enhance selection word, */\
   { 0xcc07, "pll_dpll_itrim_manual"},    /* Valid when bypass the DPLL                        , */\
   { 0xcc87, "ana_spare_nokey"},    /* reseved for analog metal ECO                      , */\
   { 0xcd00, "pll_lf_ord4_sel"},    /* Connect to rg_pl200m_lf_ord4_sel, pl200m 4-order mode selection, */\
   { 0xcd87, "ana_spare_key1"},    /* reseved for analog metal ECO                      , */\
   { 0xce42, "sel_pll_startup_time"},    /* PLL startup time selection control                , */\
   { 0xce87, "ana_spare_key2"},    /* reseved for analog metal ECO                      , */\
   { 0xcf02, "adc11b_td_t"},    /* Adc setup time                                    , */\
   { 0xcf31, "adc11b_chn_sel"},    /* Select the input to convert for ADC11 - I2C direct control mode, */\
   { 0xcf64, "adc11b_prog_sample"},    /* ADC11 program sample setting                      , */\
   { 0xcfb0, "adc11b_en"},    /* Enable ADC10 - I2C direct control mode            , */\
   { 0xcfc0, "bypass_lp_vbat"},    /* Bypass control for Low pass filter in batt sensor , */\
   { 0xcfd0, "bypass_lp_vddp"},    /* Bypass control for Low pass filter in vddp sensor , */\
   { 0xd00a, "data_adc11b"},    /* ADC 11 data output data for testing               , */\
   { 0xd0b0, "datao_in"},    /* DATAO input value, only effective in GPIO mode    , */\
   { 0xd180, "enbl_clk_out_of_range"},    /* Clock out of range checker                        , */\
   { 0xd190, "dcdcoff_mode"},    /* DCDC on/off                                       , */\
   { 0xd200, "source_in_testmode"},    /* TDM source in test mode (return only current and voltage sense), */\
   { 0xd263, "test_spare_out1"},    /* Test spare out 1                                  , */\
   { 0xd2a1, "sel_dpwm_signal"},    /* select signal to be tested                        , */\
   { 0xd2c3, "test_spare_out2"},    /* Test spare out 2                                  , */\
   { 0xd300, "atb_reset"},    /* Write 1 to reset analog ATB registers and write 0 to release., */\
   { 0xd312, "bst_fmin"},    /* Minimum frequency regulation word, used in bst_use_direct_ctrls mode, */\
   { 0xd340, "bst_ocpvalid"},    /* data in ocp is latched on the raising edge of this signal, used in bst_use_direct_ctrls mode. Auto-clear., */\
   { 0xd350, "bst_vsetvalid"},    /* data in vset is latched on the raising edge of this signal, used in bst_use_direct_ctrls mode. Auto-clear., */\
   { 0xd360, "bst_fminvalid"},    /* data in fmin is latched on the raising edge of this signal, used in bst_use_direct_ctrls mode. Auto-clear., */\
   { 0xd370, "qpump_drivevalid"},    /* qpump drive is latched on the raising edge of this signal, used in qpump_use_direct_ctrls mode. Auto-clear., */\
   { 0xd380, "qpump_aux_en"},    /* control da_qpump_aux_en, used in qpump_use_direct_ctrls mode., */\
   { 0xd400, "amp_enbl"},    /* ctrl da_amp_en directly when amp_use_directly_ctrls = 1, */\
   { 0xd410, "bst_enbl"},    /* ctrl da_bst_en directly when bst_use_directly_ctrls = 1, */\
   { 0xd427, "bst_vset"},    /* ctrl da_bst_vset[7:0] directly when bst_use_directly_ctrls = 1, */\
   { 0xd4a3, "bst_ocp"},    /* ctrl da_bst_ocp[3:0] directly when bst_use_directly_ctrls = 1, */\
   { 0xd500, "cvs_cs_itf_pu"},    /* power up biasing of current sense interface       , */\
   { 0xd510, "cvs_cs_sdm_pu"},    /* power up biasing of current sense ADC             , */\
   { 0xd520, "cvs_vs_itf_pu"},    /* power up biasing of voltage sense interface       , */\
   { 0xd530, "cvs_vs_sdm_pu"},    /* power up biasing of voltage sense ADC             , */\
   { 0xd540, "cvs_vsitf_cssdm_connect"},    /* connect vs interface to cs sdm (test)             , */\
   { 0xd550, "cvs_csitf_atb_connect"},    /* connect cs interface to atb (test)                , */\
   { 0xd560, "cvs_vsitf_atb_connect"},    /* connect vs interface to atb (test)                , */\
   { 0xd570, "cvs_cssdm_short"},    /* short inputs of cs sdm (test)                     , */\
   { 0xd580, "cvs_vssdm_short"},    /* short inputs of vs sdm (test)                     , */\
   { 0xd590, "cvs_vstress_polarity"},    /* change polarity of integrator capacitors during vstress (test), */\
   { 0xd5a0, "cvs_csls_polarity"},    /* swap polarity of low-side current sense inputs (test), */\
   { 0xd600, "amp_pst_pu"},    /* power up biasing of AMP powerstage                , */\
   { 0xd610, "amp_apwm_pu"},    /* power up biasing of PWM generator                 , */\
   { 0xd620, "amp_ciff_pu"},    /* power up biasing of CIFF integrators              , */\
   { 0xd630, "amp_tzdac_pu"},    /* power up biasing of True-Zero DAC                 , */\
   { 0xd640, "amp_test_rc"},    /* testsignal ALF: capacitor-value related current to VATBP, a resitor to VATBN, */\
   { 0xd650, "amp_test_vtriminmax"},    /* testsignal ALF: connect vtrimax to VATBP and vtrimin to VATBN, */\
   { 0xd660, "amp_test_vlf"},    /* testsignal ALF: connect vlfp to VATBP and vlfn to VATBN, */\
   { 0xd670, "amp_test_vref"},    /* testsignal ALF: connect vref_0v9_tzdac to VATBP and vref_0v9 to VATBN, */\
   { 0xd680, "amp_test_dis_cmff"},    /* disable generation of common-mode-feed-forward (CMFF) signals, */\
   { 0xd6a3, "amp_dtb"},    /* others : signals selection of digital testbus, used when amp_use_direct_ctrls = 1, */\
   { 0xd6e0, "amp_dtb_alf"},    /* selection of loop filter DTB (never together with rg_amp_dtb_pst!), */\
   { 0xd6f0, "amp_dtb_pst"},    /* selection of power stage DTB (never together with rg_amp_dtb_alf!), */\
   { 0xd700, "amp_dis_lowvoltageswing"},    /* testsignal ALF: disable the lowvoltage swing feature, */\
   { 0xd713, "amp_ciff_stress"},    /* testsignal ALF: CIFF integrator capacitor stress vector, */\
   { 0xd750, "amp_test_bist"},    /* testsignal PST: enable CVI mode (single segment)  , */\
   { 0xd760, "amp_test_ocp"},    /* testsignal PST: enable OCP test (single segment)  , */\
   { 0xd770, "amp_test_pstn"},    /* testsignal PST: enable test mode PST n-channel    , */\
   { 0xd780, "amp_test_pstp"},    /* testsignal PST: enable test mode PST p-channel    , */\
   { 0xd790, "amp_test_vstress"},    /* testsignal PST: enable Vstress (increase floating bias voltages), */\
   { 0xd7a0, "atb_hv_vddp"},    /* testsignal PST: connect vddp_sense to vatb_hv     , */\
   { 0xd7b0, "atb_hv_gndp"},    /* testsignal PST: connect gndp_sense to vatb_hv     , */\
   { 0xd800, "bst_bias_pu"},    /* power up biasing of controller and reference voltage generation, */\
   { 0xd810, "qpump_pu"},    /* power up biasing of charge pump                   , */\
   { 0xd820, "bst_timer_pu"},    /* power up biasing of timer in COT controller       , */\
   { 0xd830, "bst_fbck_pu"},    /* power up biasing of feedback DAC in COT controller, */\
   { 0xd840, "bst_pst_pu"},    /* power up biasing of BST powerstage                , */\
   { 0xd853, "bst_dtb"},    /* signal selection of digital testbus               , */\
   { 0xd890, "bst_dtb_cot"},    /* selection of control stage DTB (never together with rg_bst_dtb_pst!), */\
   { 0xd8a0, "bst_dtb_pst"},    /* selection of power stage DTB   (never together with rg_bst_dtb_cot!), */\
   { 0xd8b0, "bst_test_pst"},    /* enable testmode PST (PST responds to da_bst_pst_in), */\
   { 0xd8c0, "bst_test_bist"},    /* enable CVI mode (use powerFET as current source)  , */\
   { 0xd8d0, "bst_test_ocp"},    /* toggle around OCP level in CVI mode               , */\
   { 0xd8e0, "bst_test_vstress"},    /* enable vstress PST and Qpump                      , */\
   { 0xd900, "atb_hv_vddb"},    /* connect VDDB to vatb_hv (TEST3)                   , */\
   { 0xd910, "atb_hv_gndb"},    /* connect GNDB to vatb_hv (TEST3)                   , */\
   { 0xd920, "bst_spare_1"},    /* capacitor stress during testing                   , */\
   { 0xd930, "bst_spare_2"},    /* booster freq. select (00: 1 MHz, 01: 2 MHz, 10: 3 MHz, 11: 4 MHz), */\
   { 0xd940, "bst_spare_3"},    /* booster freq. select (00: 1 MHz, 01: 2 MHz, 10: 3 MHz, 11: 4 MHz), */\
   { 0xd950, "bst_filterenable"},    /* enable singing capacitor effect filter in current DAC, */\
   { 0xd960, "bst_ccm"},    /* force Continuous Conduction Mode                  , */\
   { 0xd970, "atb_vfbck"},    /* connect controller feedback node to ATB output    , */\
   { 0xd980, "bst_test_cot"},    /* timer test mode, forcing "aa_ton_reset" and "aa_toff_reset" to 0, */\
   { 0xd990, "atb_vref_0v9"},    /* connect 0.9 V reference to ATB output             , */\
   { 0xd9a0, "atb_vref_ton"},    /* connect reference voltage for Ton to ATB output   , */\
   { 0xd9b0, "atb_vint_toff"},    /* connect Toff timer integration node to ATB output , */\
   { 0xd9c0, "atb_vint_ton"},    /* connect Ton timer integration node to ATB output  , */\
   { 0xd9d0, "qpump_test_vstress"},    /* Enable voltage stress test in Qpump flycap        , */\
   { 0xd9e0, "qpump_test_en"},    /* Enable Qpump test (freeze clocking)               , */\
   { 0xda01, "bst_fbgain"},    /* gain selection for feedback signal                , */\
   { 0xdb00, "bgr_pu"},    /* power up biasing of BandGap Reference             , */\
   { 0xdb10, "buf_pu"},    /* power up biasing of reference voltage buffers     , */\
   { 0xdb20, "vsup_pu"},    /* power up biasing of supply sensor                 , */\
   { 0xdb30, "bod_pu"},    /* power up biasing of Brown-Out Detector            , */\
   { 0xdb40, "atbp_test1"},    /* connect atbp to outside world (TEST1)             , */\
   { 0xdb50, "atbn_test2"},    /* connect atbn to outside world (TEST2)             , */\
   { 0xdb60, "atbp_vref0v9"},    /* bandgap voltage (also for trimming)               , */\
   { 0xdb70, "atbn_gndd"},    /* ground sense (close to bandgap)                   , */\
   { 0xdb80, "atbp_iref50u"},    /* bias curent (for trim)                            , */\
   { 0xdb90, "atbn_vtemp"},    /* output temperature sensor                         , */\
   { 0xdba0, "atbp_vddd"},    /* vddd sense (close to bod)                         , */\
   { 0xdbb0, "atbn_votp"},    /* analog output otp sensor (can also be forced to test otp flag), */\
   { 0xdbc0, "atbp_pl200m"},    /* all kinds of internal PLL voltages                , */\
   { 0xdbd0, "atbn_adc11b"},    /* ADC11 test in/out                                 , */\
   { 0xdbe0, "atbp_vddpdiv"},    /* divided vddp for VDDP sense                       , */\
   { 0xdbf0, "atbn_vbatdiv"},    /* divided vbat for VBAT sense                       , */\
   { 0xdc00, "atbp_vref1v5"},    /* vref for ADC11                                    , */\
   { 0xdc10, "atbn_vdda"},    /* vdda sense (in references)                        , */\
   { 0xe00f, "sw_profile"},    /* Software profile data                             , */\
   { 0xe10f, "sw_vstep"},    /* Software vstep information                        , */\
   { 0xf002, "cvs_vs_trim"},    /* trim offset of n-side voltage sense interface     , */\
   { 0xf031, "cvs_sdm_offset"},    /* programmed offset of OTA to push idle tones       , */\
   { 0xf053, "amp_ciff_trim"},    /* trim capacitors of CIFF integrators               , */\
   { 0xf103, "amp_ochp_trim"},    /* trim OCP level AMP PST                            , */\
   { 0xf143, "amp_ochn_trim"},    /* trim OCP level AMP PST                            , */\
   { 0xf183, "amp_oclp_trim"},    /* trim OCP level AMP PST                            , */\
   { 0xf1c3, "amp_ocln_trim"},    /* trim OCP level AMP PST                            , */\
   { 0xf203, "bst_ocl_trim"},    /* trim OCP level (low side)                         , */\
   { 0xf243, "bst_och_trim"},    /* trim OCP level (high side)                        , */\
   { 0xf283, "bst_freqtrim"},    /* trim COT timer frequency                          , */\
   { 0xf303, "bgr_vtrim"},    /* trim BGR 900mV reference voltage                  , */\
   { 0xf343, "bgr_itrim"},    /* trim BGR reference current                        , */\
   { 0xf382, "bgr_tctrim"},    /* trim BGR reference voltage temperature coefficient, */\
   { 0xf407, "calibr_osc_delta_ndiv"},    /* others: clk_range_check calibration value (2s-complement)  = ((f_fro8_div8/5000) -200), */\
   { 0xf482, "pll_cp_ib_trim"},    /* connect to rg_pl200m_cp_ib_trim[2:0], pl200m charge pump current bias trim code, */\
   { 0xf4b2, "pll_vco_gm_rs_trim"},    /* connect to rg_pl200m_vco_gm_rs_trim[2:0], pl200m vco gm source resistor of gm selection word, */\
   { 0xf507, "calibr_vout_offset"},    /* DCDC offset calibration 2's complement (key1 protected), */\
   { 0xf603, "calibr_gain"},    /* HW gain module (2's complement)                   , */\
   { 0xf645, "calibr_offset"},    /* Offset for amplifier, HW gain module (2's complement), */\
   { 0xf707, "calibr_cs_gain"},    /* Current sense gain (signed two's complement format), */\
   { 0xf787, "calibr_vs_gain"},    /* Voltage sense VS gain calibration                 , */\
   { 0xf806, "htol_iic_addr"},    /* 7-bit I2C address to be used during HTOL testing  , */\
   { 0xf870, "htol_iic_addr_en"},    /* HTOL I2C address enable control                   , */\
   { 0xf887, "calibr_temp_offset"},    /* Temperature offset 2's compliment (key1 protected), */\
   { 0xf920, "lock_bypass_clipper"},    /* Disable function bypass_clipper                   , */\
   { 0xf930, "lock_max_dcdc_voltage"},    /* Limit Max DCDC voltage                            , */\
   { 0xf940, "no_of_bumps"},    /* 30 or 36 bump version                             , */\
   { 0xf986, "calibr_temp_gain"},    /* Temperature gain 2's compliment (key1 protected)  , */\
   { 0xfa0f, "efusedataA"},    /* EFUSEdataA (key1 protected)                       , */\
   { 0xfb0f, "efusedataB"},    /* EFUSEdataB (key1 protected)                       , */\
   { 0xfc0f, "efusedataC"},    /* EFUSEdataC (key1 protected)                       , */\
   { 0xfd0f, "efusedataD"},    /* EFUSEdataD (key1 protected)                       , */\
   { 0xfe0f, "efusedataE"},    /* EFUSEdataE (key1 protected)                       , */\
   { 0xff0f, "efusedataF"},    /* EFUSEdataF (key1 protected)                       , */\
   { 0xffff,"Unknown bitfield enum" }    /* not found */\
};
#if 0
enum tfa9865_irq {
	tfa9865_irq_max = -1,
	tfa9865_irq_all = -1 /* all irqs */};
#endif
#define TFA9865_IRQ_NAMETABLE static tfaIrqName_t Tfa9865IrqNames[]= {\
};
#endif /* _TFA9865_TFAFIELDNAMES_A2_H */
