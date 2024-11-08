// ADS1158 Register Addresses
#define CONFIG0_REG 0x00
#define CONFIG1_REG 0x01
#define MUXSCH_REG 0x02
#define MUXDIF_REG 0x03
#define MUXSG0_REG 0x04
#define MUXSG1_REG 0x05
#define SYSRED_REG 0x06
#define GPIOC_REG 0x07
#define GPIOD_REG 0x08
#define ID_REG 0x09

// CONFIG0 Register Bits
#define CONFIG0_SPIRST 0x40  // Bit 6: SPI Interface Reset Timer
#define CONFIG0_MUXMOD 0x20  // Bit 5: Multiplexer Mode (Auto-Scan or Fixed-Channel)
#define CONFIG0_BYPAS 0x10   // Bit 4: Bypass (Internal/External MUX connection)
#define CONFIG0_CLKENB 0x08  // Bit 3: Clock Output Enable
#define CONFIG0_CHOP 0x04    // Bit 2: Chopper Enable
#define CONFIG0_STAT 0x02    // Bit 1: Status Byte Enable

// CONFIG1 Register Bits
#define CONFIG1_IDLMOD 0x80  // Bit 7: Idle Mode (Low Power or Normal Operation)
#define CONFIG1_DLY2 0x40    // Bit 6: Conversion Delay Bit 2
#define CONFIG1_DLY1 0x20    // Bit 5: Conversion Delay Bit 1
#define CONFIG1_DLY0 0x10    // Bit 4: Conversion Delay Bit 0
#define CONFIG1_SBCS1 0x08   // Bit 3: Bias Sense Current Bit 1
#define CONFIG1_SBCS0 0x04   // Bit 2: Bias Sense Current Bit 0
#define CONFIG1_DRATE1 0x02  // Bit 1: Data Rate Bit 1
#define CONFIG1_DRATE0 0x01  // Bit 0: Data Rate Bit 0

// MUXSCH Register (Fixed Channel Selection) Bits
#define MUXSCH_AINP3 0x08  // Bit 3: AINP[3:0] (Positive Input Channel Selection)
#define MUXSCH_AINP2 0x04  // Bit 2: AINP[3:0] (Positive Input Channel Selection)
#define MUXSCH_AINP1 0x02  // Bit 1: AINP[3:0] (Positive Input Channel Selection)
#define MUXSCH_AINP0 0x01  // Bit 0: AINP[3:0] (Positive Input Channel Selection)

#define MUXSCH_AINN3 0x80  // Bit 7: AINN[3:0] (Negative Input Channel Selection)
#define MUXSCH_AINN2 0x40  // Bit 6: AINN[3:0] (Negative Input Channel Selection)
#define MUXSCH_AINN1 0x20  // Bit 5: AINN[3:0] (Negative Input Channel Selection)
#define MUXSCH_AINN0 0x10  // Bit 4: AINN[3:0] (Negative Input Channel Selection)

// MUXDIF Register (Differential Input Mode) Bits
#define MUXDIF_DIFF7 0x80  // Bit 7: DIFF7 (Differential Input Selection)
#define MUXDIF_DIFF6 0x40  // Bit 6: DIFF6 (Differential Input Selection)
#define MUXDIF_DIFF5 0x20  // Bit 5: DIFF5 (Differential Input Selection)
#define MUXDIF_DIFF4 0x10  // Bit 4: DIFF4 (Differential Input Selection)
#define MUXDIF_DIFF3 0x08  // Bit 3: DIFF3 (Differential Input Selection)
#define MUXDIF_DIFF2 0x04  // Bit 2: DIFF2 (Differential Input Selection)
#define MUXDIF_DIFF1 0x02  // Bit 1: DIFF1 (Differential Input Selection)
#define MUXDIF_DIFF0 0x01  // Bit 0: DIFF0 (Differential Input Selection)

// MUXSG0 Register (Single-Ended Input Selection for AIN0-AIN7)
#define MUXSG0_AIN7 0x80  // Bit 7: AIN7 Selection
#define MUXSG0_AIN6 0x40  // Bit 6: AIN6 Selection
#define MUXSG0_AIN5 0x20  // Bit 5: AIN5 Selection
#define MUXSG0_AIN4 0x10  // Bit 4: AIN4 Selection
#define MUXSG0_AIN3 0x08  // Bit 3: AIN3 Selection
#define MUXSG0_AIN2 0x04  // Bit 2: AIN2 Selection
#define MUXSG0_AIN1 0x02  // Bit 1: AIN1 Selection
#define MUXSG0_AIN0 0x01  // Bit 0: AIN0 Selection

// MUXSG1 Register (Single-Ended Input Selection for AIN8-AIN15)
#define MUXSG1_AIN15 0x80  // Bit 7: AIN15 Selection
#define MUXSG1_AIN14 0x40  // Bit 6: AIN14 Selection
#define MUXSG1_AIN13 0x20  // Bit 5: AIN13 Selection
#define MUXSG1_AIN12 0x10  // Bit 4: AIN12 Selection
#define MUXSG1_AIN11 0x08  // Bit 3: AIN11 Selection
#define MUXSG1_AIN10 0x04  // Bit 2: AIN10 Selection
#define MUXSG1_AIN9 0x02   // Bit 1: AIN9 Selection
#define MUXSG1_AIN8 0x01   // Bit 0: AIN8 Selection

// SYSRED Register (System Readings Configuration) Bits
#define SYSRED_REF 0x20     // Bit 5: Reference Reading Enable
#define SYSRED_GAIN 0x10    // Bit 4: Gain Reading Enable
#define SYSRED_TEMP 0x08    // Bit 3: Temperature Reading Enable
#define SYSRED_VCC 0x04     // Bit 2: VCC Reading Enable
#define SYSRED_OFFSET 0x01  // Bit 0: Offset Reading Enable

// GPIO Control Registers
#define GPIOC_CIO7 0x80  // Bit 7: GPIO Control for CIO7
#define GPIOC_CIO6 0x40  // Bit 6: GPIO Control for CIO6
#define GPIOC_CIO5 0x20  // Bit 5: GPIO Control for CIO5
#define GPIOC_CIO4 0x10  // Bit 4: GPIO Control for CIO4
#define GPIOC_CIO3 0x08  // Bit 3: GPIO Control for CIO3
#define GPIOC_CIO2 0x04  // Bit 2: GPIO Control for CIO2
#define GPIOC_CIO1 0x02  // Bit 1: GPIO Control for CIO1
#define GPIOC_CIO0 0x01  // Bit 0: GPIO Control for CIO0

#define GPIOD_DIO7 0x80  // Bit 7: GPIO Direction for DIO7
#define GPIOD_DIO6 0x40  // Bit 6: GPIO Direction for DIO6
#define GPIOD_DIO5 0x20  // Bit 5: GPIO Direction for DIO5
#define GPIOD_DIO4 0x10  // Bit 4: GPIO Direction for DIO4
#define GPIOD_DIO3 0x08  // Bit 3: GPIO Direction for DIO3
#define GPIOD_DIO2 0x04  // Bit 2: GPIO Direction for DIO2
#define GPIOD_DIO1 0x02  // Bit 1: GPIO Direction for DIO1
#define GPIOD_DIO0 0x01  // Bit 0: GPIO Direction for DIO0

// ID Register Bits (Device ID)
#define ID_DEFAULT 0x9B  // Default ID Value
