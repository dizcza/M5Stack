// All credits to https://github.com/CelliesProjects/wm8978-esp32

#include <stdio.h>
#include <Arduino.h>
#include <Wire.h>
#include "WM8978.h"

/* static */ WM8978* WM8978::instance;

WM8978::WM8978() {
    if (!instance) instance = this;
}


static const char *_TAG = "wm8978";

// WM8978 register value buffer zone (total 58 registers 0 to 57), occupies 116 bytes of memory
// Because the IIC WM8978 operation does not support read operations, so save all the register values in the local
// Write WM8978 register, synchronized to the local register values, register read, register directly back locally stored value.
// Note: WM8978 register value is 9, so use uint16_t storage.
static uint16_t REGVAL_TBL[58] = {
  0x0000, 0x0000, 0x0000, 0x0000, 0x0050, 0x0000, 0x0140, 0x0000,
  0x0000, 0x0000, 0x0000, 0x00FF, 0x00FF, 0x0000, 0x0100, 0x00FF,
  0x00FF, 0x0000, 0x012C, 0x002C, 0x002C, 0x002C, 0x002C, 0x0000,
  0x0032, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0038, 0x000B, 0x0032, 0x0000, 0x0008, 0x000C, 0x0093, 0x00E9,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0003, 0x0010, 0x0010, 0x0100,
  0x0100, 0x0002, 0x0001, 0x0001, 0x0039, 0x0039, 0x0039, 0x0039,
  0x0001, 0x0001
};

bool  WM8978::begin() {
  Wire.beginTransmission(WM8978_ADDR);
  const uint8_t error = Wire.endTransmission();
  if (error) {
    ESP_LOGE(_TAG, "No WM8978 dac @ i2c address: 0x%X", WM8978_ADDR);
    return false;
  }
  const int err = Init();
  if (err) {
    ESP_LOGE(_TAG, "WM8978 init err: 0x%X", err);
    return false;
  }
  cfgI2S(2, 0);       //Philips 16bit
  cfgADDA(1, 1);      //Enable ADC DAC
  cfgInput(0, 0, 0);  //mic, linein, aux - Note: M5Stack node has only internal microphones connected
  setMICgain(0);
  setLINEINgain(0);
  setAUXgain(0);
  setSPKvol(0);       //0-63
  setHPvol(0, 0);     //0-63
  set3Ddir(0);
  setEQ1(0, 24);
  setEQ2(0, 24);
  setEQ3(0, 24);
  setEQ4(0, 24);
  setEQ5(0, 24);
  cfgOutput(1, 0);    //Output enabled, bypass disabled
  return true;
}

bool WM8978::begin(const uint8_t sda, const uint8_t scl, const uint32_t frequency) {
  ESP_LOGD(_TAG, "i2c init sda=%i scl=%i frequency=%i", sda, scl, frequency);
  if (!Wire.begin((int) sda, (int) scl, frequency)) {
    ESP_LOGE(_TAG, "Wire setup error sda=%i scl=%i frequency=%i", sda, scl, frequency);
    return false;
  }
  return begin();
}

// Return value: 0, initialization is normal
//     Other, error code
uint8_t WM8978::Init(void) {
  uint8_t res;
  res = wm8978(0, 0); // soft reset WM8978
  if (res) return 1;  // Failed to send command, WM8978 is abnormal
  //The following are general settings
  wm8978(1, 0x1B);    // R1, MICEN is set to 1 (MIC enable), BIASEN is set to 1 (simulator work), VMIDSEL[1:0] is set to: 11 (5K)
  wm8978(2, 0x1B);    // R2, ROUT1, LOUT1 output enable (headphones can work), BOOSTENR, BOOSTENL enable
  wm8978(3, 0x6C);    // R3, LOUT2, ROUT2 output enable (speaker work), RMIX, LMIX enable
  wm8978(6, 0);       // R6, MCLK is provided by external
  wm8978(43, 1 << 4); // R43, INVROUT2 reverse, drive the speaker
  wm8978(47, 1 << 8); // R47 setting, PGABOOSTL, left channel MIC gain 20 times
  wm8978(48, 1 << 8); // R48 setting, PGABOOSTR, right channel MIC gain 20 times
  wm8978(49, 1 << 1); // R49, TSDEN, turn on overheat protection
  wm8978(10, 1 << 3); // R10, SOFTMUTE off, 128x sampling, best SNR
  wm8978(14, 1 << 3); // R14, ADC 128x sampling rate
  return 0;
}

// WM8978 write register
// reg: register address
// val: the value to be written to the register
// Return value: 0, success;
// Other, error code
uint8_t WM8978::wm8978(uint8_t reg, uint16_t val) {
  char buf[2];
  buf[0] = (reg << 1) | ((val >> 8) & 0x01);
  buf[1] = val & 0xFF;
  Wire.beginTransmission(WM8978_ADDR); // Send data to the slave with device number 4
  Wire.write((const uint8_t*)buf, 2);
  Wire.endTransmission();     // Stop sending
  REGVAL_TBL[reg] = val; // Save the register value to the local
  return 0;
}

// WM8978 read register
// Reads the value  of the local register buffer zone
// reg: Register Address
// Return Value: Register value
uint16_t WM8978::wm8978(uint8_t reg) {
  return REGVAL_TBL[reg];
}

// WM8978 DAC/ADC configuration
// adcen: adc enable (1)/disable (0)
// dacen: dac enable (1)/disable (0)
void  WM8978::cfgADDA(uint8_t dacen, uint8_t adcen) {
  uint16_t regval;
  regval = wm8978(WM8978_POWER_MANAGEMENT_3);
  if (dacen) regval |= 3 << 0;   // The lowest 2 bits of R3 are set to 1, and DACR&DACL is turned on
  else regval &= ~( 3 << 0 );   // Clear the lowest 2 bits of R3, turn off DACR&DACL.
  wm8978(WM8978_POWER_MANAGEMENT_3, regval);
  regval = wm8978(WM8978_POWER_MANAGEMENT_2);
  if (adcen) regval |= 3 << 0;  // The lowest 2 bits of R2 are set to 1, and ADCR&ADCL is turned on
  else regval &= ~( 3 << 0 );   // The lowest 2 bits of R2 are cleared, and ADCR&ADCL is turned off.
  wm8978(WM8978_POWER_MANAGEMENT_2, regval);
}

// WM8978 input channel configuration
//micen:MIC enable(1)/disable(0)
//lineinen:Line In enable(1)/disable(0)
//auxen:aux enable(1)/disable(0)
void  WM8978::cfgInput(uint8_t micen, uint8_t lineinen, uint8_t auxen) {
  uint16_t regval;
  regval = wm8978(WM8978_POWER_MANAGEMENT_2);
  if (micen) regval |= 3 << 2;  // Enable INPPGAENR, INPPGAENL (PGA enlargement of MIC)
  else regval &= ~(3 << 2);     // Disable INPPGAENR,INPPGAENL.
  wm8978(WM8978_POWER_MANAGEMENT_2, regval);
  regval = wm8978(WM8978_INPUT_CONTROL);
  if (micen) regval |= 3 << 4 | 3 << 0; // Enable LIN2INPPGA, LIP2INPGA, RIN2INPPGA, RIP2INPGA.
  else regval &= ~(3 << 4 | 3 << 0);    // Disable LIN2INPPGA, LIP2INPGA, RIN2INPPGA, RIP2INPGA.
  wm8978(WM8978_INPUT_CONTROL, regval);
  if (lineinen) setLINEINgain(5); // LINE IN gain 0dB
  else setLINEINgain(0);          // Disable LINE IN
  if (auxen) setAUXgain(7);       // AUX gain 6dB 
  else setAUXgain(0);             // Disable AUX input
}

// WM8978 output configuration
// dacen: DAC output (playback) on (1)/off (0)
// bpsen: Bypass output (recording, including MIC, LINE IN, AUX, etc.) on (1) / off (0)
void  WM8978::cfgOutput(uint8_t dacen, uint8_t bpsen) {
  uint16_t regval = 0;
  if (dacen) regval |= 1 << 0;  // DAC output enable
  if (bpsen) {
    regval |= 1 << 1; // BYPASS enable
    regval |= 5 << 2; // 0dB gain
  }
  wm8978(WM8978_LEFT_MIXER_CONTROL, regval);
  wm8978(WM8978_RIGHT_MIXER_CONTROL, regval);
}

// WM8978 MIC gain setting (not including BOOST 20dB, MIC-->ADC input part gain)
// gain: 0~63, corresponding to -12dB~35.25dB, 0.75dB/Step
void WM8978::setMICgain(uint8_t gain) {
  gain &= 0x3F;
  wm8978(WM8978_LEFT_INP_PGA_CONTROL, gain);           // R45, left channel PGA setting
  wm8978(WM8978_RIGHT_INP_PGA_CONTROL, gain | 1 << 8);  // R46, right channel PGA setting
}

// WM8978 L2/R2 (aka Line In) gain setting (L2/R2-->ADC input gain)
// gain: 0~7, 0 means channel prohibited, 1~7, corresponding to -12dB~6dB, 3dB/Step
void WM8978::setLINEINgain(uint8_t gain) {
  uint16_t regval;
  gain &= 0x07;
  regval = wm8978(WM8978_LEFT_ADC_BOOST_CONTROL);
  regval &= ~( 7 << 4 );  // Clear the original settings
  wm8978(WM8978_LEFT_ADC_BOOST_CONTROL, regval | gain << 4);
  regval = wm8978(WM8978_RIGHT_ADC_BOOST_CONTROL);
  regval &= ~( 7 << 4 );  // Clear the original settings
  wm8978(WM8978_RIGHT_ADC_BOOST_CONTROL, regval | gain << 4);
}

// WM8978 AUXR, AUXL (PWM audio part) gain setting (AUXR/L-->ADC input part gain)
// gain: 0~7, 0 means channel prohibited, 1~7, corresponding to -12dB~6dB, 3dB/Step
void WM8978::setAUXgain(uint8_t gain) {
  uint16_t regval;
  gain &= 0x07;
  regval = wm8978(WM8978_LEFT_ADC_BOOST_CONTROL);
  regval &= ~( 7 << 0 );  // Clear the original settings
  wm8978(WM8978_LEFT_ADC_BOOST_CONTROL, regval | gain << 0);
  regval = wm8978(WM8978_RIGHT_ADC_BOOST_CONTROL);
  regval &= ~( 7 << 0 );  // Clear the original settings
  wm8978(WM8978_RIGHT_ADC_BOOST_CONTROL, regval | gain << 0);
}

// Set I2S working mode
// fmt:0, LSB (right justified); 1, MSB (left justified); 2, Philips standard I2S; 3, PCM/DSP;
// len:0, 16 bits; 1, 20 bits; 2, 24 bits; 3, 32 bits;
void WM8978::cfgI2S(uint8_t fmt, uint8_t len) {
  fmt &= 0x03;
  len &= 0x03; // Limited range
  wm8978(WM8978_AUDIO_INTERFACE, (fmt << 3 ) | (len << 5 )); // R4, WM8978 working mode setting
}

// Set the volume of the left and right channels of the headset
// voll: left channel volume (0~63)
// volr: Right channel volume (0~63)
void  WM8978::setHPvol(uint8_t voll, uint8_t volr) {
  voll &= 0x3F;
  volr &= 0x3F; // Limited range
  if (voll == 0) voll |= 1 << 6;  //When the volume is 0, directly mute
  if (volr == 0) volr |= 1 << 6;  //When the volume is 0, directly mute
  wm8978(WM8978_LOUT1_HP_CONTROL, voll);               // R52, headphone left channel volume setting
  wm8978(WM8978_ROUT1_HP_CONTROL, volr | ( 1 << 8 ));  // R53, headphone right channel volume setting, synchronous update (HPVU=1)
}

// Set the speaker volume
// voll: left channel volume (0~63)
void WM8978::setSPKvol(uint8_t volx) {
  volx &= 0x3F; // Limited range
  if (volx == 0) volx |= 1 << 6;  //When the volume is 0, mute directly
  wm8978(WM8978_LOUT2_SPK_CONTROL, volx);               // R54, speaker left channel volume setting
  wm8978(WM8978_ROUT2_SPK_CONTROL, volx | ( 1 << 8 ));  // R55, speaker right channel volume setting, synchronous update (SPKVU=1)
}

// Set 3D surround sound
// depth:0~15 (3D intensity, 0 is the weakest, 15 is the strongest)
void WM8978::set3D(uint8_t depth) {
  depth &= 0xF; // Limit the range
  wm8978(WM8978_3D_CONTROL, depth);  // R41, 3D surround setting
}

// Set the direction of EQ/3D
// dir:0, works in ADC
//     1, works in DAC (default)
void WM8978::set3Ddir(uint8_t dir) {
  uint16_t regval;
  regval = wm8978(WM8978_EQ1);
  if (dir) regval |= 1 << 8;
  else regval &= ~(1 << 8);
  wm8978(WM8978_EQ1, regval); // R18, the 9th bit of EQ1 controls the EQ/3D direction
}

// cfreq: cut-off frequency, 0~3, corresponding to: 80/105/135/175Hz
// gain: gain, 0~24, corresponding to -12~+12dB
void WM8978::setEQ1(uint8_t cfreq, uint8_t gain) {
  uint16_t regval;
  cfreq &= 0x3; // Limited range
  if (gain > 24) gain = 24;
  gain = 24 - gain;
  regval = wm8978(WM8978_EQ1);
  regval &= 0X100;
  regval |= cfreq << 5; // Set the cutoff frequency
  regval |= gain;       // Set gain
  wm8978(WM8978_EQ1, regval);   // R18, EQ1 setting
}

// cfreq: center frequency, 0~3, corresponding to: 230/300/385/500Hz
// gain: gain, 0~24, corresponding to -12~+12dB
void WM8978::setEQ2(uint8_t cfreq, uint8_t gain) {
  uint16_t regval = 0;
  cfreq &= 0x3; // Limited range
  if (gain > 24) gain = 24;
  gain = 24 - gain;
  regval |= cfreq << 5; // Set the cutoff frequency
  regval |= gain;       // Set gain
  wm8978(WM8978_EQ2, regval);   // R19, ​​EQ2 settings
}

// cfreq: center frequency, 0~3, corresponding to: 650/850/1100/1400Hz
// gain: gain, 0~24, corresponding to -12~+12dB
void WM8978::setEQ3(uint8_t cfreq, uint8_t gain) {
  uint16_t regval = 0;
  cfreq &= 0x3; // Limited range
  if (gain > 24) gain = 24;
  gain = 24 - gain;
  regval |= cfreq << 5; // Set the cutoff frequency
  regval |= gain;       // Set gain
  wm8978(WM8978_EQ3, regval);   // R20, EQ3 settings
}

// cfreq: center frequency, 0~3, corresponding to: 1800/2400/3200/4100Hz
// gain: gain, 0~24, corresponding to -12~+12dB
void WM8978::setEQ4(uint8_t cfreq, uint8_t gain) {
  uint16_t regval = 0;
  cfreq &= 0x3; // Limited range
  if (gain > 24) gain = 24;
  gain = 24 - gain;
  regval |= cfreq << 5; // Set the cutoff frequency
  regval |= gain;       // Set gain
  wm8978(WM8978_EQ4, regval);   // R21, EQ4 settings
}

// cfreq: center frequency, 0~3, corresponding to: 5300/6900/9000/11700Hz
// gain: gain, 0~24, corresponding to -12~+12dB
void WM8978::setEQ5(uint8_t cfreq, uint8_t gain) {
  uint16_t regval = 0;
  cfreq &= 0x3; // Limited range
  if (gain > 24) gain = 24;
  gain = 24 - gain;
  regval |= cfreq << 5; // Set the cutoff frequency
  regval |= gain;       // Set gain
  wm8978(WM8978_EQ5, regval);   // R22, EQ5 settings
}

void WM8978::setALC(uint8_t enable, uint8_t maxgain, uint8_t mingain) {
  uint16_t regval;
  if (maxgain > 7) maxgain = 7;
  if (mingain > 7) mingain = 7;
  regval = wm8978(WM8978_ALC_CONTROL_1);
  if (enable) regval |= (3 << 7);
  regval |= (maxgain << 3 ) | (mingain << 0);
  wm8978(WM8978_ALC_CONTROL_1, regval);
}

void WM8978::setNoise(uint8_t enable, uint8_t gain) {
  uint16_t regval;
  if (gain > 7) gain = 7;
  regval = wm8978(WM8978_NOISE_GATE);
  regval = (enable << 3);
  regval |= gain;
  wm8978(WM8978_NOISE_GATE, regval);
}

void WM8978::setPLL(uint32_t k, uint8_t n) {
	REGVAL_TBL[WM8978_ADDITIONAL_CONTROL] &= ~(bit3 | bit2 | bit1);
	REGVAL_TBL[WM8978_ADDITIONAL_CONTROL] |= (bit3 | bit1);
	wm8978(WM8978_ADDITIONAL_CONTROL, REGVAL_TBL[WM8978_ADDITIONAL_CONTROL]); //sr 8K
	REGVAL_TBL[WM8978_POWER_MANAGEMENT_1] |= bit5;  //enable pll
	wm8978(WM8978_POWER_MANAGEMENT_1, REGVAL_TBL[WM8978_POWER_MANAGEMENT_1]);
	REGVAL_TBL[WM8978_PLL_N] |= bit4; //prescale enable /2
	REGVAL_TBL[WM8978_PLL_N] &= 0x1f0;
	REGVAL_TBL[WM8978_PLL_N] |= n;  //7
	wm8978(WM8978_PLL_N, REGVAL_TBL[WM8978_PLL_N]);
	//k=EE009F
	REGVAL_TBL[WM8978_PLL_K1] = (k>>18);
	wm8978(WM8978_PLL_K1, REGVAL_TBL[WM8978_PLL_K1]);
	REGVAL_TBL[WM8978_PLL_K2] = (k>>9);
	wm8978(WM8978_PLL_K2, REGVAL_TBL[WM8978_PLL_K2]);
	REGVAL_TBL[WM8978_PLL_K3] = k;					
	wm8978(WM8978_PLL_K3, REGVAL_TBL[WM8978_PLL_K3]);
}

void WM8978::setLoopback() {
  REGVAL_TBL[WM8978_COMPANDING_CONTROL] |= bit0; //start loopback
  wm8978(WM8978_COMPANDING_CONTROL, REGVAL_TBL[WM8978_COMPANDING_CONTROL]);
}

void WM8978::setClockCfg() {
  //#define REG_CLOCK_GEN			((uint16_t)(6 << 9))
  #define CLKSEL_PLL			(1 << 8)	// Default value
  #define MCLK_DIV2				(2 << 5)	// Default value
  #define BCLK_DIV8				(3 << 2)
  #define MS						  (1)
	uint16_t regval = CLKSEL_PLL | MCLK_DIV2 | BCLK_DIV8;
	regval &= MS;
	wm8978(WM8978_CLOCKING, regval);
}
