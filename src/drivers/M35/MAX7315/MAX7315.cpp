#include "MAX7315.h"

/* static */ MAX7315* MAX7315::instance;

MAX7315::MAX7315(/* const uint8_t addr */) {
  if (!instance) instance = this;
  this->addr = MAX7315_I2C_ADDRESS;
  this->mixedMode    = true;
  this->blinkEnable  = false;
  this->blinkPhase   = false;
  this->enableInt    = true;
  this->enableGlobal = true;
  this->port     = 0xFF;
  this->phase0   = 0xFF;
  this->phase1   = 0xFF;
  this->masterO8 = 0x0F;  
  this->config   = 0x0C;
  this->globalIntensity = 0x0F;
  this->masterIntensity = 0x00;
  this->intensVal[0] = 0xFF;
  this->intensVal[1] = 0xFF;
  this->intensVal[2] = 0xFF;
  this->intensVal[3] = 0xFF;
}

void MAX7315::begin(const bool mixedMode, const bool blinkEnable, const bool blinkPhase) {
  this->mixedMode   = mixedMode;
  this->blinkEnable = blinkEnable;
  this->blinkPhase  = blinkPhase;
  log_d("MAX7315 initialized...");
  enableInterrupt(false);
  flipBlink(false);
  setBlink(false);
  setGlobal(false);
  setGlobalIntensity(0xF);
  setMasterIntensity(0xF);
}

void MAX7315::setGlobal(bool enableGlobal) {
  if(this->enableGlobal == enableGlobal) return;
  this->enableGlobal = enableGlobal;
  if (enableGlobal) {
    this->config |= MAX7315_CONF_MASK_GLINT;
  } else {
    this->config &= ~MAX7315_CONF_MASK_GLINT;
  }
  max7315(MAX7315_CONFIGURATION, this->config);
} 

void MAX7315::setBlink(bool blinkEnable) {
  if(this->blinkEnable == blinkEnable) return;
  this->blinkEnable = blinkEnable;
  if (blinkEnable) {
    this->config |= MAX7315_CONF_MASK_BLEN;
  } else {
    this->config &= ~MAX7315_CONF_MASK_BLEN;
  }
  max7315(MAX7315_CONFIGURATION, this->config);
}    
 
void MAX7315::flipBlink(bool blinkPhase) {
  if(this->blinkPhase == blinkPhase) return;
  this->blinkPhase = blinkPhase;
  if (blinkPhase) {
    this->config |= MAX7315_CONF_MASK_BLFL;
  } else {
    this->config &= ~MAX7315_CONF_MASK_BLFL;
  }
  max7315(MAX7315_CONFIGURATION, this->config);
}

void MAX7315::enableInterrupt(bool enableInt) {
  if(this->enableInt == enableInt) return;
  this->enableInt = enableInt;
  if (enableInt) {
    this->config |= MAX7315_CONF_MASK_ENINT;
  } else {
    this->config &= ~MAX7315_CONF_MASK_ENINT;
  }
  max7315(MAX7315_CONFIGURATION, this->config);
}

void MAX7315::setGlobalIntensity(uint8_t intensity) {
  intensity &= 0x0F;
  if(this->globalIntensity == intensity) return;
  this->globalIntensity = intensity;
  this->masterO8 &= 0xF0;
  this->masterO8 |= intensity;
  max7315(MAX7315_MASTER_O8GLOB, this->masterO8);
}

void MAX7315::setMasterIntensity(uint8_t intensity) {
  intensity &= 0x0F;
  if(this->masterIntensity == intensity) return;
  this->masterIntensity = intensity;
  this->masterO8 &= 0x0F;
  this->masterO8 |= (intensity << 4);
  max7315(MAX7315_MASTER_O8GLOB, this->masterO8);
}

void MAX7315::setPinMode(uint8_t pin, uint8_t mode) {
  if ( pin < 8 ){
    if (mode == OUTPUT) {
      // Write static pin intensity of the individual output
      if(this->mixedMode){
        uint8_t regOffset = pin >> 1;
        uint8_t intensityMask = 0x0F;
        if(pin & 0x01) intensityMask = 0xF0;
        this->intensVal[regOffset] |= intensityMask;
        max7315((MAX7315_INTENSITY_P1P0 + regOffset), this->intensVal[regOffset]);
      }    
      // Clear the pin on the configuration register for output
      this->port &= ~(1 << pin);
      // Write the configuration of the individual pins as inputs or outputs
      max7315(MAX7315_PORTS_CONFIG, this->port);
    } else {      
      // Set the pin on the configuration register for input
      this->port |= (1 << pin);
      // Write the configuration of the individual pins as inputs or outputs
      max7315(MAX7315_PORTS_CONFIG, this->port);
    }
  } else {
      this->config &= 0xCF;
      this->config |= (mode << 4);
      max7315(MAX7315_CONFIGURATION, this->config);
  }
}

uint8_t MAX7315::getLevel(uint8_t pin) {
  uint8_t buff = 0;
  buff = max7315(MAX7315_READ_INPUT);
  return (buff & (1 << pin)) ? HIGH : LOW;
}

void MAX7315::setLevel(uint8_t pin, bool level) {
  if (level) {
    // Set the pin HIGH on the output register
    this->phase0 |= (1 << pin);
  } else {
    // Set the pin LOW on the output register
    this->phase0 &= ~(1 << pin);
  }
  // Write the status of the pins on the output register
  max7315(MAX7315_BLINK_PHASE0, this->phase0);
}

void MAX7315::setPhase1(uint8_t pin, bool level) {
  if (level) {
    // Set the pin HIGH on the output register
    this->phase1 |= (1 << pin);
  } else {
    // Set the pin LOW on the output register
    this->phase1 &= ~(1 << pin);
  }
  max7315(MAX7315_BLINK_PHASE1, this->phase1);
}

// Valid pin numbers - 0-8, 10, 32, 54, 76
bool MAX7315::setIntensity(uint8_t pin, uint8_t value) {
  uint8_t regOffset = 0;

  if (pin < 8) {            // Pins 0-7
    uint8_t regOffset = (pin >> 1); // to get intensity register  
    if ((pin & 1) == 1) {
      this->intensVal[regOffset] = (this->intensVal[regOffset] & 0x0F) | (value << 4);      //odd      
    } else {
      this->intensVal[regOffset] = (this->intensVal[regOffset] & 0xF0) | (value & 0x0F);    //0,2,4,6 
    }
    max7315((MAX7315_INTENSITY_P1P0 + regOffset), this->intensVal[regOffset]);
    return true;
  } else if (pin == 8) {    // Pin O8
    // Put value's 4 LSB to O8/Global intensity register
    this->masterO8 = (this->masterO8 & 0xF0) | (value & 0x0F);
    max7315(MAX7315_MASTER_O8GLOB, this->masterO8);  // O8 is not Int
    return true;
  } else {                                    // Pin pairs: 10, 32, 54, 76
    bool success = true;
    switch (pin) {
      case 10:
        regOffset = 0;
        break;
      case 32:
        regOffset = 1;           
        break;    
      case 54:
        regOffset = 2;          
        break;    
      case 76:    
        regOffset = 3;          
        break;
      default:
        success = false;
        break;
    }
    if (success) {
      this->intensVal[regOffset] = (value << 4) | (value & 0x0F); //duplicate four LSB value
      max7315((MAX7315_INTENSITY_P1P0 + regOffset), this->intensVal[regOffset]);
      return true;
    } else {
      return false;
    }
  }
}

// Single register read and write
uint8_t MAX7315::max7315(uint8_t reg) {
	Wire.beginTransmission(this->addr);
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom(this->addr, uint8_t(1));
	return Wire.read();
}

void MAX7315::max7315(uint8_t reg, uint8_t value) {
	Wire.beginTransmission(this->addr);
	Wire.write(reg);
	Wire.write(value);
	Wire.endTransmission();
}

#if defined (ARDUINO_ESP32_DEV) //M35
  void MAX7315::initPins() {
    setPinMode(MAX_PWR_OFF, OUTPUT);
    setPinMode(MAX_KBD_BL, OUTPUT);
    setPinMode(MAX_TFT_BL, OUTPUT);
    setPinMode(MAX_PRPH_RST, OUTPUT);
    setPinMode(MAX_INT_BAT, INPUT);
    setPinMode(MAX_INT_IMU, INPUT);
    setPinMode(MAX_EXTINT, OUTPUT);
    setPinMode(MAX_COM_MUX, OUTPUT);
    //setPinMode(MAX_INT_O8, OUTPUT);
    log_d("MAX7315 pins set...");
    setLevel(MAX_PWR_OFF, HIGH);    // HIGH on Boot, LOW to Turn off the device
    setLevel(MAX_KBD_BL, LOW);      // HIGH on Boot
    setLevel(MAX_TFT_BL, LOW);      // HIGH on PowerOn to wake up without LCD,
    setLevel(MAX_PRPH_RST, HIGH);
    setLevel(MAX_COM_MUX, LOW);
    setLevel(MAX_EXTINT, LOW);      // LOW on Boot to disable GPS, HIGH to enable GPS
    setIntensity(MAX_KBD_BL, LOW);
    setIntensity(MAX_TFT_BL, LOW);
  }

#elif defined (ARDUINO_D1_MINI32)     //K36

  void MAX7315::initPins() {
    setPinMode(MAX_PWR_OFF, OUTPUT);
    setPinMode(MAX_EXTINT, OUTPUT);
    setPinMode(MAX_TFT_BL, OUTPUT);
    setPinMode(MAX_TFT_EN, OUTPUT);
    setPinMode(MAX_INT_BAT, INPUT);
    setPinMode(MAX_INT_IMU, INPUT);
    setPinMode(MAX_KBD_BL, OUTPUT);
    setPinMode(MAX_PRPH_RST, OUTPUT);
    setPinMode(MAX_PRPH_RST, OUTPUT);
    //setPinMode(MAX_INT_O8, OUTPUT);
    log_d("MAX7315 pins set...");
    setLevel(MAX_PWR_OFF, HIGH);    // HIGH on Boot, LOW to Turn off the device
    setLevel(MAX_EXTINT, LOW);      // LOW on Boot to disable GPS, HIGH to enable GPS
    setLevel(MAX_TFT_BL, LOW);      // HIGH on PowerOn to wake up without LCD,
    setLevel(MAX_TFT_EN, LOW);      // LOW on Boot to enable, HIGH in Timer mode
    setLevel(MAX_KBD_BL, LOW);    // HIGH on Boot
    setIntensity(MAX_TFT_BL, LOW); 
    setIntensity(MAX_KBD_BL, LOW);
  }

#endif

void MAX7315::setLcdBrightness(uint8_t lcdBrightness, uint8_t phase0) {
  setLevel(MAX_TFT_BL, phase0);
  setIntensity(MAX_TFT_BL, lcdBrightness);
  //;;Serial.print("LcdBright "); Serial.print(brightnessTFT, HEX); Serial.print("  LcdPhase "); Serial.println(phase0, DEC);
}

void MAX7315::setBtnBrightness(uint8_t btnBrightness, uint8_t phase0) {
  setLevel(MAX_KBD_BL, phase0);
  setIntensity(MAX_KBD_BL, btnBrightness);
  //;;Serial.print("KbdBright "); Serial.print(brightnessKbd, HEX); Serial.print("  KbdPhase "); Serial.println(phase0, DEC);
}

void MAX7315::powerOff(void) {
  setPinMode(MAX_PWR_OFF, OUTPUT);
  setLevel(MAX_PWR_OFF, LOW);
}

void MAX7315::disableLcdBl(void) {
  setLevel(MAX_TFT_BL, HIGH);
  setIntensity(MAX_TFT_BL, 0xF); 
}

void MAX7315::enableLcdBl(void) {
  setLevel(MAX_TFT_BL, LOW);
  setIntensity(MAX_TFT_BL, intensVal[1]);
}

void MAX7315::disableBtnBl(void) {
  setLevel(MAX_KBD_BL, HIGH);
  setIntensity(MAX_KBD_BL, 0xF);
}

void MAX7315::enableBtnBl(void) {
  setLevel(MAX_KBD_BL, LOW);
  setIntensity(MAX_KBD_BL, intensVal[3]);
}

void MAX7315::disableGnss(void) {
  setLevel(MAX_EXTINT, LOW);
}

void MAX7315::enableGnss(void) {
  setLevel(MAX_EXTINT, HIGH);
}