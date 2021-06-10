
/* MAX7315 library 
 * Based on PCA9534 library by Abdulrahman Saleh Khamis
 *
 * This library allows for easy interfacing with MAX7315 GPIO expander chip
 * It has familiar functions when dealing with MAX7315 pins such as 
 * pinMode(), digitalWrite(), and digitalRead().
 *
 * It also allows using interrupt when input status changes,
 * to avoid continuous polling.
 */

#ifndef MAX7315_H
  #define MAX7315_H

  #include "Arduino.h"
  #include <Wire.h>
  #include "utility/Config.h"

  #define MAX7315_I2C_ADDRESS     0x20

  #define MAX7315_READ_INPUT      0x00
  #define MAX7315_BLINK_PHASE0    0x01
  #define MAX7315_PORTS_CONFIG    0x03
  #define MAX7315_BLINK_PHASE1    0x09
  #define MAX7315_MASTER_O8GLOB   0x0E
  #define MAX7315_CONFIGURATION   0x0F
  #define MAX7315_INTENSITY_P1P0  0x10
  #define MAX7315_INTENSITY_P3P2  0x11
  #define MAX7315_INTENSITY_P5P4  0x12
  #define MAX7315_INTENSITY_P7P6  0x13
  #define MAX7315_CONF_MASK_BLEN  0x01
  #define MAX7315_CONF_MASK_BLFL  0x02
  #define MAX7315_CONF_MASK_GLINT 0x04
  #define MAX7315_CONF_MASK_ENINT 0x08

#if defined (ARDUINO_D1_MINI32)
  #define MAX_PWR_OFF        0
  #define MAX_EXTINT         1
  #define MAX_TFT_BL         2
  #define MAX_TFT_EN         3
  #define MAX_LOAD_TEST      4
  #define MAX_PWR_MODE       5
  #define MAX_KBD_BL_B       6
  #define MAX_KBD_BL_T       7
  #define MAX_KBD_BL        76
  #define MAX_VSENSE_EN      8 
#else
  #define MAX_PWR_OFF        0
  #define MAX_KBD_BL         1
  #define MAX_TFT_BL         2
  #define MAX_PRPH_RST       3
  #define MAX_INT_BAT        4
  #define MAX_INT_IMU        5
  #define MAX_EXTINT         6
  #define MAX_COM_MUX        7
#endif

class MAX7315 {

  public:
    
    uint8_t intensVal[4] = {0xFF, 0xFF, 0xFF, 0xFF}; //Ports PWM values

    /**
     * Create an instance of the device with the set address
     *
     * @param {uint8_t} i2caddr - Sets the slave address of the MAX7315,
     * defaults to 0x20.
     */
    MAX7315(/* uint8_t addr = MAX7315_I2C_ADDRESS */);
    static MAX7315* instance;

    /**
     * Initializes the device
     * This method should be called before any others are used.
     *
     */
    void begin(bool mixedMode = true, bool blinkEnable = false, bool blinkPhase = false);
    void initPins(); 
    void setLcdBrightness(uint8_t lcdBrightness, uint8_t phase0);
    void setBtnBrightness(uint8_t btnBrightness, uint8_t phase0);
    void powerOff(void);
    void disableLcdBl(void);
    void enableLcdBl(void);
    void disableBtnBl(void);
    void enableBtnBl(void);
    void disableGnss(void);
    void enableGnss(void);

  private:

    /**
     * Configures the specified pin to behave either as an input, inverted input,
     * or output.
     *
     * @param {uint8_t} pin - Pin number whose mode you wish to set.
     * @param {uint8_t} mode - Pin mode one of: INPUT, or OUTPUT.
     */
    void setPinMode(uint8_t pin, uint8_t mode);

    /**
     * Reads the value from a specified digital pin, either HIGH or LOW.
     *
     * @param {uint8_t} pin - Pin number whose value you wish to get.
     * @returns {uint8_t} The status of the pin either HIGH or LOW.
     */
    uint8_t getLevel(uint8_t pin);
    
    /**
     * Writes a HIGH or a LOW value to a digital pin.
     *
     * @param {uint8_t} pin - Pin number whose value you wish to set.
     * @param {bool} value - Pin value one of: HIGH, or LOW.
     */
    void setLevel(uint8_t pin, bool value);

    void setPhase1(uint8_t pin, bool value);
    
    /** 
     * Set PWM intensity of a digital pin.
     *
     * @param {uint8_t} pin   - Valid pin numbers - 0-8, 10, 32, 54, 76
     * @param {uint8_t} value - Pin intensity value.
     * 
     */
    bool setIntensity(uint8_t pin, uint8_t value);  

    void setGlobal(bool enableGlobal);
    void setBlink(bool blinkEnable);
    void flipBlink(bool phaseRegister);
    void enableInterrupt(bool enableInt);
    void setGlobalIntensity(uint8_t intensity);  // 0x0 - 0xF
    void setMasterIntensity(uint8_t intensity);  // 0x0 - 0xF
    // TODO!
    //uint8_t getIntPin();

    uint8_t addr;
    uint8_t phase0;
    uint8_t port;
    uint8_t phase1;
    uint8_t masterO8;
    uint8_t config;

    uint8_t globalIntensity;
    uint8_t masterIntensity;
    bool    mixedMode;
    bool    enableInt;
    bool    enableGlobal;
    bool    blinkPhase;
    bool    blinkEnable;
    
    uint8_t max7315(uint8_t reg);
    void    max7315(uint8_t reg, uint8_t value);
    void    max7315(uint8_t reg, uint8_t size, uint8_t *data);
};

#endif
