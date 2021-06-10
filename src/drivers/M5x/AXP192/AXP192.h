#ifndef __AXP192_H__
#define __AXP192_H__

#include <Wire.h>
#include <Arduino.h>

#define SLEEP_MSEC(us) (((uint64_t)us) * 1000L)
#define SLEEP_SEC(us)  (((uint64_t)us) * 1000000L)
#define SLEEP_MIN(us)  (((uint64_t)us) * 60L * 1000000L)
#define SLEEP_HR(us)   (((uint64_t)us) * 60L * 60L * 1000000L)

#define AXP_ADDR 0X34

typedef enum {
    kMBusModeOutput = 0,  // powered by USB or Battery  
    kMBusModeInput = 1  // powered by outside input
} mbus_mode_t;

class AXP192 {
public:
    enum CHGCurrent {
        kCHG_100mA = 0,
        kCHG_190mA,
        kCHG_280mA,
        kCHG_360mA,
        kCHG_450mA,
        kCHG_550mA,
        kCHG_630mA,
        kCHG_700mA,
        kCHG_780mA,
        kCHG_880mA,
        kCHG_960mA,
        kCHG_1000mA,
        kCHG_1080mA,
        kCHG_1160mA,
        kCHG_1240mA,
        kCHG_1320mA,
    };

  AXP192();
    static AXP192* instance;
    
    #if defined (ARDUINO_M5STACK_Core2)
      void  begin(mbus_mode_t mode = kMBusModeOutput);
    #elif defined (ARDUINO_M5Stick_C) /* || defined (ARDUINO_M5Stick_C_Plus) */
      /**
       * LDO2: Display backlight
       * LDO3: Display Control
       * RTC: Don't set GPIO1 as LDO
       * DCDC1: Main rail. When not set the controller shuts down.
       * DCDC3: Use unknown
       */
      void  begin(bool disableLcdBl = false, bool disablePeriph = false, bool disableRTC = false, bool disableVibr = true);
    #endif

    void ScreenBreath(uint8_t brightness);
    void SetAdcState(bool State);
    void SetChargeCurrent(uint8_t);
    void SetDCVoltage(uint8_t number, uint16_t voltage);
    void SetESPVoltage(uint16_t voltage);
    void SetLDOVoltage(uint8_t number, uint16_t voltage);
    void SetLDOEnable(uint8_t number, bool state);
    //Device specific settings
    #if defined (ARDUINO_M5STACK_Core2)
      void SetLCDRSet(bool state);
      void SetBusPowerMode(mbus_mode_t mode);
      void SetLcdVoltage(uint16_t voltage);
      void SetLed(uint8_t state);
      void SetSpkEnable(uint8_t state);
    #elif defined (ARDUINO_M5Stick_C) || defined (ARDUINO_M5Stick_C_Plus)
      void SetLDO2(bool State); // Can turn LCD Backlight OFF for power saving
      void SetLDO3(bool State); // Lcd controller
    #endif
    // Sleep 
    void SetSleep(void);
    void DeepSleep(uint64_t time_in_us = 0);
    void LightSleep(uint64_t time_in_us = 0);
    void PowerOff();
    // Power Maintained Storage
    void Read6BytesStorage(uint8_t *bufPtr);
    void Write6BytesStorage(uint8_t *bufPtr);

    uint8_t GetBtnPress(void);
    bool GetBatState();
    uint8_t GetInputPowerStatus();      //M5StickC/+
    uint8_t GetBatteryChargingStatus(); //M5StickC/+
    // Coulomb counters
    void  EnableCoulombcounter(void);
    void  DisableCoulombcounter(void);
    void  StopCoulombcounter(void);
    void  ClearCoulombcounter(void);
    uint32_t GetCoulombchargeData(void);	  // Raw Data for Charge
    uint32_t GetCoulombdischargeData(void);	// Raw Data for Discharge
    float GetCoulombData(void);				      // total in - total out and calc

    uint16_t GetVbatData(void) __attribute__((deprecated));
    uint16_t GetIchargeData(void) __attribute__((deprecated));
    uint16_t GetIdischargeData(void) __attribute__((deprecated));
    uint16_t GetTempData(void) __attribute__((deprecated));
    uint32_t GetPowerbatData(void) __attribute__((deprecated));
    uint16_t GetVinData(void) __attribute__((deprecated));
    uint16_t GetIinData(void) __attribute__((deprecated));
    uint16_t GetVusbinData(void) __attribute__((deprecated));
    uint16_t GetIusbinData(void) __attribute__((deprecated));
    uint16_t GetVapsData(void) __attribute__((deprecated));
    void SetCoulombClear()  __attribute__((deprecated)); // use ClearCoulombcounter instead

    float GetBatVoltage();
    float GetBatCurrent();
    float GetVinVoltage();
    float GetVinCurrent();
    float GetVBusVoltage();
    float GetVBusCurrent();
    float GetTempInAXP192();
    float GetBatPower();
    float GetBatChargeCurrent();
    float GetAPSVoltage();
    float GetBatCoulombInput();
    float GetBatCoulombOut();
    uint8_t GetWarningLevel(void);

      uint8_t AXPInState();
      bool isACIN();
      bool isCharging();
      bool isVBUS();

  private:
    void Write1Byte(uint8_t Addr,  uint8_t Data);
    uint8_t Read8bit(uint8_t Addr);
    uint16_t Read12Bit(uint8_t Addr);
    uint16_t Read13Bit(uint8_t Addr);
    uint16_t Read16bit(uint8_t Addr);
    uint32_t Read24bit(uint8_t Addr);
    uint32_t Read32bit(uint8_t Addr);
    void ReadBuff( uint8_t Addr , uint8_t Size , uint8_t *Buff );
};

#endif
