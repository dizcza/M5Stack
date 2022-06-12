#include "AXP192.h"

/* static */ AXP192* AXP192::instance;

AXP192::AXP192() {
    if (!instance) instance = this;
}

#if defined (ARDUINO_M5STACK_Core2)

    void AXP192::begin(mbus_mode_t mode /* = kMBusModeOutput */) {
        log_w("[ AXP ]");
        //AXP192 30H
        Write1Byte(0x30, (Read8bit(0x30) & 0x04) | 0x02);
        log_d("  - VBUS limit off");
        //AXP192 GPIO1:OD OUTPUT
        Write1Byte(0x92, Read8bit(0x92) & 0xF8);
        log_d("  - GPIO1 init as output");
        //AXP192 GPIO2:OD OUTPUT
        Write1Byte(0x93, Read8bit(0x93) & 0xF8);
        log_d("  - GPIO2 init as output");
        //AXP192 RTC CHG
        Write1Byte(0x35, (Read8bit(0x35) & 0x1C) | 0xA3);
        log_w("  - RTC battery charging enabled");
        SetESPVoltage(3350);
        log_w("  - ESP32 power voltage was set to 3.35v");
        SetLcdVoltage(2800);
        log_w("  - TFT backlight voltage was set to 2.80v");
        SetLDOVoltage(2, 3300); //Periph power voltage preset (LCD_logic, SD card)
        log_w("  - TFT logic and SDCard voltage preset to 3.3v");
        SetLDOVoltage(3, 2000); //Vibrator power voltage preset
        log_w("  - Vibrator voltage preset to 2v");
        SetLDOEnable(2, true);  //LCD_logic, SD card
        //SetDCDC3(true); // LCD backlight
        SetLed(false);
        Write1Byte(0x33, 0xC0); //Bat charge voltage to 4.2, Current 100MA
        //SetCHGCurrent(kCHG_100mA);
        log_w("  - M5Go CHG Base current set to 100mA");
        //SetAxpPriphPower(1);
        //Serial.printf("axp: lcd_logic and sdcard power enabled\n\n");
        pinMode(37, INPUT_PULLUP);  //To unify SetSleep
        //AXP192 GPIO4
        Write1Byte(0x95, (Read8bit(0x95) & 0x72) | 0x84);
        log_d("  - GPIO4 init");
        // 128ms power on, 4s power off
        Write1Byte(0x36, 0x4C);
        // Set ADC sample rate to 200hz
        Write1Byte(0x84, 0b11110010);
        // Set ADC to All Enable
        Write1Byte(0x82, 0xFF);
        SetLCDRSet(0);
        delay(100);
        SetLCDRSet(1);
        delay(100);
        // bus power mode kMBusModeOutput
        SetBusPowerMode(mode);
    }

#elif defined (ARDUINO_TWatch)

    void AXP192::begin(mbus_mode_t mode /* = kMBusModeOutput */) {
        log_w("[ AXP ]");
        //AXP192 30H
        Write1Byte(0x30, (Read8bit(0x30) & 0x04) | 0x02);
        log_d("  - VBUS limit off");
        //AXP192 GPIO1:OD OUTPUT
        Write1Byte(0x92, Read8bit(0x92) & 0xF8);
        log_d("  - GPIO1 init as output");
        //AXP192 GPIO2:OD OUTPUT
        Write1Byte(0x93, Read8bit(0x93) & 0xF8);
        log_d("  - GPIO2 init as output");
        //AXP192 RTC CHG
        Write1Byte(0x35, (Read8bit(0x35) & 0x1C) | 0xA3);
        log_w("  - RTC battery charging enabled");
        SetESPVoltage(3350);
        log_w("  - ESP32 power voltage was set to 3.35v");
        SetLcdVoltage(2800);
        log_w("  - TFT backlight voltage was set to 2.80v");
        SetLDOVoltage(2, 3300); //Periph power voltage preset (LCD_logic, SD card)
        log_w("  - TFT logic and SDCard voltage preset to 3.3v");
        SetLDOVoltage(3, 2000); //Vibrator power voltage preset
        log_w("  - Vibrator voltage preset to 2v");
        SetLDOEnable(2, true);  //LCD_logic, SD card
        //SetDCDC3(true); // LCD backlight
        SetLed(false);
        Write1Byte(0x33, 0xC0); //Bat charge voltage to 4.2, Current 100MA
        //SetCHGCurrent(kCHG_100mA);
        log_w("  - M5Go CHG Base current set to 100mA");
        //SetAxpPriphPower(1);
        //Serial.printf("axp: lcd_logic and sdcard power enabled\n\n");
        pinMode(37, INPUT_PULLUP);  //To unify SetSleep
        //AXP192 GPIO4
        Write1Byte(0x95, (Read8bit(0x95) & 0x72) | 0x84);
        log_d("  - GPIO4 init");
        // 128ms power on, 4s power off
        Write1Byte(0x36, 0x4C);
        // Set ADC sample rate to 200hz
        Write1Byte(0x84, 0b11110010);
        // Set ADC to All Enable
        Write1Byte(0x82, 0xFF);
        SetLCDRSet(0);
        delay(100);
        SetLCDRSet(1);
        delay(100);
        // bus power mode kMBusModeOutput
        SetBusPowerMode(mode);
    }

#elif defined (ARDUINO_M5Stick_C) || defined (ARDUINO_M5Stick_C_Plus)

    void AXP192::begin(bool disableLcdBl, bool disablePeriph, bool disableRTC, bool disableVibr) {  
        // Set LDO2 & LDO3(TFT_LED & TFT) 3.0V
        Write1Byte(0x28, 0xCC);
        // Set ADC sample rate to 200hz
        //Write1Byte(0x84, 0b11110010);
        // Bat charge voltage to 4.2, Current 100MA
        Write1Byte(0x33, 0xC0);
        // Set ADC to All Enable
        Write1Byte(0x82, 0xFF);

        // Depending on configuration enable LDO2, LDO3, DCDC1, DCDC3.
        // byte buf = (Read8bit(0x12) & 0xEF) | 0x4D;
        // if(disableVibr)   buf &= ~(1<<3);
        // if(disablePeriph) buf &= ~(1<<2);
        // if(disableLcdBl)  buf &= ~(1<<1);
        // Write1Byte(0x12, buf);

        // Enable Ext, LDO2, LDO3, DCDC1
	    Write1Byte(0x12, Read8bit(0x12) | 0x4D);	

        // 128ms power on, 4s power off
        Write1Byte(0x36, 0x0C);
        if(!disableRTC) {
            // Set RTC voltage to 3.3V
            Write1Byte(0x91, 0xF0);
            // Set GPIO0 to LDO
            Write1Byte(0x90, 0x02);
        }
        // Disable vbus hold limit
        Write1Byte(0x30, 0x80);
        // Set temperature protection
        Write1Byte(0x39, 0xFC);
        // Enable RTC BAT charge 
        Write1Byte(0x35, 0xA2 & (disableRTC ? 0x7F : 0xFF));
        // Enable bat detection
        Write1Byte(0x32, 0x46);
        // Set Power off voltage 3.0v
        //Write1Byte(0x31 , (Read8bit(0x31) & 0xF8) | (1 << 2));
        ScreenBreath(11);
    }

#elif defined (ARDUINO_ESP32_DEV)     //M35
    void AXP192::begin(mbus_mode_t mode /* = kMBusModeOutput */,
                        bool disableRTC /* = false */,
                        bool disableLcdBl /* = false */,
                        bool disablePeriph /* = false */) {
        log_w("[ AXP ]");
        //AXP192 30H
        Write1Byte(0x30, (Read8bit(0x30) & 0x04) | 0x02);
        log_d("  - VBUS limit off");
        //AXP192 GPIO1:OD OUTPUT
        Write1Byte(0x92, Read8bit(0x92) & 0xF8);
        log_d("  - GPIO1 init as output");
        //AXP192 GPIO2:OD OUTPUT
        Write1Byte(0x93, Read8bit(0x93) & 0xF8);
        log_d("  - GPIO2 init as output");
        //AXP192 RTC CHG
        Write1Byte(0x35, (Read8bit(0x35) & 0x1C) | 0xA3);
        log_w("  - RTC battery charging enabled");
        SetESPVoltage(3350);
        log_w("  - ESP32 power voltage was set to 3.35v");
        SetLcdVoltage(2800);
        log_w("  - TFT backlight voltage was set to 2.80v");
        SetLDOVoltage(2, 3300); //Periph power voltage preset (LCD_logic, SD card)
        log_w("  - TFT logic and SDCard voltage preset to 3.3v");
        SetLDOVoltage(3, 1800); //ICM-20948 voltage preset
        log_w("  - IMU voltage preset to 1v8");
        SetLDOEnable(2, true);  //LCD_logic, SD card
        //SetDCDC3(true); // LCD backlight
        SetLed(false);
        Write1Byte(0x33, 0xC0); //Bat charge voltage to 4.2, Current 100MA
        //SetCHGCurrent(kCHG_100mA);
        log_w("  - M5Go CHG Base current set to 100mA");
        //SetAxpPriphPower(1);
        //Serial.printf("axp: lcd_logic and sdcard power enabled\n\n");
        pinMode(37, INPUT_PULLUP);  //To unify SetSleep
        //AXP192 GPIO4
        Write1Byte(0x95, (Read8bit(0x95) & 0x72) | 0x84);
        log_d("  - GPIO4 init");
        // 128ms power on, 4s power off
        Write1Byte(0x36, 0x4C);
        // Set ADC sample rate to 200hz
        Write1Byte(0x84, 0b11110010);
        // Set ADC to All Enable
        Write1Byte(0x82, 0xFF);
        // Set GPIO4 to 0
        SetPeriphReset(0);
        delay(100);
        // Set GPIO4 to 1
        SetPeriphReset(1);
        delay(100);
        // Set bus power mode kMBusModeOutput
        SetBusPowerMode(mode);
    }

#endif

void AXP192::Write1Byte(uint8_t Addr, uint8_t Data) {
    AXPWIRE.beginTransmission(AXP_ADDR);
    AXPWIRE.write(Addr);
    AXPWIRE.write(Data);
    AXPWIRE.endTransmission();
}

uint8_t AXP192::Read8bit(uint8_t Addr) {
    AXPWIRE.beginTransmission(AXP_ADDR);
    AXPWIRE.write(Addr);
    AXPWIRE.endTransmission();
    AXPWIRE.requestFrom(AXP_ADDR, 1);
    return AXPWIRE.read();
}

uint16_t AXP192::Read12Bit(uint8_t Addr) {
    uint16_t Data = 0;
    uint8_t buf[2];
    ReadBuff(Addr, 2, buf);
    Data = ((buf[0] << 4) + buf[1]);
    return Data;
}

uint16_t AXP192::Read13Bit(uint8_t Addr) {
    uint16_t Data = 0;
    uint8_t buf[2];
    ReadBuff(Addr, 2, buf);
    Data = ((buf[0] << 5) + buf[1]);
    return Data;
}

uint16_t AXP192::Read16bit(uint8_t Addr) {
    uint16_t ReData = 0;
    AXPWIRE.beginTransmission(AXP_ADDR);
    AXPWIRE.write(Addr);
    AXPWIRE.endTransmission();
    AXPWIRE.requestFrom(AXP_ADDR, 2);
    for (int i = 0; i < 2; i++) {
        ReData <<= 8;
        ReData |= AXPWIRE.read();
    }
    return ReData;
}

uint32_t AXP192::Read24bit(uint8_t Addr) {
    uint32_t ReData = 0;
    AXPWIRE.beginTransmission(AXP_ADDR);
    AXPWIRE.write(Addr);
    AXPWIRE.endTransmission();
    AXPWIRE.requestFrom(AXP_ADDR, 3);
    for (int i = 0; i < 3; i++) {
        ReData <<= 8;
        ReData |= AXPWIRE.read();
    }
    return ReData;
}

uint32_t AXP192::Read32bit(uint8_t Addr) {
    uint32_t ReData = 0;
    AXPWIRE.beginTransmission(AXP_ADDR);
    AXPWIRE.write(Addr);
    AXPWIRE.endTransmission();
    AXPWIRE.requestFrom(AXP_ADDR, 4);
    for (int i = 0; i < 4; i++) {
        ReData <<= 8;
        ReData |= AXPWIRE.read();
    }
    return ReData;
}

void AXP192::ReadBuff(uint8_t Addr, uint8_t Size, uint8_t *Buff) {
    AXPWIRE.beginTransmission(AXP_ADDR);
    AXPWIRE.write(Addr);
    AXPWIRE.endTransmission();
    AXPWIRE.requestFrom(AXP_ADDR, (int)Size);
    for (int i = 0; i < Size; i++) {
        *(Buff + i) = AXPWIRE.read();
    }
}

void AXP192::ScreenBreath(uint8_t brightness) {
    if (brightness > 15) brightness = 15;
    if (brightness < 7) brightness = 7;
    uint8_t buf = Read8bit(0x28);
    Write1Byte(0x28, ((buf & 0x0f) | (brightness << 4)));
}

// Return True = Battery Exist
bool AXP192::GetBatState() {
    if (Read8bit(0x01) | 0x20) return true;
    else return false;
}

// Input Power Status 
uint8_t AXP192::GetInputPowerStatus() {
    return Read8bit(0x00);
}

// Battery Charging Status 
uint8_t AXP192::GetBatteryChargingStatus() {
    return Read8bit(0x01);
}

//---------coulombcounter_from_here---------
//enable: void EnableCoulombcounter(void);
//disable: void DisableCOulombcounter(void);
//stop: void StopCoulombcounter(void);
//clear: void ClearCoulombcounter(void);
//get charge data: uint32_t GetCoulombchargeData(void);
//get discharge data: uint32_t GetCoulombdischargeData(void);
//get coulomb val affter calculation: float GetCoulombData(void);
//------------------------------------------
void AXP192::EnableCoulombcounter(void) { Write1Byte(0xB8, 0x80); }

void AXP192::DisableCoulombcounter(void) { Write1Byte(0xB8, 0x00); }

void AXP192::StopCoulombcounter(void) { Write1Byte(0xB8, 0xC0); }

void AXP192::ClearCoulombcounter(void) { Write1Byte(0xB8, Read8bit(0xB8) | 0x20); }   // Only set the Clear Flag
// #if defined (ARDUINO_M5STACK_Core2)
//     void AXP192::ClearCoulombcounter(void) { Write1Byte(0xB8, 0xA0); }
// #elif defined (ARDUINO_TWatch)
//     void AXP192::ClearCoulombcounter(void) { Write1Byte(0xB8, 0xA0); }
// #elif defined (ARDUINO_M5Stick_C) || defined (ARDUINO_M5Stick_C_Plus)
//     void AXP192::ClearCoulombcounter(void) { Write1Byte(0xB8, Read8bit(0xB8) | 0x20); }   // Only set the Clear Flag
// #endif

uint32_t AXP192::GetCoulombchargeData(void) { return Read32bit(0xB0); }

uint32_t AXP192::GetCoulombdischargeData(void) { return Read32bit(0xB4); }

float AXP192::GetCoulombData(void) {
    uint32_t coin = GetCoulombchargeData();
    uint32_t coout = GetCoulombdischargeData();
    uint32_t valueDifferent = 0;
    bool bIsNegative = false;
    if (coin > coout) {    // Expected, Cin always more then Cout
        valueDifferent = coin - coout;
    } else {    // Warning: Cout is more than Cin, the battery is not started at 0% 
        // just Flip the output sign later
        bIsNegative = true;
        valueDifferent = coout - coin;
    }
    //c = 65536 * current_LSB * (coin - coout) / 3600 / ADC rate
    //ADC rate can be read from 84H, change this variable if you change the ADC rate
    float ccc = (65536 * 0.5 * valueDifferent) / 3600.0 / 200.0;  // Note the ADC has defaulted to be 200 Hz
    if( bIsNegative ) ccc = 0.0 - ccc;    // Flip it back to negative
    return ccc;
}
//----------coulomb_end_at_here----------

uint16_t AXP192::GetVbatData(void) {
    uint16_t vbat = 0;
    uint8_t buf[2];
    ReadBuff(0x78, 2, buf);
    vbat = ((buf[0] << 4) + buf[1]); // V
    return vbat;
}

uint16_t AXP192::GetVinData(void) {
    uint16_t vin = 0;
    uint8_t buf[2];
    ReadBuff(0x56, 2, buf);
    vin = ((buf[0] << 4) + buf[1]); // V
    return vin;
}

uint16_t AXP192::GetIinData(void) {
    uint16_t iin = 0;
    uint8_t buf[2];
    ReadBuff(0x58, 2, buf);
    iin = ((buf[0] << 4) + buf[1]);
    return iin;
}

uint16_t AXP192::GetVusbinData(void) {
    uint16_t vin = 0;
    uint8_t buf[2];
    ReadBuff(0x5a, 2, buf);
    vin = ((buf[0] << 4) + buf[1]); // V
    return vin;
}

uint16_t AXP192::GetIusbinData(void) {
    uint16_t iin = 0;
    uint8_t buf[2];
    ReadBuff(0x5C, 2, buf);
    iin = ((buf[0] << 4) + buf[1]);
    return iin;
}

uint16_t AXP192::GetIchargeData(void) {
    uint16_t icharge = 0;
    uint8_t buf[2];
    ReadBuff(0x7A, 2, buf);
    icharge = (buf[0] << 5) + buf[1] ;
    return icharge;
}

uint16_t AXP192::GetIdischargeData(void) {
    uint16_t idischarge = 0;
    uint8_t buf[2];
    ReadBuff(0x7C, 2, buf);
    idischarge = (buf[0] << 5) + buf[1] ;
    return idischarge;
}

uint16_t AXP192::GetTempData(void) {
    uint16_t temp = 0;
    uint8_t buf[2];
    ReadBuff(0x5e, 2, buf);
    temp = ((buf[0] << 4) + buf[1]);
    return temp;
}

uint32_t AXP192::GetPowerbatData(void) {
    uint32_t power = 0;
    uint8_t buf[3];
    ReadBuff(0x70, 2, buf);
    power = (buf[0] << 16) + (buf[1] << 8) + buf[2];
    return power;
}

uint16_t AXP192::GetVapsData(void) {
    uint16_t vaps = 0;
    uint8_t buf[2];
    ReadBuff(0x7e, 2, buf);
    vaps = ((buf[0] << 4) + buf[1]);
    return vaps;
}

#if defined (ARDUINO_M5STACK_Core2)
    void AXP192::SetSleep(void) {
        Write1Byte(0x31 , Read8bit(0x31) | ( 1 << 3)); // Turn on short press to wake up
        Write1Byte(0x90 , Read8bit(0x90) & 0xF8); // GPIO0 - floating in M5StickC/+, OD - M5Core2
        Write1Byte(0x82, 0x00); // Disable ADCs
        Write1Byte(0x12, Read8bit(0x12) & 0xA1); // Disable all outputs but DCDC1
    }
#elif defined (ARDUINO_TWatch)
    void AXP192::SetSleep(void) {
        Write1Byte(0x31 , Read8bit(0x31) | ( 1 << 3)); // Turn on short press to wake up
        Write1Byte(0x90 , Read8bit(0x90) & 0xF8); // GPIO0 - floating in M5StickC/+, OD - M5Core2
        Write1Byte(0x82, 0x00); // Disable ADCs
        Write1Byte(0x12, Read8bit(0x12) & 0xA1); // Disable all outputs but DCDC1
    }
#elif defined (ARDUINO_M5Stick_C) /*|| defined (ARDUINO_M5Stick_C_Plus) */
    void AXP192::SetSleep(void) {
        Write1Byte(0x31 , Read8bit(0x31) | ( 1 << 3)); // Turn on short press to wake up
        Write1Byte(0x90 , Read8bit(0x90) | 0x07); // GPIO0 - floating in M5StickC/+, OD - M5Core2
        Write1Byte(0x82, 0x00); // Disable ADCs
        Write1Byte(0x12, Read8bit(0x12) & 0xA1); // Disable all outputs but DCDC1
    }
#elif defined (ARDUINO_ESP32_DEV)     //M35
    void AXP192::SetSleep(void) {
        Write1Byte(0x31 , Read8bit(0x31) | ( 1 << 3)); // Turn on short press to wake up
        Write1Byte(0x90 , Read8bit(0x90) & 0xF8); // GPIO0 - floating in M5StickC/+, OD - M5Core2
        Write1Byte(0x82, 0x00); // Disable ADCs
        Write1Byte(0x12, Read8bit(0x12) & 0xA1); // Disable all outputs but DCDC1
    }
#endif

// -- sleep
void AXP192::DeepSleep(uint64_t time_in_us) {
    SetSleep();
    //For M5StickC/Plus only, in M5Core2 GPIO37 is not connected
    esp_sleep_enable_ext0_wakeup((gpio_num_t)37, LOW);
    if (time_in_us > 0)     {
        esp_sleep_enable_timer_wakeup(time_in_us);
    } else {
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    }
    (time_in_us == 0) ? esp_deep_sleep_start() : esp_deep_sleep(time_in_us);
}

void AXP192::LightSleep(uint64_t time_in_us) {
    SetSleep(); //Not in M5Stick. Why?
    if (time_in_us > 0) {
        esp_sleep_enable_timer_wakeup(time_in_us);
    } else {
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    }
    esp_light_sleep_start();
}

// Return 0 = no press, 0x01 = long press(1.5s), 0x02 = short press
uint8_t AXP192::GetBtnPress() {
    uint8_t state = Read8bit(0x46);  // IRQ 3 status.  
    if(state) {
        Write1Byte( 0x46 , 0x03 );   // Write 1 back to clear IRQ
    }
    return state;
}

// Low Volt Level 1, when APS Volt Output < 3.4496 V
// Low Volt Level 2, when APS Volt Output < 3.3992 V, then this flag is SET (0x01)
// Flag will reset once battery volt is charged above Low Volt Level 1
// Note: now AXP192 have the Shutdown Voltage of 3.0V (B100) Def in REG 31H
uint8_t AXP192::GetWarningLevel(void) {
    return Read8bit(0x47) & 0x01;
}

float AXP192::GetBatVoltage() {
    float ADCLSB = 1.1 / 1000.0;
    uint16_t ReData = Read12Bit(0x78);
    return ReData * ADCLSB;
}

float AXP192::GetBatCurrent() {
    float ADCLSB = 0.5;
    uint16_t CurrentIn = Read13Bit(0x7A);
    uint16_t CurrentOut = Read13Bit(0x7C);
    return (CurrentIn - CurrentOut) * ADCLSB;
}

float AXP192::GetVinVoltage() {
    float ADCLSB = 1.7 / 1000.0;
    uint16_t ReData = Read12Bit(0x56);
    return ReData * ADCLSB;
}

float AXP192::GetVinCurrent() {
    float ADCLSB = 0.625;
    uint16_t ReData = Read12Bit(0x58);
    return ReData * ADCLSB;
}

float AXP192::GetVBusVoltage() {
    float ADCLSB = 1.7 / 1000.0;
    uint16_t ReData = Read12Bit(0x5A);
    return ReData * ADCLSB;
}

float AXP192::GetVBusCurrent() {
    float ADCLSB = 0.375;
    uint16_t ReData = Read12Bit(0x5C);
    return ReData * ADCLSB;
}

float AXP192::GetTempInAXP192() {
    float ADCLSB = 0.1;
    const float OFFSET_DEG_C = -144.7;
    uint16_t ReData = Read12Bit(0x5E);
    return OFFSET_DEG_C + ReData * ADCLSB;
}

float AXP192::GetBatPower() {
    float VoltageLSB = 1.1;
    float CurrentLCS = 0.5;
    uint32_t ReData = Read24bit(0x70);
    return VoltageLSB * CurrentLCS * ReData / 1000.0;
}

float AXP192::GetBatChargeCurrent() {
    float ADCLSB = 0.5;
    uint16_t ReData = Read13Bit(0x7A);
    return ReData * ADCLSB;
}

float AXP192::GetAPSVoltage() {
    float ADCLSB = 1.4 / 1000.0;
    uint16_t ReData = Read12Bit(0x7E);
    return ReData * ADCLSB;
}

float AXP192::GetBatCoulombInput() {
    uint32_t ReData = Read32bit(0xB0);
    return ReData * 65536 * 0.5 / 3600 / 25.0;
}

float AXP192::GetBatCoulombOut() {
    uint32_t ReData = Read32bit(0xB4);
    return ReData * 65536 * 0.5 / 3600 / 25.0;
}

void AXP192::SetCoulombClear() {
    Write1Byte(0xB8, 0x20);
}

uint8_t AXP192::AXPInState() {
    return Read8bit(0x00);
}

bool AXP192::isACIN() {
    return ( Read8bit(0x00) & 0x80 ) ? true : false;
}

bool AXP192::isCharging() {
    return ( Read8bit(0x00) & 0x02 ) ? true : false;
}

bool AXP192::isVBUS() {
    return ( Read8bit(0x00) & 0x20 ) ? true : false;
}

void AXP192::SetLDOVoltage(uint8_t number, uint16_t voltage) {
    voltage = (voltage > 3300) ? 15 : (voltage / 100) - 18;
    switch (number) {
        //uint8_t reg, data;
        case 2:
            Write1Byte(AXP_ADDR, (Read8bit(0x28) & 0x0F) | (voltage << 4));
            break;
        case 3:
            Write1Byte(AXP_ADDR, (Read8bit(0x28) & 0xF0) | voltage);
            break;
    }
}

void AXP192::SetDCVoltage(uint8_t number, uint16_t voltage) {
    uint8_t addr;
    if (number > 2) return;
    voltage = (voltage < 700) ? 0 : (voltage - 700) / 25;
    switch (number) {
        case 0:
            addr = 0x26;
            break;
        case 1:
            addr = 0x25;
            break;
        case 2:
            addr = 0x27;
            break;
    }
    Write1Byte(addr, (Read8bit(addr) & 0x80) | (voltage & 0x7F));
}

void AXP192::SetESPVoltage(uint16_t voltage) {
    if (voltage >= 3000 && voltage <= 3400) {
        SetDCVoltage(0, voltage);
    }
}


void AXP192::SetLDOEnable(uint8_t number, bool state) {
    uint8_t mark = 0x01;
    if ((number < 2) || (number > 3)) return;
    mark <<= number;
    if (state) Write1Byte(0x12, (Read8bit(0x12) | mark));
    else Write1Byte(0x12, (Read8bit(0x12) & (~mark)));
}

#if defined (ARDUINO_M5STACK_Core2)
    void AXP192::SetLCDRSet(bool state) {
        uint8_t reg_addr = 0x96;
        uint8_t gpio_bit = 0x02;
        uint8_t data;
        data = Read8bit(reg_addr);
        if (state) data |= gpio_bit;
        else data &= ~gpio_bit;
        Write1Byte(reg_addr, data);
    }

    // Select source for BUS_5V
    // kMBusModeOutput : powered by USB or Battery
    // kMBusModeInput  : powered by external input
    void AXP192::SetBusPowerMode(mbus_mode_t mode) {
        uint8_t data;
        if (mode == kMBusModeOutput) {
            // Set GPIO to 3.3V (LDO OUTPUT mode)
            data = Read8bit(0x91);
            Write1Byte(0x91, (data & 0x0F) | 0xF0);
            // Set GPIO0 to LDO OUTPUT, pullup N_VBUSEN to disable VBUS supply from BUS_5V
            data = Read8bit(0x90);
            Write1Byte(0x90, (data & 0xF8) | 0x02);
            // Set EXTEN to enable 5v boost
            data = Read8bit(0x10);
            Write1Byte(0x10, data | 0x04);
        } else {
            // Set EXTEN to disable 5v boost
            data = Read8bit(0x10);
            Write1Byte(0x10, data & ~0x04);
            // Set GPIO0 to float, using enternal pulldown resistor to enable VBUS supply from BUS_5V
            data = Read8bit(0x90);
            Write1Byte(0x90, (data & 0xF8) | 0x07);
        }
    }

    void AXP192::SetLcdVoltage(uint16_t voltage) {
        if (voltage >= 2500 && voltage <= 3300) {
            SetDCVoltage(2, voltage);
        }
    }

    void AXP192::SetLed(uint8_t state) {
        uint8_t reg_addr=0x94;
        uint8_t data;
        data=Read8bit(reg_addr);
        if(state) data &= 0xFD;
        else data |= 0x02;
        Write1Byte(reg_addr, data);
    }

    //set led state(GPIO high active,set 1 to enable amplifier)
    void AXP192::SetSpkEnable(uint8_t state) {
        uint8_t reg_addr=0x94;
        uint8_t gpio_bit=0x04;
        uint8_t data;
        data=Read8bit(reg_addr);
        if(state) data |= gpio_bit;
        else data &= ~gpio_bit;
        Write1Byte(reg_addr, data);
    }

#elif defined (ARDUINO_TWatch)
    void AXP192::SetLCDRSet(bool state) {
        uint8_t reg_addr = 0x96;
        uint8_t gpio_bit = 0x02;
        uint8_t data;
        data = Read8bit(reg_addr);
        if (state) data |= gpio_bit;
        else data &= ~gpio_bit;
        Write1Byte(reg_addr, data);
    }

    // Select source for BUS_5V
    // kMBusModeOutput : powered by USB or Battery
    // kMBusModeInput  : powered by external input
    void AXP192::SetBusPowerMode(mbus_mode_t mode) {
        uint8_t data;
        if (mode == kMBusModeOutput) {
            // Set GPIO to 3.3V (LDO OUTPUT mode)
            data = Read8bit(0x91);
            Write1Byte(0x91, (data & 0x0F) | 0xF0);
            // Set GPIO0 to LDO OUTPUT, pullup N_VBUSEN to disable VBUS supply from BUS_5V
            data = Read8bit(0x90);
            Write1Byte(0x90, (data & 0xF8) | 0x02);
            // Set EXTEN to enable 5v boost
            data = Read8bit(0x10);
            Write1Byte(0x10, data | 0x04);
        } else {
            // Set EXTEN to disable 5v boost
            data = Read8bit(0x10);
            Write1Byte(0x10, data & ~0x04);
            // Set GPIO0 to float, using enternal pulldown resistor to enable VBUS supply from BUS_5V
            data = Read8bit(0x90);
            Write1Byte(0x90, (data & 0xF8) | 0x07);
        }
    }

    void AXP192::SetLcdVoltage(uint16_t voltage) {
        if (voltage >= 2500 && voltage <= 3300) {
            SetDCVoltage(2, voltage);
        }
    }

    void AXP192::SetLed(uint8_t state) {
        uint8_t reg_addr=0x94;
        uint8_t data;
        data=Read8bit(reg_addr);
        if(state) data &= 0xFD;
        else data |= 0x02;
        Write1Byte(reg_addr, data);
    }

    //set led state(GPIO high active,set 1 to enable amplifier)
    void AXP192::SetSpkEnable(uint8_t state) {
        uint8_t reg_addr=0x94;
        uint8_t gpio_bit=0x04;
        uint8_t data;
        data=Read8bit(reg_addr);
        if(state) data |= gpio_bit;
        else data &= ~gpio_bit;
        Write1Byte(reg_addr, data);
    }

#elif defined (ARDUINO_M5Stick_C) || defined (ARDUINO_M5Stick_C_Plus)
    // Can turn LCD Backlight OFF for power saving
    void AXP192::SetLDO2(bool State) {
        uint8_t buf = Read8bit(0x12);
        if (State == true) buf = (1 << 2) | buf;
        else buf = ~(1 << 2) & buf;
        Write1Byte(0x12, buf);
    }
    
    // Can turn LCD controller power OFF
    void AXP192::SetLDO3(bool State) {
        uint8_t buf = Read8bit(0x12);
        if(State == true) buf = (1<<3) | buf;
        else buf = ~(1<<3) & buf;
        Write1Byte(0x12, buf);
    }

#elif defined (ARDUINO_ESP32_DEV)     //M35

    // Select source for BUS_5V
    // kMBusModeOutput : powered by USB or Battery
    // kMBusModeInput  : powered by external input
    void AXP192::SetBusPowerMode(mbus_mode_t mode) {
        uint8_t data;
        if (mode == kMBusModeOutput) {
            // Set GPIO to 3.3V (LDO OUTPUT mode)
            data = Read8bit(0x91);
            Write1Byte(0x91, (data & 0x0F) | 0xF0);
            // Set GPIO0 to LDO OUTPUT, pullup N_VBUSEN to disable VBUS supply from BUS_5V
            data = Read8bit(0x90);
            Write1Byte(0x90, (data & 0xF8) | 0x02);
            // Set EXTEN to enable 5v boost
            data = Read8bit(0x10);
            Write1Byte(0x10, data | 0x04);
        } else {
            // Set EXTEN to disable 5v boost
            data = Read8bit(0x10);
            Write1Byte(0x10, data & ~0x04);
            // Set GPIO0 to float, using enternal pulldown resistor to enable VBUS supply from BUS_5V
            data = Read8bit(0x90);
            Write1Byte(0x90, (data & 0xF8) | 0x07);
        }
    }

    // Set DCDC2 in 2V5 to 3V3 range
    void AXP192::SetLcdVoltage(uint16_t voltage) {
        if (voltage >= 2500 && voltage <= 3300) {
            SetDCVoltage(2, voltage);
        }
    }

    //Set LED on GPIO1 to state
    void AXP192::SetLed(uint8_t state) {
        uint8_t data;
        data=Read8bit(0x94);
        if(state) data &= 0xFD;
        else data |= 0x02;
        Write1Byte(0x94, data);
    }

    // Set SPK_EN (GPIO2) 1 to enable amplifier
    void AXP192::SetSpkEnable(uint8_t state) {
        uint8_t data;
        data=Read8bit(0x94);
        if(state) data |= 0x04;
        else data &= 0xFB; // ~0x04
        Write1Byte(0x94, data);
    }

    // Set GNSS EXTINT (GPIO3) state
    void AXP192::SetExtint(uint8_t state) {
        uint8_t data;
        data = Read8bit(0x96);
        if (state) data |= 0x01;
        else data &= 0xFE; //~0x01
        Write1Byte(0x96, data);
    }

    // Set PERIPH_RESET (GPIO4) state
    void AXP192::SetPeriphReset(bool state) {
        uint8_t data;
        data = Read8bit(0x96);
        if (state) data |= 0x02;
        else data &= 0xFD; //~0x02
        Write1Byte(0x96, data);
    }
#endif

// Not recommend to set M5StickC/Plus charge current > 100mA, since Battery is only 80/120mAh
// and 190mA for M5Core2
// more then 1C charge-rate may shorten battery life-span.
void AXP192::SetChargeCurrent(uint8_t current) {
    uint8_t buf = Read8bit(0x33);
    buf = (buf & 0xf0) | (current & 0x07);  //0x07 limits to 700mA
    Write1Byte(0x33, buf);
}

// Cut all power, except for LDO1 (RTC)
void AXP192::PowerOff() {
    Write1Byte(0x32, Read8bit(0x32) | 0x80);     // MSB for Power Off
}

void AXP192::SetAdcState(bool state) {
    Write1Byte(0x82, state ? 0xff : 0x00);  // Enable / Disable all ADCs
}

// AXP192 have a 6 byte storage, when the power is still valid, the data will not be lost
void AXP192::Read6BytesStorage( uint8_t *bufPtr ) {
    // Address from 0x06 - 0x0B
    AXPWIRE.beginTransmission(AXP_ADDR);
    AXPWIRE.write(0x06);
    AXPWIRE.endTransmission();
    AXPWIRE.requestFrom(AXP_ADDR, 6);
    for( int i = 0; i < 6; ++i ) {
        bufPtr[i] = AXPWIRE.read();
    }
}

// AXP192 have a 6 byte storage, when the power is still valid, the data will not be lost
void AXP192::Write6BytesStorage( uint8_t *bufPtr ) {
    // Address from 0x06 - 0x0B
    AXPWIRE.beginTransmission(AXP_ADDR);
    AXPWIRE.write(0x06);
    AXPWIRE.write(bufPtr[0]);
    AXPWIRE.write(0x07);
    AXPWIRE.write(bufPtr[1]);
    AXPWIRE.write(0x08);
    AXPWIRE.write(bufPtr[2]);
    AXPWIRE.write(0x09);
    AXPWIRE.write(bufPtr[3]);
    AXPWIRE.write(0x0A);
    AXPWIRE.write(bufPtr[4]);
    AXPWIRE.write(0x0B);
    AXPWIRE.write(bufPtr[5]);
    AXPWIRE.endTransmission();
}
