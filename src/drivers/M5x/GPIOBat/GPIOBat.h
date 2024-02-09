#pragma once

#include <Arduino.h>

#define VBAT_TEST_ON HIGH
#define VBAT_TEST_OFF LOW
#define VBAT_ADC_MAX 4100
#define VBAT_ADC_FULL 4100
#define VBAT_ADC_MIN 1900

class GPIOBat
{
public:
    GPIOBat(uint8_t pin);
    void begin();
    uint16_t readADC();
    bool isCharging();
    bool isChargeFull();
    int8_t getBatteryLevel();

private:
    uint8_t pin;  // GPIO battery pin
    uint16_t _vBatOld = 0;
};
