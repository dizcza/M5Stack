#pragma once

#include <Arduino.h>

#define GPIOBat_VBAT_ADC_MAX    4170
#define GPIOBat_VBAT_ADC_FULL   4100
#define GPIOBat_VBAT_ADC_MIN    3650

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
