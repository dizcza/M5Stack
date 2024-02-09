
#include "GPIOBat.h"
#include <driver/adc.h>


GPIOBat::GPIOBat(uint8_t pin) : pin(pin) {
}

void GPIOBat::begin() {
}

uint16_t GPIOBat::readADC(){
    uint16_t vBat = 2 * analogReadMilliVolts(pin);
    return vBat;
}

bool GPIOBat::isCharging()
{
    uint16_t vBat = readADC();
    bool isCharge = false;
    if (vBat > _vBatOld)
        isCharge = true;
    _vBatOld = vBat;
    return isCharge;
}

bool GPIOBat::isChargeFull()
{
    uint16_t vBat = readADC();
    return vBat >= VBAT_ADC_FULL;
}

// Return percentage * 100
int8_t GPIOBat::getBatteryLevel() {
  uint16_t vBat = readADC();
  float battLevelFloat = ((vBat - VBAT_ADC_MIN) * 100.0f) / (float) (VBAT_ADC_MAX - VBAT_ADC_MIN);
  int8_t batLevel = std::min((int8_t) 100, std::max((int8_t) 0, (int8_t) battLevelFloat));
  return batLevel;
}
