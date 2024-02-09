
#include "K46Bat.h"
#include "M5StX.h"
#include <driver/adc.h>

#if defined (PIN_VBAT_TEST) && defined (CHAN_VBAT_ADC)

K46Bat::K46Bat() {
}

void K46Bat::begin(uint8_t deviceId) {
  _deviceId = deviceId;
  _deviceAddress = (adrMCP3021 | _deviceId);
  Wire.beginTransmission(_deviceAddress);
  if(!Wire.endTransmission()) return;
  _deviceAddress = 0;
  //;;Serial.println("MCP3021 not found");
  pinMode(PIN_VBAT_TEST, OUTPUT);
  digitalWrite(PIN_VBAT_TEST, VBAT_TEST_OFF);
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(CHAN_VBAT_ADC, ADC_ATTEN_DB_11);
}

uint16_t K46Bat::readADC(){
  uint16_t vBat = 0x0000;
  if(_deviceAddress != 0){
    uint8_t data[2];
    data[0] = 0b00000000;
    data[1] = 0b00000000;
    Wire.requestFrom(_deviceAddress, 2);
    for(int i = 0; Wire.available() > 0; i++) {
      data[i] = Wire.read();
    }
    //;;Serial.print(data[0], HEX); Serial.print(" "); Serial.print(data[1], HEX);
    vBat = (vBat | (data[0] & 0x0F)) << 6;
    data[1] = data[1] >> 2;
    vBat = vBat | data[1];
  } else {                        // no MCP3021
    digitalWrite(PIN_VBAT_TEST, VBAT_TEST_ON);
    delay(10);
    vBat  = adc1_get_raw(CHAN_VBAT_ADC);
    vBat += adc1_get_raw(CHAN_VBAT_ADC);
    vBat /= 2;
    digitalWrite(PIN_VBAT_TEST, VBAT_TEST_OFF);
  }
  //;;Serial.print(" vBat "); Serial.print(vBat, HEX); Serial.print(" DEC "); Serial.println(vBat, DEC);
  return vBat;
}

bool K46Bat::isCharging() {
  uint16_t vBat = readADC();
  bool isCharge = false;
  if(_deviceAddress != 0){
    if(vBat < _vBatOld) isCharge = true;
  } else {
    if(vBat > _vBatOld) isCharge = true;
  }
  _vBatOld = vBat;
  return isCharge;
}

bool K46Bat::isChargeFull() {
  uint16_t vBat = readADC();
  if(_deviceAddress != 0){
    return (vBat <= VBAT_MCP_FULL ? true : false);
  } else {
    return (vBat >= VBAT_ADC_FULL ? true : false);
  }
}

// Return percentage * 100
int8_t K46Bat::getBatteryLevel() {
  uint16_t vBat = readADC();
  //Serial.print("vBat "); Serial.print(vBat, DEC);
  int8_t batLevel = 0;
  if(_deviceAddress != 0){
    batLevel = ((VBAT_MCP_MIN - vBat) * 100) / (VBAT_MCP_MIN - VBAT_MCP_MAX);
  } else {
    batLevel = ((vBat - VBAT_ADC_MIN) * 100) / (VBAT_ADC_MAX - VBAT_ADC_MIN);
  }
  //Serial.print(" batLevel "); Serial.println(batLevel, DEC);
  return batLevel;
}

#endif  /* PIN_VBAT_TEST */
