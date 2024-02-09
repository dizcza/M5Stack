#pragma once

#include <Arduino.h>

#define VBAT_TEST_ON  HIGH
#define VBAT_TEST_OFF LOW
#define VBAT_MCP_MAX      817 //0x032E
#define VBAT_MCP_FULL     822 //0x0333
#define VBAT_MCP_MIN      980 //0x03D4
#define VBAT_ADC_MAX      3124
#define VBAT_ADC_FULL     2960
#define VBAT_ADC_MIN      1924

const uint8_t adrMCP3021 = 0b01001000; //base address for mcp3021

class K46Bat
{
  public:
    K46Bat();
    void begin(uint8_t deviceId = 0x05);
    uint16_t readADC();
    bool isCharging();
    bool isChargeFull();
    int8_t getBatteryLevel();
    
  private:
      uint8_t _deviceId;  // device id.  From actual IC.  ex: mcp3021A5 = 0b00000101
      int _deviceAddress; // calculated device address using deviceID and constant mcp3021Adress
      uint16_t _vBatOld = 0;
};
