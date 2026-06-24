// Copyright (c) UT2UH. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "M5StX.h"
#include "SD.h"


M5StX::M5StX() : isInited(0)
 {
#if defined(ARDUINO_TTGO_T1)
   MCPMan.addButton(&BtnA);
   MCPMan.addButton(&BtnB);
   MCPMan.addButton(&BtnC);
#endif
 }

void M5StX::begin(bool SDEnable, bool SerialEnable, bool LCDEnable, bool externalPower) {
  if (isInited == true) {
    log_d("M5StX already inited");
    return;
  } else {
    isInited = true;
  }
  if (SerialEnable == true) {
    Serial.begin(115200);
    Serial.flush();
    delay(50);
  }
  #if defined (ARDUINO_TTGO_T1)
    Wire.begin(21, 22, (uint32_t) 400000);
    Wire1.begin(32, 33, (uint32_t) 400000);
    MCPMan.begin();
  #endif

  // LCD INIT
  if (LCDEnable == true) {
    Lcd.begin();
    Lcd.setGPIOExpander(&MCPMan);
  }

  if (SDEnable == true) {
    SD.begin(TFCARD_CS_PIN, SPI, 40000000);
  }

  log_d("M5 begin completed");
}

void M5StX::update() {
  MCPMan.update();
  M5.BtnA.read();
  M5.BtnB.read();
  M5.BtnC.read();
  yield();
}

M5StX M5;
