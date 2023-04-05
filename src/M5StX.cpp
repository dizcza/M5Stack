// Copyright (c) UT2UH. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "M5StX.h"


M5StX::M5StX() : isInited(0) { }

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
  #if defined (ARDUINO_M5Stack_Core_ESP32)
    Wire.begin(21, 22, 400000);
    Power.begin();
  #elif defined (ARDUINO_M5STACK_FIRE)
    Wire.begin(21, 22, 400000);
    Power.begin();
  #elif defined (ARDUINO_FROG_ESP32)  //K46
    Wire.begin(21, 22, (uint32_t) 400000);
    Bat.begin();
  #elif defined (ARDUINO_WESP32)      // K46v2
    Wire.begin(22, 21, 400000);
    Bat.begin();
  #elif defined (ARDUINO_LOLIN_D32_PRO) //TTGO T4 v1.3
    Wire.begin(21, 22, 400000);
    Power.begin();
    pinMode(SPEAKER_EN_PIN, OUTPUT);
    digitalWrite(SPEAKER_EN_PIN, LOW);
  #elif defined (ARDUINO_M5STACK_Core2)
    Wire1.begin(21, 22, 400000);
    Axp.begin((mbus_mode_t)externalPower);
    Axp.SetLDOEnable(3,0);  // turn any vibration off
    Axp.SetSpkEnable(false);
    Rtc.begin();
    Touch.begin();  // Touch begin after AXP begin. (Reset at the start of AXP)
    Wire.begin(32, 33, 400000);
  #elif defined (ARDUINO_TWatch)
    Wire.begin(21, 22, 400000);
    Wire1.begin(23, 32, 400000);
    // Axp.begin((mbus_mode_t)externalPower);
    // Axp.SetLDOEnable(3,0);  // turn any vibration off
    // Axp.SetSpkEnable(false);
    Rtc.begin();
    Touch.begin();  // Touch begin after AXP begin. (Reset at the start of AXP)
  #elif defined (ARDUINO_M5Stick_C) /* || defined (ARDUINO_M5Stick_C_Plus) */
    Wire1.begin(21, 22, 400000);
    Axp.begin();
    Rtc.begin();
    pinMode(SPEAKER_EN_PIN, OUTPUT);
    digitalWrite(SPEAKER_EN_PIN, LOW);
    //Set Wire on HY2.0-4P ?
  #elif defined (ARDUINO_ESP32_DEV) //M35
    Wire.begin(21, 22, 400000);
    //Touch.begin();
    Axp.begin();
    //Bat.begin();
    Ioe.begin();
    Ioe.initPins();
    Dac.begin();
  #elif defined (ARDUINO_D1_MINI32)  //K36
    Wire.begin(21, 22, 400000);
    Ioe.begin();
    Ioe.initPins();
  #endif

  // LCD INIT
  if (LCDEnable == true) {
    Lcd.begin();
  }

  #if defined (ARDUINO_M5Stack_Core_ESP32)
    // TF Card
    // if (SDEnable == true) {
    //   SD.begin(TFCARD_CS_PIN, SPI, 40000000);
    // }
  #elif defined (ARDUINO_M5STACK_Core2) || defined (ARDUINO_M5STACK_FIRE)
    // TF Card
    if (SDEnable == true) {
      SD.begin(TFCARD_CS_PIN, SPI, 40000000);
    }
  #elif defined (ARDUINO_ESP32_DEV)  //M35
    // if (SDEnable == true) {
    //   SD.begin(TFCARD_CS_PIN, SPI, 40000000);
    // }
  #endif

  #if defined (SRS_SDP32) || defined (ARDUINO_ESP32_DEV)
    Sos.begin(/* 170 */);
    Sht.begin();
    // #if defined (SRS_SDP32) && defined (ARDUINO_M5STACK_Core2)
    //   Gnss.begin(Wire1);
    // #else
    //   Gnss.begin(Wire);
    // #endif
    Tof.begin(0x50);
    pinMode(GNSS_IRQ_PIN, INPUT_PULLUP);
    pinMode(RTC_IRQ_PIN, INPUT);
    pinMode(IOE_IRQ_PIN, INPUT);
  #endif
}

void M5StX::update() {
  #if defined (ARDUINO_M5STACK_Core2) || defined (ARDUINO_TWatch) /* || defined (ARDUINO_ESP32_DEV) */
    Touch.update();
    Buttons.update();
  #else
    M5.BtnA.read();
	  M5.BtnB.read();
    M5.BtnC.read();
    //Buttons.update(); //Doesnot work with 3-button text entry!
  #endif
  yield();
}

M5StX M5;
