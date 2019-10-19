// Copyright (c) M5Stack. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "L0Stack.h"

L0Stack::L0Stack() : isInited(0) {
}

void L0Stack::begin(bool TFTEnable, bool SDEnable, bool SerialEnable, bool I2CEnable) {
  // Correct init once
  if (isInited == true) {
    return;
  } else {
    isInited = true;
  }

  // UART
  if (SerialEnable == true) {
    Serial.begin(115200);
    Serial.flush();
    delay(50);
    Serial.print("L0Stack initializing...");
  }

  // TFT INIT
  if (TFTEnable == true) {
    TFT.begin();
  }

  // SD Card
  if (SDEnable == true) {
    SD.begin(TFCARD_CS_PIN);
  }

}

void L0Stack::update() {
  //Button update
  BtnA.read();
  BtnB.read();
  BtnC.read();

}

L0Stack L0;
