// Copyright (c) UT2UH. All rights reserved.

// Licensed under the MIT license. See LICENSE file in the project root for full license information.
/**
 * \par Copyright (C), 2021, UT2UH
 * \class M5Stack
 * \brief   M5Stx library.
 * @file    M5Stx.h
 * @author  M5Stack, UT2UH
 * @version V0.3.5
 * @date    2021/05/29
 * @brief   Header for M5StX.cpp module
 *
 * \par Description
 * This file is a drive for M5Stack devices and derivativses.
 *
 * \par Method List:
 *
 *  System:
        M5.begin();
        M5.update();

    Power:
        M5.Power.setPowerBoostKeepOn()
        M5.Power.setCharge(uint8_t mode);
        M5.Power.setPowerBoostKeepOn(bool en);
        M5.Power.isChargeFull();
        M5.Power.setWakeupButton(uint8_t button);
        M5.Power.powerOFF();

        bool setPowerBoostOnOff(bool en);
        bool setPowerBoostSet(bool en);
        bool setPowerVin(bool en);
        bool setPowerWLEDSet(bool en);

    LCD:
        M5.lcd.setBrightness(uint8_t brightness);
        M5.Lcd.drawPixel(int16_t x, int16_t y, uint16_t color);
        M5.Lcd.drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
        M5.Lcd.fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
        M5.Lcd.fillScreen(uint16_t color);
        M5.Lcd.drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
        M5.Lcd.drawCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername,uint16_t color);
        M5.Lcd.fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
        M5.Lcd.fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername,int16_t delta, uint16_t color);
        M5.Lcd.drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
        M5.Lcd.fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
        M5.Lcd.drawRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
        M5.Lcd.fillRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
        M5.Lcd.drawBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color);
        M5.Lcd.drawRGBBitmap(int16_t x, int16_t y, const uint16_t bitmap[], int16_t w, int16_t h),
        M5.Lcd.drawChar(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg, uint8_t size);
        M5.Lcd.setCursor(uint16_t x0, uint16_t y0);
        M5.Lcd.setTextColor(uint16_t color);
        M5.Lcd.setTextColor(uint16_t color, uint16_t backgroundcolor);
        M5.Lcd.setTextSize(uint8_t size);
        M5.Lcd.setTextWrap(boolean w);
        M5.Lcd.printf();
        M5.Lcd.print();
        M5.Lcd.println();
        M5.Lcd.drawCentreString(const char *string, int dX, int poY, int font);
        M5.Lcd.drawRightString(const char *string, int dX, int poY, int font);
        M5.Lcd.drawJpg(const uint8_t *jpg_data, size_t jpg_len, uint16_t x, uint16_t y);
        M5.Lcd.drawJpgFile(fs::FS &fs, const char *path, uint16_t x, uint16_t y);
        M5.Lcd.drawBmpFile(fs::FS &fs, const char *path, uint16_t x, uint16_t y);

    Button:
        M5.BtnA/B/C.read();
        M5.BtnA/B/C.isPressed();
        M5.BtnA/B/C.isReleased();
        M5.BtnA/B/C.wasPressed();
        M5.BtnA/B/C.wasReleased();
        M5.BtnA/B/C.wasreleasedFor()
        M5.BtnA/B/C.pressedFor(uint32_t ms);
        M5.BtnA/B/C.releasedFor(uint32_t ms);
        M5.BtnA/B/C.lastChange();

    Speaker:
        M5.Speaker.tone(uint32_t freq);
        M5.Speaker.tone(freq, time);
        M5.Speaker.beep();
        M5.Speaker.setBeep(uint16_t frequency, uint16_t duration);
        M5.Speaker.mute();

 *
 * \par History:
 * <pre>
 * `<Author>`         `<Time>`        `<Version>`        `<Descr>`
 * UT2UH               2021/05/20        0.0.1          M35 rework created
 * UT2UH               2021/10/20        0.0.2          K46 support added
 * UT2UH               2022/01/05        0.0.3          TWatch support added
 * UT2UH               2023/02/06        0.0.4          K46v2 support added
 * </pre>
 *
 */


#ifndef _M5STX_H_
  #define _M5STX_H_

    #include <Arduino.h>
    #include <Wire.h>
    #include <SPI.h>

    #include "utility/Config.h"
    #include "M5Display.h"

    #if defined (ARDUINO_TTGO_T1)
      #include "drivers/M5x/Button/Button.h"
      #include "drivers/M5x/GPIOBat/GPIOBat.h"
    #endif

    class M5StX
    {
      public:
        M5StX();
        void begin(bool SDEnable = SD_ENABLE, bool SerialEnable = true, bool LCDEnable = true, bool externalPower = false); //mbus_mode_t mode = kMBusModeOutput
        void update();

        #if defined (ARDUINO_TTGO_T1)

          #define DEBOUNCE_MS 10
          MCPXManager MCPMan = MCPXManager(MCP23017_I2C_ADDRESS, Wire, DEBOUNCE_MS);
          MCPBtn BtnA = MCPBtn(MCP_EXPANDER_BTN_A_PIN, true);
          MCPBtn BtnB = MCPBtn(MCP_EXPANDER_BTN_B_PIN, true);
          MCPBtn BtnC = MCPBtn(MCP_EXPANDER_BTN_C_PIN, true);

          GPIOBat Bat = GPIOBat(35);
        #endif

        M5Display Lcd = M5Display();

      private:
          bool isInited;
    };
    
    extern M5StX M5;
#endif
