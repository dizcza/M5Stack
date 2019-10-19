// Copyright (c) M5Stack. All rights reserved.

// Licensed under the MIT license. See LICENSE file in the project root for full license information.
/**
 * \par Copyright (C), 2016-2017, M5Stack
 * \class L0Stack
 * \brief   L0Stack library.
 * @file    L0Stack.h
 * @author  L0Stack
 * @version V0.0.1
 * @date    2019/10/19
 * @brief   Header for L0Stack.cpp module
 *
 * \par Description
 * This file is a drive for L0Stack core.
 *
 * \par Method List:
 *
 *  System:
        L0.begin();
        L0.update();

    LCD:
        L0.TFT.setBrightness(uint8_t brightness);
        L0.TFT.drawPixel(int16_t x, int16_t y, uint16_t color);
        L0.TFT.drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
        L0.TFT.fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
        L0.TFT.fillScreen(uint16_t color);
        L0.TFT.drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
        L0.TFT.drawCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername,uint16_t color);
        L0.TFT.fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
        L0.TFT.fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername,int16_t delta, uint16_t color);
        L0.TFT.drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
        L0.TFT.fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
        L0.TFT.drawRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
        L0.TFT.fillRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
        L0.TFT.drawBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color);
        L0.TFT.drawRGBBitmap(int16_t x, int16_t y, const uint16_t bitmap[], int16_t w, int16_t h),
        L0.TFT.drawChar(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg, uint8_t size);
        L0.TFT.setCursor(uint16_t x0, uint16_t y0);
        L0.TFT.setTextColor(uint16_t color);
        L0.TFT.setTextColor(uint16_t color, uint16_t backgroundcolor);
        L0.TFT.setTextSize(uint8_t size);
        L0.TFT.setTextWrap(boolean w);
        L0.TFT.printf();
        L0.TFT.print();
        L0.TFT.println();
        L0.TFT.drawCentreString(const char *string, int dX, int poY, int font);
        L0.TFT.drawRightString(const char *string, int dX, int poY, int font);
        L0.TFT.drawJpg(const uint8_t *jpg_data, size_t jpg_len, uint16_t x, uint16_t y);
        L0.TFT.drawJpgFile(fs::FS &fs, const char *path, uint16_t x, uint16_t y);
        L0.TFT.drawBmpFile(fs::FS &fs, const char *path, uint16_t x, uint16_t y);

    Button:
        L0.BtnA/B/C.read();
        L0.BtnA/B/C.isPressed();
        L0.BtnA/B/C.isReleased();
        L0.BtnA/B/C.wasPressed();
        L0.BtnA/B/C.wasReleased();
        L0.BtnA/B/C.wasreleasedFor()
        L0.BtnA/B/C.pressedFor(uint32_t ms);
        L0.BtnA/B/C.releasedFor(uint32_t ms);
        L0.BtnA/B/C.lastChange();

 *
 * \par History:
 * <pre>
 * `<Author>`         `<Time>`        `<Version>`        `<Descr>`
 * Zibin Zheng         2017/07/14        0.0.1          Rebuild the new.
 * Bin                 2018/10/29        0.2.4          Add Button API
 * UT2UH               2019/10/19        0.0.1          STM32L0 fork
 * </pre>
 *
 */

#ifndef _L0STACK_H_
  #define _L0STACK_H_
  
  #if defined(ARDUINO_ARCH_STM32L0)

    #include "gitTagVersion.h"
    #include <Arduino.h>
    #include <Wire.h>
    #include <SPI.h>
    #include "FS.h"
    #include "SD.h"

    #include "L0Display.h"
    #include "utility/Config.h"
    #include "utility/Button.h"

    class L0Stack
    {
      public:
        L0Stack();
        void begin(bool TFTEnable = true, bool SDEnable = true, bool SerialEnable = true, bool I2CEnable = false);
        void update();

        // Button API
        #define DEBOUNCE_MS 10
        Button BtnA = Button(BUTTON_A_PIN, true, DEBOUNCE_MS);
        Button BtnB = Button(BUTTON_B_PIN, true, DEBOUNCE_MS);
        Button BtnC = Button(BUTTON_C_PIN, true, DEBOUNCE_MS);

        // LCD
        M5Display TFT = L0Display();
        
      private:
          bool isInited;
    };
    
    extern L0Stack L0;
    #define M5 L0
    #define m5 L0
    #define l0 L0    
    #define Lcd TFT
    #define lcd TFT
    #define tft TFT        
  #else
    #error “This library only supports boards with STM32L0 processor.”
  #endif
#endif
