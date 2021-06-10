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
 * </pre>
 *
 */


#ifndef _M5STX_H_
  #define _M5STX_H_
  // create marker to ease core (legacy or M5StX) detection
  // e.g.:
  // #ifdef _M5STX_
  //    M5.ScreenShot.snap();
  // #endif
  #define _M5STX_

  #if defined(ESP32)

    #include "gitTagVersion.h"
    #include <Arduino.h>
    #include <Wire.h>
    #include <SPI.h>

    #include "utility/Config.h"
    #include "M5Display.h"
    #include "drivers/M5x/M5Sound/M5Sound.h"

    #if defined (ARDUINO_M5Stack_Core_ESP32) || defined (ARDUINO_M5STACK_FIRE)
      #include "drivers/M5x/Button/Button.h"
      #include "drivers/M5x/IP5306/Power.h"
      #if defined MPU9250_INSDE                   // Grey, Fire
        #include "drivers/M5x/MPU9250/MPU9250.h"
      #endif
      #if defined M5STACK_NODE
        #include "drivers/M5x/WM8978/WM8978.h"
      #endif
      #include "SD.h"
      #include "FS.h"
    #elif defined (ARDUINO_M5STACK_Core2)
      #include "drivers/M5x/M5Touch/M5Touch.h"
      #include "drivers/M5x/M5Button/M5Button.h"	// M5Buttons, M5Events, Button, Gesture
      #include "drivers/M5x/AXP192/AXP192.h"
      #include "drivers/M5x/BM8563/BM8563.h"
      #include "drivers/M5x/MPU6886/MPU6886.h"
      #include "SD.h"
      #include "FS.h"
    #elif defined (ARDUINO_M5Stick_C) /*|| defined (ARDUINO_M5Stick_C_Plus) */
      #include "drivers/M5x/Button/Button.h"
      #include "drivers/M5x/AXP192/AXP192.h"
      #include "drivers/M5x/BM8563/BM8563.h"
      #include "drivers/M5x/MPU6886/MPU6886.h"
    #elif defined (ARDUINO_LOLIN_D32_PRO) //TTGO T4 v1.3
      #include "drivers/M5x/Button/Button.h"
      #include "drivers/M5x/IP5306/Power.h"
      #include "SD.h"
      #include "FS.h"
    #elif defined (ARDUINO_ESP32_DEV)     //M35
      #include "drivers/M5x/Button/Button.h"
      #include "drivers/M35/LTC2943/LTC2943.h"
      #include "drivers/M35/MAX7315/MAX7315.h"
      #include "drivers/M5x/WM8978/WM8978.h"
      #include "SD.h"
      #include "FS.h"
    #elif defined (ARDUINO_D1_MINI32)     //K36
      #include "drivers/M5x/Button/Button.h"
      #include "drivers/K36/LTC2942/LTC2942.h"
      #include "drivers/K36/MAX7315/MAX7315.h"
    #endif

    #if defined (ARDUINO_ESP32_DEV) || defined (SRS_SDP32)
      #include "drivers/SRS/BME280/BME280.h"
      #include "drivers/SRS/RCWL9600/RCWL9600.h"
      #include "drivers/SRS/SHT3x/SHT3x.h"
      #include "drivers/SRS/VL53L0X/VL53L0X.h"
      // #include <SDPSensors.h>
      // #include <SparkFun_u-blox_GNSS_Arduino_Library.h>
    #endif


    class M5StX
    {
      public:
        M5StX();
        void begin(bool SDEnable = SD_ENABLE, bool SerialEnable = true, bool LCDEnable = true, bool externalPower = false); //mbus_mode_t mode = kMBusModeOutput
        void update();

        #if defined (ARDUINO_M5STACK_Core2)

          M5Touch Touch;

          // Buttons (global button and gesture functions)
          M5Buttons Buttons;

          // Default "button" that gets events where there is no button.
          Button background = Button(0, 0, TOUCH_W, TOUCH_H, true, "background");

          #define DEBOUNCE_MS 1
          // Touch version of the buttons on older M5stack cores, below screen
          Button BtnA = Button(10,240,110,40, true ,"BtnA");
          Button BtnB = Button(130,240,70,40, true, "BtnB");
          Button BtnC = Button(230,240,80,40, true, "BtnC");

        #elif defined (ARDUINO_ESP32_DEV) //M35 both with HW buttons and touchscreen

          #define DEBOUNCE_MS 10
          HWButton BtnA = HWButton(BUTTON_A_PIN, true, DEBOUNCE_MS);
          HWButton BtnB = HWButton(BUTTON_B_PIN, true, DEBOUNCE_MS);
          HWButton BtnC = HWButton(BUTTON_C_PIN, true, DEBOUNCE_MS);

        #elif defined  (ARDUINO_M5Stack_Core_ESP32) || defined (ARDUINO_LOLIN_D32_PRO) //TTGO T4 v1.3 // M5Stack_Core_ESP32, TTGO T4 v1.3, M5StickC/+

          #define DEBOUNCE_MS 10
          HWButton BtnA = HWButton(BUTTON_A_PIN, true, DEBOUNCE_MS);
          HWButton BtnB = HWButton(BUTTON_B_PIN, true, DEBOUNCE_MS);
          HWButton BtnC = HWButton(BUTTON_C_PIN, true, DEBOUNCE_MS);

        #else // M5StickC/+

          #define DEBOUNCE_MS 10
          HWButton BtnA = HWButton(BUTTON_A_PIN, true, DEBOUNCE_MS);
          HWButton BtnB = HWButton(BUTTON_B_PIN, true, DEBOUNCE_MS);
          HWButton BtnC = HWButton(BUTTON_C_PIN, true, DEBOUNCE_MS);

        #endif

        #if defined (ARDUINO_M5Stack_Core_ESP32) || defined (ARDUINO_M5STACK_FIRE) || defined (ARDUINO_LOLIN_D32_PRO) //TTGO T4 v1.3
          POWER Power;
          void powerOFF() { Power.powerOFF(); } 
          void setPowerBoostKeepOn(bool en)  { Power.setPowerBoostKeepOn(en); }
          void setWakeupButton(uint8_t button) { Power.setWakeupButton(button); }
          #ifdef MPU9250_INSDE
            MPU9250 IMU = MPU9250();
          #endif
        #elif defined (ARDUINO_M5STACK_Core2)
          AXP192 Axp = AXP192();  //!Power
          BM8563 Rtc;  //!RTC!
          MPU6886 IMU = MPU6886(0x69);  //Your Core2 use 0x68! I moved it to 0x69 to use DS3231SN
          void powerOFF() { Axp.PowerOff(); }
          void powerOff() { Axp.PowerOff(); }
        #elif defined (ARDUINO_M5Stick_C) /* || defined (ARDUINO_M5Stick_C_Plus) */
          AXP192 Axp = AXP192();  //!Power
          BM8563 Rtc;  //!RTC!
          MPU6886 IMU = MPU6886();
          void powerOFF() { Axp.PowerOff(); }
          void powerOff() { Axp.PowerOff(); }
        #elif defined (ARDUINO_ESP32_DEV)     //M35
          LTC2943 Bat;
          MAX7315 Ioe = MAX7315(/*MAX7315_I2C_ADDRESS*/);
          WM8978 Dac;
          void powerOFF() { Ioe.powerOff(); } //For M5ez-demo
        #elif defined (ARDUINO_D1_MINI32)     //K36
          LTC2942 Bat;
          MAX7315 Ioe = MAX7315(/*MAX7315_I2C_ADDRESS*/);
          MPU6886 Imu = MPU6886(0x69);
          void powerOFF() { Ioe.powerOff(); }
        #endif

        M5Display Lcd = M5Display();

        #if defined (SRS_SDP32)
          RCWL9600 Sos;
          // SDP3X Mic = SDP3X(Address5, MassFlow, Wire);
          SHT3x Sht  = SHT3x(adrSht0x44);
          // SFE_UBLOX_GNSS Gnss;
          VL53L0X Tof;
        #elif defined (ARDUINO_ESP32_DEV)
          RCWL9600 Sos;
          // SDP8XX Mic = SDP8XX(Address5, MassFlow, Wire);
          SHT3x Sht  = SHT3x(adrSht0x44);
          // SFE_UBLOX_GNSS Gnss;
          VL53L0X Tof;
        #endif

      private:
          bool isInited;
    };
    
    extern M5StX M5;
    #define m5 M5
    #define M35 M5
    #define lcd Lcd
  #else
    #error "This library only supports boards with ESP32 processor."
  #endif
#endif
