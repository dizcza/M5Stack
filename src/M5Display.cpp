#include "M5Display.h"

#define BLK_PWM_CHANNEL 7 // LEDC_CHANNEL_7

#if defined (ARDUINO_M5Stick_C) || defined (ARDUINO_M5STACK_Core2) || defined (ARDUINO_TWatch) || defined (ARDUINO_ESP32_DEV) //M35
  #include "drivers/M5x/AXP192/AXP192.h"
#endif
#if defined (ARDUINO_ESP32_DEV) || defined (ARDUINO_D1_MINI32)	//M35, K36
  #include "drivers/M35/MAX7315/MAX7315.h"
#endif

#ifdef CST_INT
  #include "drivers/M5x/M5Touch/M5Touch.h"
#endif

// So we can use this instance without including all of M5Core2 / M5Stack
M5Display* M5Display::instance;

M5Display::M5Display() : TFT_eSPI() {
  if (!instance) instance = this;
}

void M5Display::begin() {
  TFT_eSPI::begin();
  setRotation(DEFAULT_ROTATION);
  fillScreen(0);
  #if defined (ARDUINO_M5Stack_Core_ESP32) || defined (ARDUINO_M5STACK_FIRE) || defined (ARDUINO_LOLIN_D32_PRO) || defined (ARDUINO_FROG_ESP32) || defined (ARDUINO_WESP32) || defined (ARDUINO_TTGO_T1) //TTGO T4 v1.3 || K46 || K46v2
    // Init the back-light LED PWM
    ledcSetup(BLK_PWM_CHANNEL, 44100, 8);
    ledcAttachPin(TFT_BL, BLK_PWM_CHANNEL);
  #endif
  setBrightness(200);
}

void M5Display::sleep() {
  startWrite();
  writecommand(TFT_SLPIN); // Software reset
  endWrite();
}

void M5Display::wakeup() {
  startWrite();
  writecommand(TFT_SLPOUT);
  endWrite();
}

// M5Stack compatible range 0-255
void M5Display::setBrightness(uint8_t brightness) {
  #if defined (ARDUINO_M5Stack_Core_ESP32) || defined (ARDUINO_M5STACK_FIRE) || defined (ARDUINO_LOLIN_D32_PRO) || defined (ARDUINO_FROG_ESP32) || defined (ARDUINO_WESP32) || defined (ARDUINO_TTGO_T1)  //TTGO T4 v1.3 || K46 || K46v2
    ledcWrite(BLK_PWM_CHANNEL, brightness); // brightness 0-255
  #elif defined (ARDUINO_M5STACK_Core2)
    //AXP->SetLcdVoltage(brightness * 80 + 2500); //For M5ez
    AXP->SetLcdVoltage(brightness * 3 + 2530);  // brightness 0-255
  #elif defined (ARDUINO_M5Stick_C)
    //AXP->ScreenBreath(brightness * 10 / 13 + 7);  // brightness in % for 7 to 15 LDO Vvalue
    AXP->ScreenBreath((brightness >> 5) + 7);  // brightness 0-255
  #elif defined (ARDUINO_ESP32_DEV) //M35
    //AXP->SetLcdVoltage(brightness * 80 + 2500); //For M5ez
    AXP->SetLcdVoltage(brightness * 3 + 2530);  // brightness 0-255
  #elif defined (ARDUINO_D1_MINI32)  //M35 or K36
    IOE->setLcdBrightness(brightness, LOW);
  #endif
}

void M5Display::drawBitmap(int16_t x0, int16_t y0, int16_t w, int16_t h, const uint16_t *data) {
  bool swap = getSwapBytes();
  setSwapBytes(true);
  pushImage((int32_t)x0, (int32_t)y0, (uint32_t)w, (uint32_t)h, data);
  setSwapBytes(swap);
}

void M5Display::drawBitmap(int16_t x0, int16_t y0, int16_t w, int16_t h, uint16_t *data) {
  bool swap = getSwapBytes();
  setSwapBytes(true);
  pushImage((int32_t)x0, (int32_t)y0, (uint32_t)w, (uint32_t)h, data);
  setSwapBytes(swap);
}

void M5Display::drawBitmap(int16_t x0, int16_t y0, int16_t w, int16_t h, const uint16_t *data, uint16_t transparent) {
  bool swap = getSwapBytes();
  setSwapBytes(true);
  pushImage((int32_t)x0, (int32_t)y0, (uint32_t)w, (uint32_t)h, data, transparent);
  setSwapBytes(swap);
}

void M5Display::drawBitmap(int16_t x0, int16_t y0, int16_t w, int16_t h, const uint8_t *data) {
  bool swap = getSwapBytes();
  setSwapBytes(true);
  pushImage((int32_t)x0, (int32_t)y0, (uint32_t)w, (uint32_t)h, (const uint16_t*)data);
  setSwapBytes(swap);
}

void M5Display::drawBitmap(int16_t x0, int16_t y0, int16_t w, int16_t h, uint8_t *data) {
  bool swap = getSwapBytes();
  setSwapBytes(true);
  pushImage((int32_t)x0, (int32_t)y0, (uint32_t)w, (uint32_t)h, (uint16_t*)data);
  setSwapBytes(swap);
}

void M5Display::progressBar(int x, int y, int w, int h, uint8_t val) {
  drawRect(x, y, w, h, 0x09F1);
  fillRect(x + 1, y + 1, w * (((float)val) / 100.0), h - 1, 0x09F1);
}

#include "utility/qrcode.h"
void M5Display::qrcode(const char *string, uint16_t x, uint16_t y, uint8_t width, uint8_t version) {

  // Create the QR code
  QRCode qrcode;
  uint8_t qrcodeData[qrcode_getBufferSize(version)];
  qrcode_initText(&qrcode, qrcodeData, version, 0, string);

  // Top quiet zone
  uint8_t thickness = width / qrcode.size;
  uint16_t lineLength = qrcode.size * thickness;
  uint8_t xOffset = x + (width-lineLength)/2;
  uint8_t yOffset = y + (width-lineLength)/2;
  fillRect(x, y, width, width, TFT_WHITE);

  for (uint8_t y = 0; y < qrcode.size; y++) {
    for (uint8_t x = 0; x < qrcode.size; x++) {
      uint8_t q = qrcode_getModule(&qrcode, x, y);
      if (q) fillRect(x * thickness + xOffset, y * thickness + yOffset, thickness, thickness, TFT_BLACK);
    }
  }
}

void M5Display::qrcode(const String &string, uint16_t x, uint16_t y, uint8_t width, uint8_t version) {
  int16_t len = string.length() + 2;
  char buffer[len];
  string.toCharArray(buffer, len);
  qrcode(buffer, x, y, width, version);
}

// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.

uint16_t read16(fs::File &f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

uint32_t read32(fs::File &f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}


/***************************************************
  This library is written to be compatible with Adafruit's ILI9341
  library and automatically detects the display type on ESP_WROVER_KITs
  Earlier WROVERs had ILI9341, while newer releases have ST7789V

  MIT license, all text above must be included in any redistribution
 ****************************************************/



// Saves and restores font properties, datum, cursor, colors

void M5Display::pushState() {
  DisplayState s;
  s.textfont = textfont;
  s.textsize = textsize;
  s.textcolor = textcolor;
  s.textbgcolor = textbgcolor;
  s.cursor_x = cursor_x;
  s.cursor_y = cursor_y;
  s.padX = padX;
  s.gfxFont = gfxFont;
  _displayStateStack.push_back(s);
}

void M5Display::popState() {
  if (_displayStateStack.empty()) return;
  DisplayState s = _displayStateStack.back();
  _displayStateStack.pop_back();
  textfont = s.textfont;
  textsize = s.textsize;
  textcolor = s.textcolor;
  textbgcolor = s.textbgcolor;
  cursor_x = s.cursor_x;
  cursor_y = s.cursor_y;
  padX = s.padX;
  if (s.gfxFont && s.gfxFont != gfxFont) setFreeFont(s.gfxFont);
}

#ifdef CST_INT

  #ifdef TFT_eSPI_TOUCH_EMULATION

    // Emulates the native (resistive) TFT_eSPI touch interface using M5.Touch

    uint8_t M5Display::getTouchRaw(uint16_t *x, uint16_t *y) {
      return getTouch(x, y);
    }

    uint16_t M5Display::getTouchRawZ(void) {
      return (TOUCH->ispressed()) ? 1000 : 0;
    }

    void M5Display::convertRawXY(uint16_t *x, uint16_t *y) { return; }

    uint8_t M5Display::getTouch(uint16_t *x, uint16_t *y,
                                uint16_t threshold /* = 600 */) {
      TOUCH->read();
      if (TOUCH->points) {
        *x = TOUCH->point[0].x;
        *y = TOUCH->point[0].y;
        return true;
      }
      return false;
    }

    void M5Display::calibrateTouch(uint16_t *data, uint32_t color_fg,
                                  uint32_t color_bg, uint8_t size) {
      return;
    }

    void M5Display::setTouch(uint16_t *data) { return; }

  #endif /* TFT_eSPI_TOUCH_EMULATION */

#endif /* CST_INT */
