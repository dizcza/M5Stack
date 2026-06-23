#include "M5Display.h"

// So we can use this instance without including all of M5Core2 / M5Stack
M5Display* M5Display::instance;

M5Display::M5Display() : TFT_eSPI() {
  if (!instance) instance = this;
}

void M5Display::begin() {
  TFT_eSPI::begin();
  setRotation(DEFAULT_ROTATION);
  fillScreen(0);
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
  #if defined (ARDUINO_TTGO_T1)
    if (mcpx != nullptr) {
      // either ON/OFF
      mcpx->enableTFT_BL(brightness > 0);
    }
  #endif
}

void M5Display::progressBar(int x, int y, int w, int h, uint8_t val) {
  drawRect(x, y, w, h, 0x09F1);
  fillRect(x + 1, y + 1, w * (((float)val) / 100.0), h - 1, 0x09F1);
}

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
