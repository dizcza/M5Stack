#ifndef _M5DISPLAY_H_
  #define _M5DISPLAY_H_

  #include <vector>
  #include <Arduino.h>
  #include <SPI.h>

  #include "utility/Config.h"
  #include "utility/In_eSPI.h"

  #include "drivers/M5x/Button/MCPXManager.h"

  struct DisplayState {
    uint8_t textfont, textsize, datum;
    const GFXfont *gfxFont;
    uint32_t textcolor, textbgcolor;
    int32_t cursor_x, cursor_y, padX;
  };

  class M5Display : public TFT_eSPI {
    private:
      MCPXManager* mcpx = nullptr;
    public:
      static M5Display* instance;
      M5Display();
      void begin();
      void sleep();
      void wakeup();
      void setBrightness(uint8_t brightness);
      void clearDisplay(uint32_t color=TFT_BLACK) { fillScreen(color); }
      void clear(uint32_t color=TFT_BLACK) { fillScreen(color); }
      void display() {}
      void setGPIOExpander(MCPXManager* mcpx) { this->mcpx = mcpx; }

      inline void startWrite(void){
        #if defined (SPI_HAS_TRANSACTION) && defined (SUPPORT_TRANSACTIONS) && !defined(ESP32_PARALLEL)
          if (locked) {
            locked = false; SPI.beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, SPI_MODE0));
          }
        #endif
        CS_L;
      }
      inline void endWrite(void){
        #if defined (SPI_HAS_TRANSACTION) && defined (SUPPORT_TRANSACTIONS) && !defined(ESP32_PARALLEL)
          if(!inTransaction) {
            if (!locked) {
              locked = true;
              SPI.endTransaction();
            }
          }
        #endif
        CS_H;
      }
      inline void writePixel(uint16_t color) {
        SPI.write16(color);
      }
      inline void writePixels(uint16_t * colors, uint32_t len){
        SPI.writePixels((uint8_t*)colors , len * 2);
      }
      void progressBar(int x, int y, int w, int h, uint8_t val);

      #define setFont setFreeFont


    // Saves and restores font properties, datum, cursor and colors so
    // code can be non-invasive. Just make sure that every push is also
    // popped when you're done to prevent stack from growing.
    //
    // (User code can never do this completely because the gfxFont
    // class variable of TFT_eSPI is protected.)
    #define M5DISPLAY_HAS_PUSH_POP
     public:
      void pushState();
      void popState();

     private:
      std::vector<DisplayState> _displayStateStack;
};
#endif /* _M5DISPLAY_H_ */
