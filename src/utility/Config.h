#ifndef M5_CONFIG_H_
  #define M5_CONFIG_H_

  #if defined (ARDUINO_TTGO_T1)
    #define BUTTON_A_PIN      5
    #define BUTTON_B_PIN      5
    #define BUTTON_C_PIN      5

    #define M5STACK
    #define ILI9341_DRIVER
    #define SPI_FREQUENCY      40000000
    #define SPI_READ_FREQUENCY 20000000
    #define DEFAULT_ROTATION   1
    #define TFT_SDA_READ

    #define TFT_CS            27
    #undef TFT_RST
    #define TFT_DC            26
    #define TFT_MOSI          15
    #define TFT_SCLK          14
    #undef TFT_BL
    #define TFT_MISO          2

    #undef BTN_BL

    #define SD_ENABLE          1
    #define TFCARD_CS_PIN      13
    #define TFCARD_MISO_PIN    TFT_MISO
    #define TFCARD_MOSI_PIN    TFT_MOSI
    #define TFCARD_SCLK_PIN    TFT_SCLK

    //SX1276 - RadioHead names
    #define RADIO_RST         -1
    #define RADIO_NSS         25
    #define RADIO_INT         12
    #define RADIO_SCK         TFT_SCLK
    #define RADIO_MISO        TFT_MISO
    #define RADIO_MOSI        TFT_MOSI

    #undef PIN_VBAT_TEST
    #undef CHAN_VBAT_ADC

  #endif

  #define ILI9341_SLPIN   0x10
  #define ILI9341_SLPOUT  0x11
  #define ILI9341_DISPOFF 0x28
  #define ILI9341_DISPON  0x29
  #define TFT_SLPIN       0x10
  #define TFT_SLPOUT      0x11
  #define TFT_DISPOFF     0x28
  #define TFT_DISPON      0x29

#endif /* M5_CONFIG_H_ */
