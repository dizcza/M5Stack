#ifndef _CONFIG_H_
  #define _CONFIG_H_

  #define AXP     AXP192::instance
  #define TFT     M5Display::instance
  //#define TFT_eSPI_TOUCH_EMULATION
  #define TOUCH		M5Touch::instance
  #define BUTTONS	M5Buttons::instance
  #define IOE     MAX7315::instance
  #define DAC     WM8978::instance

  #if defined (ARDUINO_M5Stack_Core_ESP32)
    #define BUTTON_A_PIN      39
    #define BUTTON_B_PIN      38
    #define BUTTON_C_PIN      37

    //In_eSPI_Setup settings first
    #define M5STACK
    #define M5CORE
    #define ILI9341_DRIVER
    #define SPI_FREQUENCY      40000000
    #define SPI_READ_FREQUENCY 20000000
    #define DEFAULT_ROTATION   1
    #define TFT_SDA_READ

    #define TFT_CS            14
    #define TFT_SCLK          18
    #define TFT_MISO          19
    #define TFT_MOSI          23
    #define TFT_DC            27
    #define TFT_BL            32
    #define TFT_RST           33

    #define SD_ENABLE          0
    #define TFCARD_CS_PIN      4
    #define TFCARD_MISO_PIN    2
    #define TFCARD_MOSI_PIN   15
    #define TFCARD_SCLK_PIN   14

    // #define SPEAKER_PIN       25

    // RA-1H/SX1276 - RadioHead names
    #define RADIO_NSS          5
    #define RADIO_RST         -1  //26
    #define RADIO_INT         36
    #define RADIO_SCK         18
    #define RADIO_MISO        19
    #define RADIO_MOSI        23

    #define PIN_VBAT_TEST     18  //not used
  #elif defined (ARDUINO_M5STACK_FIRE)  //K45
    #define BUTTON_A_PIN      39
    #define BUTTON_B_PIN      38
    #define BUTTON_C_PIN      37

    #define M5STACK
    #define M5CORE
    #define ILI9341_DRIVER
    #define SPI_FREQUENCY      40000000
    #define SPI_READ_FREQUENCY 20000000
    #define DEFAULT_ROTATION   1
    #define TFT_SDA_READ

    #define TFT_CS            14
    #define TFT_RST           33
    #define TFT_DC            27
    #define TFT_MOSI          23
    #define TFT_SCLK          18
    #define TFT_BL            32
    #define TFT_MISO          19

    #define SD_ENABLE          0
    // #define TFCARD_CS_PIN      4
    // #define TFCARD_MISO_PIN    2
    // #define TFCARD_MOSI_PIN   15
    // #define TFCARD_SCLK_PIN   14

    #define SPEAKER_PIN       25

    #define MPU9250_INSDE

    // RFM95C - RadioHead names
    #define RADIO_NSS          5
    #define RADIO_RST         26
    #define RADIO_INT         36
    #define RADIO_SCK         18
    #define RADIO_MISO        19
    #define RADIO_MOSI        23

  #elif defined (ARDUINO_WESP32)  //K46v1
    #define BUTTON_A_PIN      33
    #define BUTTON_B_PIN      13
    #define BUTTON_C_PIN       4

    #define M5STACK
    #define ILI9341_DRIVER
    #define SPI_FREQUENCY      40000000
    #define SPI_READ_FREQUENCY 20000000
    #define DEFAULT_ROTATION   1
    #define TFT_SDA_READ

    #define TFT_CS            17
    #define TFT_RST            5
    #define TFT_DC            27
    #define TFT_MOSI           2
    #define TFT_SCLK          15
    #define TFT_BL            14
    #define TFT_MISO          16

    #define BTN_BL            26

    #define SD_ENABLE          0

    // SX1276 - RadioHead names
    #define RADIO_NSS         25
    #define RADIO_RST         -1
    #define RADIO_INT         36
    #define RADIO_SCK         15
    #define RADIO_MISO        16
    #define RADIO_MOSI         2

    #define PIN_VBAT_TEST     18
    #define CHAN_VBAT_ADC     ADC1_CHANNEL_4
  
  #elif defined (ARDUINO_TTGO_T1)
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
    #define TFT_BL            4
    #define TFT_MISO          2

    #undef BTN_BL

    #define SD_ENABLE          0
    #define TFCARD_CS_PIN      13
    #define TFCARD_MISO_PIN    TFT_MISO
    #define TFCARD_MOSI_PIN    TFT_MOSI
    #define TFCARD_SCLK_PIN    TFT_SCLK

    //SX1276 - RadioHead names
    #define RADIO_RST         -1
    #define RADIO_NSS         12
    #define RADIO_SCK         TFT_SCLK
    #define RADIO_MISO        TFT_MISO
    #define RADIO_MOSI        TFT_MOSI

    #undef PIN_VBAT_TEST
    #undef CHAN_VBAT_ADC

  #elif defined (ARDUINO_FROG_ESP32)  //K46v4
    #define BUTTON_A_PIN      33
    #define BUTTON_B_PIN      13
    #define BUTTON_C_PIN       4

    #define M5STACK
    #define ILI9341_DRIVER
    #define SPI_FREQUENCY      40000000
    #define SPI_READ_FREQUENCY 20000000
    #define DEFAULT_ROTATION   1
    #define TFT_SDA_READ

    #define TFT_CS            17
    #define TFT_RST            5
    #define TFT_DC            27
    #define TFT_MOSI           2
    #define TFT_SCLK          15
    #define TFT_BL            14
    #define TFT_MISO          16

    #define BTN_BL            19

    #define SD_ENABLE          0

    //SX1276 - RadioHead names
    #define RADIO_RST         -1
    #define RADIO_NSS         25
    #define RADIO_SCK         15
    #define RADIO_MISO        16
    #define RADIO_MOSI         2

    #define PIN_VBAT_TEST     18
    #define CHAN_VBAT_ADC     ADC1_CHANNEL_6

  #elif defined (ARDUINO_TWatch)
    #define ST7789_DRIVER
    #define SPI_FREQUENCY      27000000
    #define SPI_READ_FREQUENCY 16000000
    #define DEFAULT_ROTATION   1

    #define TFT_CS             5
    #define TFT_RST           -1
    #define TFT_DC            27
    #define TFT_MOSI          19
    #define TFT_SCLK          18
    #define TFT_BL            12
    #define TFT_MISO          -1

    #define CST_INT           38
    #define TOUCH_W          240
    #define TOUCH_H          240

    #define SD_ENABLE          0
    #define TFCARD_CS_PIN     13
    #define TFCARD_MISO_PIN    2
    #define TFCARD_MOSI_PIN   15
    #define TFCARD_SCLK_PIN   14

    #define SPEAKER_PIN       -1
    #define PIN_IRQ_AXP       35  //AXP202
    #define PIN_BUTTON        36
    #define PIN_IRQ_RTC       37  //PCF8563
    #define PIN_IRQ_CTS       38  //FT6236
    #define PIN_IRQ_IMU       39  //BMA423
    #define GPIO_IRQ_IMU      GPIO_NUM_39

    // TTGO LoRa/SX1276 - RadioHead names
    #define RADIO_NSS         13
    #define RADIO_RST         25
    #define RADIO_INT         26
    #define RADIO_SCK         14
    #define RADIO_MISO        2
    #define RADIO_MOSI        15

  #elif defined (ARDUINO_M5STACK_Core2)
    #define M5Stack_M5Core2
    #define M5STACK
    #define ILI9341_DRIVER
    #define SPI_FREQUENCY      40000000
    #define SPI_READ_FREQUENCY 16000000
    #define DEFAULT_ROTATION   1
    #define TFT_SDA_READ

    #define TFT_CS             5
    #define TFT_RST           -1
    #define TFT_DC            15
    #define TFT_MOSI          23
    #define TFT_SCLK          18
    #define TFT_BL            -1
    #define TFT_MISO          38

    #define CST_INT           39
    #define TOUCH_W          320
    #define TOUCH_H          280

    #define SD_ENABLE          1
    #define TFCARD_CS_PIN      4
    #define TFCARD_MISO_PIN    2
    #define TFCARD_MOSI_PIN   15
    #define TFCARD_SCLK_PIN   14

    #define SPEAKER_PIN       -1

    // RA-1H/SX1276 - RadioHead names
    #define RADIO_NSS         26
    #define RADIO_RST         -1
    #define RADIO_INT         36
    #define RADIO_SCK         18
    #define RADIO_MISO        19
    #define RADIO_MOSI        23

  #elif defined (ARDUINO_M5Stick_C) // M5Stick C Plus
    #define BUTTON_A_PIN      37
    #define BUTTON_B_PIN      39
    #define BUTTON_C_PIN      37

    #define M5_BUTTON_HOME    37
    #define M5_BUTTON_RST     39

    #define ST7789_DRIVER
    #define SPI_FREQUENCY      27000000
    #define SPI_READ_FREQUENCY 20000000
    #define DEFAULT_ROTATION   1

    #define TFT_CS             5
    #define TFT_RST           18
    #define TFT_DC            23
    #define TFT_MOSI          15
    #define TFT_SCLK          13
    #define TFT_MISO          14

    #define M5_IR              9
    #define M5_LED            10
    #define SPEAKER_PIN       26
    #define SPEAKER_EN_PIN     0  // Low - shutdown

    #define IMU_IRQ_PIN       35

    #define SD_ENABLE          0

  #elif defined (ARDUINO_LOLIN_D32_PRO) // Select "LOLIN D32 PRO" for TTGO T4 v1.3 w/portrait mode
    #define BUTTON_A_PIN      38
    #define BUTTON_B_PIN      37
    #define BUTTON_C_PIN      39

    #define ILI9341_DRIVER
    #define DEFAULT_ROTATION   0
    #define SPI_FREQUENCY      40000000
    #define SPI_READ_FREQUENCY 6000000
    #define USE_HSPI_PORT

    #define TFT_CS            27
    #define TFT_RST            5
    #define TFT_DC            32
    #define TFT_MOSI          23
    #define TFT_SCLK          18
    #define TFT_BL             4
    #define TFT_MISO          12
    #define TFT_BACKLIGHT_ON HIGH

    #define SD_ENABLE          1
    #define TFCARD_CS_PIN     13
    #define TFCARD_MISO_PIN    2
    #define TFCARD_MOSI_PIN   15
    #define TFCARD_SCLK_PIN   14

    #define SPEAKER_PIN       25  // Connected to NS4150 via C only... unusable
    #define SPEAKER_EN_PIN    19  // Low - shutdown

  #elif defined (ARDUINO_ESP32_DEV) //Select "ESP Wrover Module" for M35 = WROVERB + BTNx3 + 3.5" ILI9488 + CTS
    #define BUTTON_A_PIN      39
    #define BUTTON_B_PIN      35
    #define BUTTON_C_PIN      34

    #define ILI9488_DRIVER
    #define SPI_FREQUENCY      40000000
    #define SPI_READ_FREQUENCY 20000000
    #define DEFAULT_ROTATION   0
    
    #define TFT_CS             5
    #define TFT_RST           -1
    #define TFT_DC            32
    #define TFT_MOSI          23
    #define TFT_SCLK          18
    #define TFT_MISO          19

    //#define CST_INT            4
    #define TOUCH_W          320
    #define TOUCH_H          480
    #define TOUCH		M5Touch::instance

    #define SD_ENABLE          1
    #define TFCARD_CS_PIN     33
    #define TFCARD_SCLK_PIN   18
    #define TFCARD_MISO_PIN   19
    #define TFCARD_MOSI_PIN   23
    #define TFCARD_USE_WIRE1

    // Interupt pins
    #define GNSS_IRQ_PIN       2
    #define RTC_IRQ_PIN       26
    #define IOE_IRQ_PIN       27

    // RA-1H/SX1276 - RadioHead names
    #define RADIO_NSS         33
    #define RADIO_INT         36
    #define RADIO_SCK         18
    #define RADIO_MISO        19
    #define RADIO_MOSI        23
    
    #define ETH_MISO_PIN      12
    #define ETH_MOSI_PIN      13
    #define ETH_SCLK_PIN      14  
    #define ETH_CS_PIN        15
    #define ETH_IRQ_PIN       25
    // UART
    #define USE_SERIAL    Serial

  #elif defined (ARDUINO_D1_MINI32) //Select "WEMOS D1 MINI ESP32" for non-PSRAM custom boards
    #define BUTTON_A_PIN      39
    #define BUTTON_B_PIN      16
    #define BUTTON_C_PIN       5

    #define ILI9341_DRIVER
    #define SPI_FREQUENCY      40000000
    #define SPI_READ_FREQUENCY 20000000
    #define DEFAULT_ROTATION   4

    #define TFT_CS            32
    #define TFT_RST           33
    #define TFT_DC            27
    #define TFT_MOSI          23
    #define TFT_SCLK          18
    #define TFT_MISO          19

    #define SD_ENABLE          0

    // Interupt pins
    #define GNSS_IRQ_PIN      26
    #define RTC_IRQ_PIN        4
    #define DMP_IRQ_PIN       25
    #define IOE_IRQ_PIN       35
    #define KEEP_PWR_PIN       2

    // RA-1H/SX1276 - RadioHead names
    #define RADIO_NSS         17
    #define RADIO_RST         -1
    #define RADIO_INT         36
    #define RADIO_SCK         18
    #define RADIO_MISO        19
    #define RADIO_MOSI        23

    // UART
    #define USE_SERIAL Serial

  #endif

  #define BTN_A     0
  #define BTN_B     1
  #define BTN_C     2
  #define BUTTON_A  0
  #define BUTTON_B  1
  #define BUTTON_C  2

  #define TONE_PIN_CHANNEL 0

  #define ILI9341_SLPIN   0x10
  #define ILI9341_SLPOUT  0x11
  #define ILI9341_DISPOFF 0x28
  #define ILI9341_DISPON  0x29
  #define TFT_SLPIN       0x10
  #define TFT_SLPOUT      0x11
  #define TFT_DISPOFF     0x28
  #define TFT_DISPON      0x29

#endif /* SETTINGS_C */
