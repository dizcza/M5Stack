#pragma once

#include <Arduino.h>
#include <vector>
#include <Wire.h>
#include "drivers/SRS/MCP23017/MCP23017.h"
#include "Button.h"


#define MCP_EXPANDER_LORA_RST_PIN    0
#define MCP_EXPANDER_BTN_A_PIN       2
#define MCP_EXPANDER_BTN_B_PIN       3
#define MCP_EXPANDER_BTN_C_PIN       4
#define MCP_EXPANDER_IMU_INT1_PIN    8
#define MCP_EXPANDER_IMU_INT2_PIN    9
#define MCP_EXPANDER_MAG_INT_PIN     10
#define MCP_EXPANDER_GPS_EXTINT_PIN  11
#define MCP_EXPANDER_GPS_RST_PIN     12
#define MCP_EXPANDER_RTK_STAT_PIN    13
#define MCP_EXPANDER_TFT_RST_PIN     14
#define MCP_EXPANDER_TFT_BL_PIN      15


/**
 * MCP23017 GPIO Expander manager.
*/
class MCPXManager {
    public:
        MCP23017 mcpx;

        MCPXManager(uint8_t address = MCP23017_I2C_ADDRESS, TwoWire& bus = Wire, uint32_t dbTime = 10);
        void begin();
        void enableLoRa();                 // reset LoRa
        void enableTFT();                  // reset TFT
        void enableTFT_BL(bool enable);    // enable TFT backlight
        void enableGPS();                  // reset GPS
        void wakeUpGPS();                  // toggle GPS EXTI pin

        void addButton(MCPBtn& btn);
        void nextState();

    protected:
        QueueHandle_t statesQueue;
        uint32_t dbTime;
        ButtonDebounceState state;
        std::vector<MCPBtn> privateButtons;
        std::vector<MCPBtn> userButtons;

        void startUpdateTask();
        void configurePins();
        bool stateChanged();
        void read();
        void saveLastReading();

    private:
        static void updateTask(void *args);
};
