#include "MCPXManager.h"


MCPXManager::MCPXManager(uint8_t address, TwoWire& bus, uint32_t dbTime) : mcpx(address, bus), dbTime(dbTime) {
    lastReadMs = 0;
    state = std::make_pair(0, 0);
}


void MCPXManager::begin() {
    mcpx.begin();
    mcpx.pinMode(MCP_EXPANDER_LORA_RST_PIN, 0);
    mcpx.pinMode(MCP_EXPANDER_TFT_RST_PIN, 0);
    mcpx.pinMode(MCP_EXPANDER_TFT_BL_PIN, 0);
    for (uint8_t pin : btnPins) {
        mcpx.pinMode(pin, 1);  // set as input
        log_i("MCP23017: set pin %d as button input", pin);
    }
    btnPins.clear();
    enableTFT();
    enableLoRa();
}


std::pair<uint16_t, uint16_t> MCPXManager::read() {
    if (millis() - lastReadMs > dbTime) {
        uint16_t val1 = mcpx.read();
        delay(dbTime);
        uint16_t val2 = mcpx.read();
        state = std::make_pair(val1, val2);
        lastReadMs = millis();
    }
    return state;
}


void MCPXManager::wakeUpGPS() {
    mcpx.digitalWrite(MCP_EXPANDER_GPS_EXTINT_PIN, 0);
    delay(1000);
    mcpx.digitalWrite(MCP_EXPANDER_GPS_EXTINT_PIN, 1);
    delay(1000);
    mcpx.digitalWrite(MCP_EXPANDER_GPS_EXTINT_PIN, 0);
    log_d("GPS woken");
}


void MCPXManager::registerButton(uint8_t pin) {
    btnPins.push_back(pin);
}

void MCPXManager::enableLoRa()
{
    mcpx.digitalWrite(MCP_EXPANDER_LORA_RST_PIN, 0);
    delay(100);
    mcpx.digitalWrite(MCP_EXPANDER_LORA_RST_PIN, 1);
}

void MCPXManager::enableTFT()
{
    mcpx.digitalWrite(MCP_EXPANDER_TFT_RST_PIN, 0);
    delay(100);
    mcpx.digitalWrite(MCP_EXPANDER_TFT_RST_PIN, 1);
    mcpx.digitalWrite(MCP_EXPANDER_TFT_BL_PIN, 1);
}
