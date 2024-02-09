#include "MCPXManager.h"


MCPXManager::MCPXManager(uint8_t address, TwoWire& bus, uint32_t dbTime) : mcpx(address, bus), dbTime(dbTime) {
    lastReadMs = 0;
    state = std::make_pair(0, 0);
}


void MCPXManager::begin() {
    mcpx.begin();
    for (uint8_t pin : btnPins) {
        mcpx.pinMode(pin, 1);  // set as input
        log_i("MCP23017: set pin %d as button input", pin);
    }
    btnPins.clear();
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


void MCPXManager::registerButton(uint8_t pin) {
    btnPins.push_back(pin);
}


void MCPXManager::enableLoRa(uint8_t rstPin) {
   mcpx.digitalWrite(rstPin, 0);
   delay(100);
   mcpx.digitalWrite(rstPin, 1);
}


void MCPXManager::enableTFT(uint8_t rstPin, uint8_t blPin) {
   mcpx.digitalWrite(rstPin, 0);
   delay(100);
   mcpx.digitalWrite(rstPin, 1);
   mcpx.digitalWrite(blPin, 1);
}
