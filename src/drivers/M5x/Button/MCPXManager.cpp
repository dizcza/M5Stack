#include "MCPXManager.h"
#include "esp_task_wdt.h"


MCPXManager::MCPXManager(uint8_t address, TwoWire& bus, uint32_t dbTime) : mcpx(address, bus), dbTime(dbTime) {
}


bool MCPXManager::begin() {
    if (!mcpx.begin()) {
        log_e("MCP23017 begin failed");
        return false;
    }
    configurePins();
    enableTFT(true);
    enableLoRa(true);
    enableGPS(true);
    return true;
}


void MCPXManager::addButton(ExpanderButton& btn) {
    userButtons.push_back(btn);
}


void MCPXManager::update() {
    ButtonDebounceState state = read();
    for (auto&& btn : userButtons) {
        btn.setState(state);
    }
}


ButtonDebounceState MCPXManager::read() {
    ButtonDebounceState state;
    state.val1 = mcpx.read();
    delay(dbTime);
    state.val2 = mcpx.read();
    return state;
}


void MCPXManager::configurePins() {
    // 1 - input, 0 - output
    mcpx.pinMode(MCP_EXPANDER_LORA_RST_PIN, 0);
    mcpx.pinMode(MCP_EXPANDER_BTN_A_PIN, 1);
    mcpx.pinMode(MCP_EXPANDER_BTN_B_PIN, 1);
    mcpx.pinMode(MCP_EXPANDER_BTN_C_PIN, 1);
    mcpx.pinMode(MCP_EXPANDER_IMU_INT1_PIN, 1);
    mcpx.pinMode(MCP_EXPANDER_IMU_INT2_PIN, 1);
    mcpx.pinMode(MCP_EXPANDER_MAG_INT_PIN, 1);
    mcpx.pinMode(MCP_EXPANDER_GPS_EXTINT_PIN, 0);
    mcpx.pinMode(MCP_EXPANDER_GPS_RST_PIN, 0);
    mcpx.pinMode(MCP_EXPANDER_RTK_STAT_PIN, 1);
    mcpx.pinMode(MCP_EXPANDER_TFT_RST_PIN, 0);
    mcpx.pinMode(MCP_EXPANDER_TFT_BL_PIN, 0);
}


void MCPXManager::wakeUpGPS() {
    mcpx.digitalWrite(MCP_EXPANDER_GPS_EXTINT_PIN, 0);
    delay(1000);
    mcpx.digitalWrite(MCP_EXPANDER_GPS_EXTINT_PIN, 1);
    delay(1000);
    mcpx.digitalWrite(MCP_EXPANDER_GPS_EXTINT_PIN, 0);
    log_d("GPS woken");
}


void MCPXManager::enableGPS(bool enable) {
    mcpx.digitalWrite(MCP_EXPANDER_GPS_RST_PIN, 0);
    if (enable) {
        delay(100);
        mcpx.digitalWrite(MCP_EXPANDER_GPS_RST_PIN, 1);
        mcpx.digitalWrite(MCP_EXPANDER_GPS_EXTINT_PIN, 0);
    }
}


void MCPXManager::enableLoRa(bool enable)
{
    mcpx.digitalWrite(MCP_EXPANDER_LORA_RST_PIN, 0);
    if (enable) {
        delay(100);
        mcpx.digitalWrite(MCP_EXPANDER_LORA_RST_PIN, 1);
    }
}

void MCPXManager::enableTFT(bool enable)
{
    mcpx.digitalWrite(MCP_EXPANDER_TFT_RST_PIN, 0);
    if (enable) {
        delay(100);
        mcpx.digitalWrite(MCP_EXPANDER_TFT_RST_PIN, 1);
    }
    enableTFT_BL(enable);
}

void MCPXManager::enableTFT_BL(bool enable) {
    mcpx.digitalWrite(MCP_EXPANDER_TFT_BL_PIN, enable);
}
