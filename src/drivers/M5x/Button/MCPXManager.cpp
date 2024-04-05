#include "MCPXManager.h"
#include "esp_task_wdt.h"


MCPXManager::MCPXManager(uint8_t address, TwoWire& bus, uint32_t dbTime) : mcpx(address, bus), dbTime(dbTime) {
    statesQueue = xQueueCreate( 30, sizeof(ButtonDebounceState) );
    memset(&state, 0, sizeof(state));
    privateButtons.emplace_back(MCP_EXPANDER_BTN_A_PIN, true);
    privateButtons.emplace_back(MCP_EXPANDER_BTN_B_PIN, true);
    privateButtons.emplace_back(MCP_EXPANDER_BTN_C_PIN, true);
}


void MCPXManager::begin() {
    mcpx.begin();
    configurePins();
    enableTFT();
    enableLoRa();
    enableGPS();
    startUpdateTask();
}


void MCPXManager::addButton(MCPBtn& btn) {
    userButtons.push_back(btn);
}


void MCPXManager::startUpdateTask() {
    xTaskCreatePinnedToCore(MCPXManager::updateTask, "mcpx_update", 4096, this, 1, NULL, PRO_CPU_NUM);
}


void MCPXManager::nextState() {
    ButtonDebounceState st;
    if (xQueueReceive(statesQueue, &st, 0) == pdTRUE) {
        for (MCPBtn& btn : userButtons) {
            btn.setState(st);
        }
    }
}


bool MCPXManager::stateChanged() {
    for (MCPBtn& btn : privateButtons) {
        if (btn.stateChanged(state)) {
            return true;
        }
    }
    return false;
}


void MCPXManager::saveLastReading() {
    read();
    if (stateChanged()) {
        xQueueSend(statesQueue, &state, pdMS_TO_TICKS(10));
    }
}


void MCPXManager::updateTask(void *args) {
    log_i("updateTask started");
    MCPXManager* mng = (MCPXManager*) args;
    esp_task_wdt_add(NULL);
    while (true) {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(50));
        mng->saveLastReading();
    }
}


void MCPXManager::read() {
    uint16_t val1 = mcpx.read();
    delay(dbTime);
    uint16_t val2 = mcpx.read();
    state.val1 = val1;
    state.val2 = val2;
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


void MCPXManager::enableGPS() {
    mcpx.digitalWrite(MCP_EXPANDER_GPS_RST_PIN, 0);
    delay(100);
    mcpx.digitalWrite(MCP_EXPANDER_GPS_RST_PIN, 1);
    mcpx.digitalWrite(MCP_EXPANDER_GPS_EXTINT_PIN, 0);
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
    enableTFT_BL(true);
}

void MCPXManager::enableTFT_BL(bool enable) {
    mcpx.digitalWrite(MCP_EXPANDER_TFT_BL_PIN, enable);
}
