#include "MCPXManager.h"
#include "esp_task_wdt.h"


MCPXManager::MCPXManager(uint8_t address, TwoWire& bus, uint32_t dbTime) : mcpx(address, bus), dbTime(dbTime) {
    statesQueue = xQueueCreate( 30, sizeof(ButtonDebounceState) );
}


void MCPXManager::begin() {
    mcpx.begin();
    configurePins();
    enableTFT();
    enableLoRa();
    enableGPS();
}


void MCPXManager::addButton(MCPBtn* btn) {
    userButtons.push_back(btn);
}


void MCPXManager::startReadButtonsTask() {
    if (taskStarted) {
        log_i("Task already started");
        return;
    }
    for (MCPBtn* btn : userButtons) {
        privateButtons.push_back(*btn);
    }
    xTaskCreatePinnedToCore(MCPXManager::readButtonsTask, "mcpx_update", 4096, this, 1, NULL, PRO_CPU_NUM);
    taskStarted = true;
}


void MCPXManager::update() {
    ButtonDebounceState state;
    bool received = false;
    if (taskStarted) {
        if (uxQueueMessagesWaiting(statesQueue) > 1) {
            received = xQueueReceive(statesQueue, &state, 0) == pdTRUE;
        } else {
            received = xQueuePeek(statesQueue, &state, 0) == pdTRUE;
        }
    } else {
        state = read();
        received = true;
    }

    if (received) {
        for (MCPBtn* btn : userButtons) {
            btn->setState(state);
        }
    }
}


bool MCPXManager::stateChanged(ButtonDebounceState state) {
    bool changed = false;
    for (MCPBtn& btn : privateButtons) {
        if (btn.stateChanged(state)) {
            btn.updateStateFromPair(state);
            changed = true;
        }
    }
    return changed;
}


void MCPXManager::saveLastReading() {
    ButtonDebounceState state = read();
    if (stateChanged(state)) {
        xQueueSend(statesQueue, &state, pdMS_TO_TICKS(10));
    }
}


void MCPXManager::readButtonsTask(void *args) {
    log_i("readButtonsTask started");
    MCPXManager* mng = (MCPXManager*) args;
    esp_task_wdt_add(NULL);
    while (true) {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(50));
        mng->saveLastReading();
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
