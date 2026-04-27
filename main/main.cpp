#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "app_config.hpp"
#include "pinmap.hpp"

namespace {
constexpr const char* TAG = "drone";
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Drone ESP32-S3 N16R8 boot  fw %s  target %s",
             drone::config::FIRMWARE_VERSION, drone::config::TARGET_BOARD);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // TODO Phase 1+: spawn flight_engine, networking, telemetry tasks.
    ESP_LOGW(TAG, "Stub: nenhuma task iniciada. Aguardando Phase 1.");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
