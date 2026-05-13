#include "app_config.h"
#include "app_events.h"
#include "app_state.h"
#include "crsf/crsf_rx.h"
#include "crsf/crsf_tx.h"
#include "esp_err.h"
#include "esp_log.h"
#include "gps/gps_input.h"
#include "nvs_flash.h"
#include "simulator/simulator.h"
#include "ui/ui.h"
#include "web/web_ui.h"
#include "wifi/wifi_manager.h"

static const char *TAG = "app_main";

static void log_start_result(const char *name, esp_err_t err)
{
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "%s started", name);
    } else {
        ESP_LOGE(TAG, "%s failed: %s", name, esp_err_to_name(err));
    }
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    app_state_init();
    ESP_ERROR_CHECK(app_events_init(APP_EVENTS_QUEUE_LENGTH));

    err = wifi_manager_start();
    log_start_result("wifi_manager", err);

    err = web_ui_start();
    log_start_result("web_ui", err);

    err = ui_start();
    log_start_result("ui", err);

    err = crsf_rx_start();
    log_start_result("crsf_rx", err);

    err = crsf_tx_start();
    log_start_result("crsf_tx", err);

    err = gps_input_start();
    log_start_result("gps_input", err);

    err = simulator_start();
    log_start_result("simulator", err);

    ESP_LOGI(TAG, "CRSF Display bootstrap complete");
}
