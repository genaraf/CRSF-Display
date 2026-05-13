#include "app_config.h"
#include "app_events.h"
#include "app_state.h"
#include <stdlib.h>
#include <string.h>
#include "crsf/crsf_rx.h"
#include "crsf/crsf_tx.h"
#include "display/display.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gps/gps_input.h"
#include "input/buttons.h"
#include "nvs_flash.h"
#include "sdcard/sdcard_json.h"
#include "simulator/simulator.h"
#include "storage/storage.h"
#include "ui/ui.h"
#include "ui/ui_lvgl.h"
#include "web/web_ui.h"
#include "wifi/wifi_manager.h"

static const char *TAG = "app_main";

typedef struct {
    char active_profile_path[APP_MAX_PROFILE_PATH_LEN];
} bootstrap_profile_state_t;

static void log_start_result(const char *name, esp_err_t err)
{
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "%s started", name);
    } else {
        ESP_LOGE(TAG, "%s failed: %s", name, esp_err_to_name(err));
    }
}

static void capture_bootstrap_profile_state(const app_state_t *state, void *ctx)
{
    bootstrap_profile_state_t *profile_state = (bootstrap_profile_state_t *) ctx;

    if (profile_state == NULL) {
        return;
    }

    snprintf(profile_state->active_profile_path,
             sizeof(profile_state->active_profile_path),
             "%s",
             state->simulator.active_profile_path);
}

static void load_initial_simulator_profile(void)
{
    bootstrap_profile_state_t profile_state = {0};
    esp_err_t err;

    err = sdcard_json_init();
    log_start_result("sdcard_json", err);
    if (err != ESP_OK) {
        return;
    }

    app_state_read(capture_bootstrap_profile_state, &profile_state);
    err = sdcard_json_load_profile(profile_state.active_profile_path);
    if ((err != ESP_OK) &&
        (strcmp(profile_state.active_profile_path, APP_SIMULATOR_DEFAULT_PROFILE_PATH) != 0)) {
        err = sdcard_json_load_profile(APP_SIMULATOR_DEFAULT_PROFILE_PATH);
    }

    if (err != ESP_OK) {
        err = sdcard_json_save_profile(APP_SIMULATOR_DEFAULT_PROFILE_PATH, false);
    }

    log_start_result("simulator_profile", err);
    if (err == ESP_OK) {
        (void) storage_save_runtime_state();
    }
}

static void bootstrap_task(void *arg)
{
    (void) arg;
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    app_state_init();
    ESP_ERROR_CHECK(app_events_init(APP_EVENTS_QUEUE_LENGTH));
    err = storage_load_runtime_state();
    log_start_result("storage_load", err);

    err = display_init();
    log_start_result("display", err);
    if (err == ESP_OK) {
        err = ui_lvgl_init();
        log_start_result("ui_lvgl", err);
    }

    load_initial_simulator_profile();

    err = wifi_manager_start();
    log_start_result("wifi_manager", err);

    err = web_ui_start();
    log_start_result("web_ui", err);

    err = buttons_start();
    log_start_result("buttons", err);

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
    vTaskDelete(NULL);
}

void app_main(void)
{
    BaseType_t result = xTaskCreate(bootstrap_task,
                                    "bootstrap",
                                    APP_TASK_STACK_BOOTSTRAP,
                                    NULL,
                                    APP_TASK_PRIORITY_IO,
                                    NULL);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create bootstrap task");
        abort();
    }
}
