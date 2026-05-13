#include "crsf/crsf_tx.h"

#include <stdio.h>

#include "app_config.h"
#include "app_state.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "crsf_tx";

static void update_tx_state(app_state_t *state, void *ctx)
{
    (void) ctx;

    state->telemetry_tx.status = TELEMETRY_TX_OK;
    state->telemetry_tx.last_tx_age_ms = 0;
    state->telemetry.timestamp_ms = app_state_now_ms();
}

static void crsf_tx_task(void *arg)
{
    (void) arg;

    ESP_LOGI(TAG, "Starting CRSF telemetry TX stub task");

    while (true) {
        app_state_write(update_tx_state, NULL);
        vTaskDelay(pdMS_TO_TICKS(APP_TELEMETRY_TX_REFRESH_MS));
    }
}

esp_err_t crsf_tx_start(void)
{
    BaseType_t result = xTaskCreate(crsf_tx_task, "task_telemetry_tx", APP_TASK_STACK_MEDIUM, NULL, APP_TASK_PRIORITY_NORMAL, NULL);
    return (result == pdPASS) ? ESP_OK : ESP_FAIL;
}
