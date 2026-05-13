#include "crsf/crsf_rx.h"

#include <math.h>

#include "app_config.h"
#include "app_state.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "crsf_rx";

typedef struct {
    float phase;
} crsf_stub_context_t;

static void update_crsf_state(app_state_t *state, void *ctx)
{
    crsf_stub_context_t *stub = (crsf_stub_context_t *) ctx;

    state->crsf_link.link_up = true;
    state->crsf_link.rx_active = true;
    state->crsf_link.frame_error = false;
    state->crsf_link.last_frame_age_ms = 0;

    for (int i = 0; i < APP_RX_CHANNEL_COUNT; ++i) {
        const float offset = stub->phase + ((float) i * 0.35f);
        state->rx_channels[i].normalized_value = sinf(offset) * 100.0f;
    }

    state->link_statistics.uplink_rssi_ant1_dbm_neg = 58 + (int) (sinf(stub->phase) * 3.0f);
    state->link_statistics.uplink_rssi_ant2_dbm_neg = 61 + (int) (cosf(stub->phase) * 3.0f);
    state->link_statistics.uplink_lq_percent = 97 + (int) ((sinf(stub->phase * 0.5f) + 1.0f) * 1.5f);
    state->link_statistics.uplink_snr_db = 11 + (int) (cosf(stub->phase * 0.7f) * 2.0f);
    state->link_statistics.active_antenna = ((int) (stub->phase * 3.0f)) & 0x01;
    state->link_statistics.rf_mode = 3;
    state->link_statistics.uplink_tx_power = 3;
    state->link_statistics.downlink_rssi_dbm_neg = 65 + (int) (sinf(stub->phase * 0.8f) * 3.0f);
    state->link_statistics.downlink_lq_percent = 95 + (int) ((cosf(stub->phase * 0.6f) + 1.0f) * 2.0f);
    state->link_statistics.downlink_snr_db = 9 + (int) (sinf(stub->phase * 0.4f) * 2.0f);
    state->link_statistics.timestamp_ms = app_state_now_ms();
}

static void crsf_rx_task(void *arg)
{
    (void) arg;

    crsf_stub_context_t context = {
        .phase = 0.0f,
    };

    ESP_LOGI(TAG, "Starting CRSF RX stub task");

    while (true) {
        context.phase += 0.14f;
        app_state_write(update_crsf_state, &context);
        vTaskDelay(pdMS_TO_TICKS(APP_CRSF_STUB_REFRESH_MS));
    }
}

esp_err_t crsf_rx_start(void)
{
    BaseType_t result = xTaskCreate(crsf_rx_task, "task_crsf_rx", APP_TASK_STACK_MEDIUM, NULL, APP_TASK_PRIORITY_IO, NULL);
    return (result == pdPASS) ? ESP_OK : ESP_FAIL;
}
