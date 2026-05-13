#include "gps/gps_input.h"

#include <math.h>
#include <stdio.h>

#include "app_config.h"
#include "app_state.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "telemetry/telemetry_manager.h"

static const char *TAG = "gps_input";

typedef struct {
    float phase;
} gps_stub_context_t;

static void update_external_gps_state(app_state_t *state, void *ctx)
{
    gps_stub_context_t *stub = (gps_stub_context_t *) ctx;

    gps_telemetry_t external_gps = {
        .latitude = 32.0853 + (cosf(stub->phase) * 0.00012),
        .longitude = 34.7818 + (sinf(stub->phase) * 0.00012),
        .gps_altitude_m = 118.4f + (sinf(stub->phase * 0.6f) * 0.8f),
        .ground_speed_kmh = 18.5f,
        .heading_deg = 182.0f,
        .satellites = 11,
        .fix_valid = true,
        .source = GPS_SOURCE_EXTERNAL_GPS,
    };

    state->gps_status.connected = true;
    state->gps_status.data_stream_present = true;
    state->gps_status.last_update_age_ms = 0;
    telemetry_manager_apply_external_gps_locked(state, &external_gps);
}

static void gps_input_task(void *arg)
{
    (void) arg;

    gps_stub_context_t context = {
        .phase = 0.0f,
    };

    ESP_LOGI(TAG, "Starting GPS input stub task");

    while (true) {
        context.phase += 0.08f;
        app_state_write(update_external_gps_state, &context);
        vTaskDelay(pdMS_TO_TICKS(APP_GPS_STUB_REFRESH_MS));
    }
}

esp_err_t gps_input_start(void)
{
    BaseType_t result = xTaskCreate(gps_input_task, "task_gps_rx", APP_TASK_STACK_MEDIUM, NULL, APP_TASK_PRIORITY_NORMAL, NULL);
    return (result == pdPASS) ? ESP_OK : ESP_FAIL;
}
