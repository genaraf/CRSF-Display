#include "simulator/simulator.h"

#include <math.h>

#include "app_config.h"
#include "app_state.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "telemetry/telemetry_manager.h"

static const char *TAG = "simulator";
static const double DEG_TO_RAD = 0.017453292519943295;

static double deg_to_rad(double degrees)
{
    return degrees * DEG_TO_RAD;
}

static void advance_simulator_state(app_state_t *state, void *ctx)
{
    (void) ctx;

    const float speed_kmh = state->simulator.parameters.speed_kmh;
    const float heading_deg = state->simulator.parameters.heading_deg;
    const uint32_t interval_ms = state->simulator.parameters.update_interval_ms;

    state->simulator.next_emit_in_ms = (int32_t) interval_ms;

    if (state->simulator.state != SIM_RUNNING) {
        telemetry_manager_rebuild_locked(state);
        return;
    }

    const double heading_rad = deg_to_rad(heading_deg);
    const double distance_km = ((double) speed_kmh * (double) interval_ms) / 3600000.0;
    const double delta_north_km = distance_km * cos(heading_rad);
    const double delta_east_km = distance_km * sin(heading_rad);
    const double latitude_rad = deg_to_rad(state->simulator.current_gps.latitude);

    state->simulator.current_gps.latitude += delta_north_km / 110.574;
    state->simulator.current_gps.longitude += delta_east_km / (111.320 * cos(latitude_rad));
    state->simulator.current_gps.gps_altitude_m = state->simulator.parameters.altitude_m;
    state->simulator.current_gps.ground_speed_kmh = speed_kmh;
    state->simulator.current_gps.heading_deg = heading_deg;
    state->simulator.current_gps.satellites = state->simulator.parameters.satellites;
    state->simulator.current_gps.fix_valid = state->simulator.parameters.fix_valid;
    state->simulator.current_gps.source = GPS_SOURCE_SIMULATOR;

    telemetry_manager_apply_simulator_gps_locked(state, &state->simulator.current_gps);
}

static void simulator_task(void *arg)
{
    (void) arg;

    ESP_LOGI(TAG, "Starting simulator task");

    while (true) {
        app_state_write(advance_simulator_state, NULL);
        vTaskDelay(pdMS_TO_TICKS(APP_SIMULATOR_REFRESH_MS));
    }
}

esp_err_t simulator_start(void)
{
    BaseType_t result = xTaskCreate(simulator_task, "task_simulator", APP_TASK_STACK_MEDIUM, NULL, APP_TASK_PRIORITY_NORMAL, NULL);
    return (result == pdPASS) ? ESP_OK : ESP_FAIL;
}
