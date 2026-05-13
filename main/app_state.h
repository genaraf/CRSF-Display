#ifndef APP_STATE_H
#define APP_STATE_H

#include <stdbool.h>
#include <stdint.h>

#include "app_events.h"
#include "esp_err.h"
#include "telemetry/telemetry_types.h"

typedef struct {
    crsf_link_state_t crsf_link;
    rx_channel_t rx_channels[APP_RX_CHANNEL_COUNT];
    link_statistics_t link_statistics;
    telemetry_tx_state_t telemetry_tx;
    gps_source_t gps_source;
    telemetry_data_t telemetry;
    telemetry_data_t manual_telemetry;
    simulator_state_t simulator;
    gps_status_t gps_status;
    sdcard_status_t sdcard;
    wifi_status_t wifi;
    ui_state_t ui;
    diagnostic_state_t diagnostic;
} app_state_t;

typedef void (*app_state_write_fn_t)(app_state_t *state, void *ctx);

void app_state_init(void);
void app_state_get_snapshot(app_state_t *snapshot);
void app_state_write(app_state_write_fn_t write_fn, void *ctx);
int64_t app_state_now_ms(void);

const char *app_state_gps_source_to_string(gps_source_t source);
const char *app_state_screen_id_to_string(ui_screen_id_t screen_id);
const char *app_state_telemetry_tx_status_to_string(telemetry_tx_status_t status);
const char *app_state_simulator_state_to_string(simulator_run_state_t state);
const char *app_state_json_operation_type_to_string(json_operation_type_t operation);
const char *app_state_json_operation_status_to_string(json_operation_status_code_t status);
const char *app_state_wifi_mode_to_string(app_wifi_mode_t mode);

bool app_state_parse_gps_source(const char *text, gps_source_t *source);
bool app_state_parse_simulator_command(const char *text, simulator_command_t *command);

#endif
