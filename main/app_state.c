#include "app_state.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>

#include "app_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "telemetry/telemetry_manager.h"

typedef struct {
    SemaphoreHandle_t mutex;
    app_state_t data;
} app_state_store_t;

static app_state_store_t s_state_store;

static void set_rx_channel_name(rx_channel_t *channel, const char *name)
{
    if (channel == NULL) {
        return;
    }

    snprintf(channel->name, sizeof(channel->name), "%s", name);
}

static void set_flight_mode(flight_mode_telemetry_t *flight_mode, const char *name)
{
    if (flight_mode == NULL) {
        return;
    }

    snprintf(flight_mode->mode_name, sizeof(flight_mode->mode_name), "%s", name);
}

static void app_state_set_default_channels(app_state_t *state)
{
    static const char *const channel_names[APP_RX_CHANNEL_COUNT] = {
        "Ail", "Elv", "Thr", "Roll",
        "Aux1", "Aux2", "Aux3", "Aux4",
        "Aux5", "Aux6", "Aux7", "Aux8",
        "Aux9", "Aux10", "Aux11", "Aux12",
    };
    static const float channel_values[APP_RX_CHANNEL_COUNT] = {
        38.0f, -12.0f, 71.0f, 5.0f,
        100.0f, -100.0f, 0.0f, 100.0f,
        -100.0f, 100.0f, -30.0f, 22.0f,
        0.0f, 48.0f, -100.0f, 100.0f,
    };

    for (int i = 0; i < APP_RX_CHANNEL_COUNT; ++i) {
        set_rx_channel_name(&state->rx_channels[i], channel_names[i]);
        state->rx_channels[i].normalized_value = channel_values[i];
    }
}

static void app_state_set_default_telemetry(telemetry_data_t *telemetry, gps_source_t source)
{
    telemetry->gps.latitude = 32.0853;
    telemetry->gps.longitude = 34.7818;
    telemetry->gps.gps_altitude_m = 118.0f;
    telemetry->gps.ground_speed_kmh = 41.4f;
    telemetry->gps.heading_deg = 178.0f;
    telemetry->gps.satellites = 12;
    telemetry->gps.fix_valid = true;
    telemetry->gps.source = source;

    telemetry->attitude.pitch_deg = 4.2f;
    telemetry->attitude.roll_deg = -1.3f;
    telemetry->attitude.yaw_deg = 178.0f;

    telemetry->barometric_altitude.baro_altitude_m = 117.6f;

    telemetry->battery.voltage_v = 15.8f;
    telemetry->battery.current_a = 12.4f;

    set_flight_mode(&telemetry->flight_mode, "Angle Mode");
    telemetry->timestamp_ms = app_state_now_ms();
}

static void app_state_set_defaults(app_state_t *state)
{
    memset(state, 0, sizeof(*state));

    state->crsf_link.link_up = true;
    state->crsf_link.rx_active = true;
    state->crsf_link.frame_error = false;
    state->crsf_link.last_frame_age_ms = 0;
    snprintf(state->crsf_link.uart_port_name, sizeof(state->crsf_link.uart_port_name), "%s", APP_CRSF_UART_PORT_NAME);

    app_state_set_default_channels(state);

    state->link_statistics.uplink_rssi_ant1_dbm_neg = 58;
    state->link_statistics.uplink_rssi_ant2_dbm_neg = 61;
    state->link_statistics.uplink_lq_percent = 99;
    state->link_statistics.uplink_snr_db = 12;
    state->link_statistics.active_antenna = 0;
    state->link_statistics.rf_mode = 3;
    state->link_statistics.uplink_tx_power = 3;
    state->link_statistics.downlink_rssi_dbm_neg = 65;
    state->link_statistics.downlink_lq_percent = 96;
    state->link_statistics.downlink_snr_db = 9;
    state->link_statistics.timestamp_ms = app_state_now_ms();

    state->telemetry_tx.status = TELEMETRY_TX_OK;
    state->telemetry_tx.last_tx_age_ms = 0;

    state->gps_source = GPS_SOURCE_SIMULATOR;

    app_state_set_default_telemetry(&state->manual_telemetry, GPS_SOURCE_MANUAL);
    app_state_set_default_telemetry(&state->telemetry, GPS_SOURCE_SIMULATOR);

    state->simulator.state = SIM_RUNNING;
    state->simulator.parameters.version = 1;
    state->simulator.parameters.start_latitude = 32.0853;
    state->simulator.parameters.start_longitude = 34.7818;
    state->simulator.parameters.altitude_m = 118.0f;
    state->simulator.parameters.speed_kmh = 41.4f;
    state->simulator.parameters.heading_deg = 178.0f;
    state->simulator.parameters.update_interval_ms = APP_SIMULATOR_REFRESH_MS;
    state->simulator.parameters.satellites = 12;
    state->simulator.parameters.fix_valid = true;
    state->simulator.current_gps = state->telemetry.gps;
    snprintf(state->simulator.active_profile_name, sizeof(state->simulator.active_profile_name), "%s", APP_SIMULATOR_DEFAULT_PROFILE_NAME);
    snprintf(state->simulator.active_profile_path, sizeof(state->simulator.active_profile_path), "%s", APP_SIMULATOR_DEFAULT_PROFILE_PATH);
    state->simulator.next_emit_in_ms = APP_SIMULATOR_REFRESH_MS;
    state->simulator.last_json_operation.operation = JSON_OP_NONE;
    state->simulator.last_json_operation.status = JSON_OP_STATUS_IDLE;
    state->simulator.last_json_operation.timestamp_ms = app_state_now_ms();

    state->gps_status.connected = true;
    state->gps_status.data_stream_present = true;
    snprintf(state->gps_status.protocol_name, sizeof(state->gps_status.protocol_name), "%s", APP_GPS_PROTOCOL_NAME);
    snprintf(state->gps_status.serial_type_name, sizeof(state->gps_status.serial_type_name), "%s", APP_GPS_SERIAL_TYPE_NAME);
    state->gps_status.rx_gpio = APP_GPS_RX_GPIO;
    state->gps_status.tx_gpio = APP_GPS_TX_GPIO;
    state->gps_status.baud_rate = APP_GPS_BAUD_RATE;
    state->gps_status.last_fix = state->manual_telemetry.gps;
    state->gps_status.last_update_age_ms = 0;
    state->gps_status.parse_gga = true;
    state->gps_status.parse_rmc = true;
    state->gps_status.parse_gsa = true;
    state->gps_status.parse_gsv = true;

    state->sdcard.mounted = false;
    snprintf(state->sdcard.profiles_directory, sizeof(state->sdcard.profiles_directory), "%s", APP_SIMULATOR_DEFAULT_PROFILE_DIR);
    state->sdcard.free_bytes = -1;

    state->wifi.ready = false;
    state->wifi.mode = APP_WIFI_MODE_SOFTAP;
    snprintf(state->wifi.ssid, sizeof(state->wifi.ssid), "%s", APP_WIFI_SOFTAP_SSID);
    snprintf(state->wifi.ip_address, sizeof(state->wifi.ip_address), "%s", "0.0.0.0");
    snprintf(state->wifi.hostname, sizeof(state->wifi.hostname), "%s", "crsf-display");
    state->wifi.connected_clients = 0;
    state->wifi.web_ui_ready = false;

    state->ui.screen_id = SCREEN_CRSF_CHANNELS;
    snprintf(state->ui.selected_item, sizeof(state->ui.selected_item), "%s", "Ail");
    state->ui.mode = UI_MODE_VIEW;
    state->ui.gps_source = state->gps_source;

    snprintf(state->diagnostic.code, sizeof(state->diagnostic.code), "%s", "init");
    snprintf(state->diagnostic.message, sizeof(state->diagnostic.message), "%s", "System bootstrapped with stub data");

    telemetry_manager_rebuild_locked(state);
}

void app_state_init(void)
{
    if (s_state_store.mutex == NULL) {
        s_state_store.mutex = xSemaphoreCreateMutex();
    }

    app_state_set_defaults(&s_state_store.data);
}

void app_state_get_snapshot(app_state_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }

    xSemaphoreTake(s_state_store.mutex, portMAX_DELAY);
    memcpy(snapshot, &s_state_store.data, sizeof(*snapshot));
    xSemaphoreGive(s_state_store.mutex);
}

void app_state_write(app_state_write_fn_t write_fn, void *ctx)
{
    if (write_fn == NULL) {
        return;
    }

    xSemaphoreTake(s_state_store.mutex, portMAX_DELAY);
    write_fn(&s_state_store.data, ctx);
    xSemaphoreGive(s_state_store.mutex);
}

int64_t app_state_now_ms(void)
{
    struct timeval now = {0};
    gettimeofday(&now, NULL);
    return ((int64_t) now.tv_sec * 1000LL) + (now.tv_usec / 1000LL);
}

const char *app_state_gps_source_to_string(gps_source_t source)
{
    switch (source) {
        case GPS_SOURCE_MANUAL:
            return "GPS_SOURCE_MANUAL";
        case GPS_SOURCE_EXTERNAL_GPS:
            return "GPS_SOURCE_EXTERNAL_GPS";
        case GPS_SOURCE_SIMULATOR:
            return "GPS_SOURCE_SIMULATOR";
        default:
            return "GPS_SOURCE_UNKNOWN";
    }
}

const char *app_state_screen_id_to_string(ui_screen_id_t screen_id)
{
    switch (screen_id) {
        case SCREEN_CRSF_CHANNELS:
            return "SCREEN_CRSF_CHANNELS";
        case SCREEN_LINK_STATISTICS:
            return "SCREEN_LINK_STATISTICS";
        case SCREEN_TELEMETRY_OVERVIEW:
            return "SCREEN_TELEMETRY_OVERVIEW";
        case SCREEN_MANUAL_TELEMETRY:
            return "SCREEN_MANUAL_TELEMETRY";
        case SCREEN_SIMULATION_PARAMETERS:
            return "SCREEN_SIMULATION_PARAMETERS";
        case SCREEN_GPS_STATUS:
            return "SCREEN_GPS_STATUS";
        default:
            return "SCREEN_UNKNOWN";
    }
}

const char *app_state_telemetry_tx_status_to_string(telemetry_tx_status_t status)
{
    switch (status) {
        case TELEMETRY_TX_IDLE:
            return "idle";
        case TELEMETRY_TX_OK:
            return "ok";
        case TELEMETRY_TX_ERROR:
            return "error";
        default:
            return "unknown";
    }
}

const char *app_state_simulator_state_to_string(simulator_run_state_t state)
{
    switch (state) {
        case SIM_STOPPED:
            return "SIM_STOPPED";
        case SIM_RUNNING:
            return "SIM_RUNNING";
        case SIM_PAUSED:
            return "SIM_PAUSED";
        default:
            return "SIM_UNKNOWN";
    }
}

const char *app_state_json_operation_type_to_string(json_operation_type_t operation)
{
    switch (operation) {
        case JSON_OP_NONE:
            return "none";
        case JSON_OP_LOAD_PROFILE:
            return "load_profile";
        case JSON_OP_SAVE_PROFILE:
            return "save_profile";
        default:
            return "unknown";
    }
}

const char *app_state_json_operation_status_to_string(json_operation_status_code_t status)
{
    switch (status) {
        case JSON_OP_STATUS_IDLE:
            return "idle";
        case JSON_OP_STATUS_QUEUED:
            return "queued";
        case JSON_OP_STATUS_IN_PROGRESS:
            return "in_progress";
        case JSON_OP_STATUS_SUCCESS:
            return "success";
        case JSON_OP_STATUS_ERROR:
            return "error";
        default:
            return "unknown";
    }
}

const char *app_state_wifi_mode_to_string(app_wifi_mode_t mode)
{
    switch (mode) {
        case APP_WIFI_MODE_SOFTAP:
            return "softap";
        case APP_WIFI_MODE_STA:
            return "sta";
        case APP_WIFI_MODE_AP_STA:
            return "ap_sta";
        default:
            return "unknown";
    }
}

bool app_state_parse_gps_source(const char *text, gps_source_t *source)
{
    if ((text == NULL) || (source == NULL)) {
        return false;
    }

    if (strcmp(text, "GPS_SOURCE_MANUAL") == 0) {
        *source = GPS_SOURCE_MANUAL;
        return true;
    }

    if (strcmp(text, "GPS_SOURCE_EXTERNAL_GPS") == 0) {
        *source = GPS_SOURCE_EXTERNAL_GPS;
        return true;
    }

    if (strcmp(text, "GPS_SOURCE_SIMULATOR") == 0) {
        *source = GPS_SOURCE_SIMULATOR;
        return true;
    }

    return false;
}

bool app_state_parse_simulator_command(const char *text, simulator_command_t *command)
{
    if ((text == NULL) || (command == NULL)) {
        return false;
    }

    if (strcmp(text, "Start") == 0) {
        *command = SIMULATOR_COMMAND_START;
        return true;
    }

    if (strcmp(text, "Pause") == 0) {
        *command = SIMULATOR_COMMAND_PAUSE;
        return true;
    }

    if (strcmp(text, "Resume") == 0) {
        *command = SIMULATOR_COMMAND_RESUME;
        return true;
    }

    if (strcmp(text, "Stop") == 0) {
        *command = SIMULATOR_COMMAND_STOP;
        return true;
    }

    if (strcmp(text, "Reset") == 0) {
        *command = SIMULATOR_COMMAND_RESET;
        return true;
    }

    return false;
}
