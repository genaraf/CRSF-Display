#include "web/web_ui.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app_config.h"
#include "app_state.h"
#include "cJSON.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdcard/sdcard_json.h"
#include "storage/storage.h"
#include "telemetry/telemetry_manager.h"
#include "wifi/wifi_manager.h"

static const char *TAG = "web_ui";
static httpd_handle_t s_http_server;

typedef struct {
    bool gps_latitude_present;
    bool gps_longitude_present;
    bool gps_altitude_present;
    bool gps_speed_present;
    bool gps_heading_present;
    bool gps_satellites_present;
    bool gps_fix_valid_present;
    double gps_latitude;
    double gps_longitude;
    float gps_altitude_m;
    float gps_speed_kmh;
    float gps_heading_deg;
    int gps_satellites;
    bool gps_fix_valid;
    bool attitude_pitch_present;
    bool attitude_roll_present;
    bool attitude_yaw_present;
    float attitude_pitch_deg;
    float attitude_roll_deg;
    float attitude_yaw_deg;
    bool baro_altitude_present;
    float baro_altitude_m;
    bool battery_voltage_present;
    bool battery_current_present;
    float battery_voltage_v;
    float battery_current_a;
    bool flight_mode_present;
    char flight_mode[APP_MAX_FLIGHT_MODE_LEN];
} manual_telemetry_patch_t;

typedef struct {
    bool version_present;
    bool start_latitude_present;
    bool start_longitude_present;
    bool altitude_present;
    bool speed_present;
    bool heading_present;
    bool update_interval_present;
    bool satellites_present;
    bool fix_valid_present;
    int version;
    double start_latitude;
    double start_longitude;
    float altitude_m;
    float speed_kmh;
    float heading_deg;
    int update_interval_ms;
    int satellites;
    bool fix_valid;
} simulator_parameters_patch_t;

typedef struct {
    bool screen_id_present;
    ui_screen_id_t screen_id;
    bool selected_item_present;
    char selected_item[APP_MAX_SELECTED_ITEM_LEN];
    bool edit_mode_present;
    bool edit_mode;
    bool gps_source_present;
    gps_source_t gps_source;
    bool dialog_state_present;
    char dialog_state[APP_MAX_DIALOG_STATE_LEN];
} ui_state_patch_t;

static bool is_valid_flight_mode(const char *mode_name)
{
    return (mode_name != NULL) &&
           ((strcmp(mode_name, "Angle Mode") == 0) ||
            (strcmp(mode_name, "Horizon Mode") == 0) ||
            (strcmp(mode_name, "Acro Mode") == 0));
}

static cJSON *json_create_crsf_link(const crsf_link_state_t *link)
{
    cJSON *json = cJSON_CreateObject();
    cJSON_AddBoolToObject(json, "link_up", link->link_up);
    cJSON_AddBoolToObject(json, "rx_active", link->rx_active);
    cJSON_AddBoolToObject(json, "frame_error", link->frame_error);
    cJSON_AddNumberToObject(json, "last_frame_age_ms", link->last_frame_age_ms);
    cJSON_AddStringToObject(json, "uart_port", link->uart_port_name);
    return json;
}

static cJSON *json_create_telemetry_tx(const telemetry_tx_state_t *tx)
{
    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "status", app_state_telemetry_tx_status_to_string(tx->status));
    cJSON_AddNumberToObject(json, "last_tx_age_ms", tx->last_tx_age_ms);
    cJSON_AddStringToObject(json, "last_error", tx->last_error);
    return json;
}

static cJSON *json_create_channels(const app_state_t *state)
{
    cJSON *root = cJSON_CreateObject();
    cJSON *channels = cJSON_CreateArray();

    cJSON_AddNumberToObject(root, "timestamp_ms", app_state_now_ms());
    cJSON_AddItemToObject(root, "crsf_link", json_create_crsf_link(&state->crsf_link));
    cJSON_AddItemToObject(root, "telemetry_tx", json_create_telemetry_tx(&state->telemetry_tx));

    for (int i = 0; i < APP_RX_CHANNEL_COUNT; ++i) {
        cJSON *item = cJSON_CreateObject();
        cJSON_AddStringToObject(item, "name", state->rx_channels[i].name);
        cJSON_AddNumberToObject(item, "normalized_value", state->rx_channels[i].normalized_value);
        cJSON_AddItemToArray(channels, item);
    }

    cJSON_AddItemToObject(root, "channels", channels);
    return root;
}

static cJSON *json_create_link_statistics_payload(const link_statistics_t *statistics)
{
    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "uplink_rssi_ant1_dbm_neg", statistics->uplink_rssi_ant1_dbm_neg);
    cJSON_AddNumberToObject(json, "uplink_rssi_ant2_dbm_neg", statistics->uplink_rssi_ant2_dbm_neg);
    cJSON_AddNumberToObject(json, "uplink_lq_percent", statistics->uplink_lq_percent);
    cJSON_AddNumberToObject(json, "uplink_snr_db", statistics->uplink_snr_db);
    cJSON_AddNumberToObject(json, "active_antenna", statistics->active_antenna);
    cJSON_AddNumberToObject(json, "rf_mode", statistics->rf_mode);
    cJSON_AddNumberToObject(json, "uplink_tx_power", statistics->uplink_tx_power);
    cJSON_AddNumberToObject(json, "downlink_rssi_dbm_neg", statistics->downlink_rssi_dbm_neg);
    cJSON_AddNumberToObject(json, "downlink_lq_percent", statistics->downlink_lq_percent);
    cJSON_AddNumberToObject(json, "downlink_snr_db", statistics->downlink_snr_db);
    cJSON_AddNumberToObject(json, "timestamp_ms", statistics->timestamp_ms);
    return json;
}

static cJSON *json_create_link_statistics(const app_state_t *state)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "timestamp_ms", app_state_now_ms());
    cJSON_AddItemToObject(root, "statistics", json_create_link_statistics_payload(&state->link_statistics));
    return root;
}

static cJSON *json_create_gps_telemetry(const gps_telemetry_t *gps)
{
    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "latitude", gps->latitude);
    cJSON_AddNumberToObject(json, "longitude", gps->longitude);
    cJSON_AddNumberToObject(json, "gps_altitude_m", gps->gps_altitude_m);
    cJSON_AddNumberToObject(json, "ground_speed_kmh", gps->ground_speed_kmh);
    cJSON_AddNumberToObject(json, "heading_deg", gps->heading_deg);
    cJSON_AddNumberToObject(json, "satellites", gps->satellites);
    cJSON_AddBoolToObject(json, "fix_valid", gps->fix_valid);
    cJSON_AddStringToObject(json, "source", app_state_gps_source_to_string(gps->source));
    return json;
}

static cJSON *json_create_attitude(const attitude_telemetry_t *attitude)
{
    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "pitch_deg", attitude->pitch_deg);
    cJSON_AddNumberToObject(json, "roll_deg", attitude->roll_deg);
    cJSON_AddNumberToObject(json, "yaw_deg", attitude->yaw_deg);
    return json;
}

static cJSON *json_create_baro(const barometric_altitude_telemetry_t *baro)
{
    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "baro_altitude_m", baro->baro_altitude_m);
    return json;
}

static cJSON *json_create_battery(const battery_telemetry_t *battery)
{
    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "voltage_v", battery->voltage_v);
    cJSON_AddNumberToObject(json, "current_a", battery->current_a);
    return json;
}

static cJSON *json_create_flight_mode(const flight_mode_telemetry_t *flight_mode)
{
    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "mode_name", flight_mode->mode_name);
    return json;
}

static cJSON *json_create_telemetry_overview(const app_state_t *state)
{
    cJSON *root = cJSON_CreateObject();

    cJSON_AddNumberToObject(root, "timestamp_ms", state->telemetry.timestamp_ms);
    cJSON_AddStringToObject(root, "gps_source", app_state_gps_source_to_string(state->gps_source));
    cJSON_AddItemToObject(root, "gps", json_create_gps_telemetry(&state->telemetry.gps));
    cJSON_AddItemToObject(root, "attitude", json_create_attitude(&state->telemetry.attitude));
    cJSON_AddItemToObject(root, "barometric_altitude", json_create_baro(&state->telemetry.barometric_altitude));
    cJSON_AddItemToObject(root, "battery", json_create_battery(&state->telemetry.battery));
    cJSON_AddItemToObject(root, "flight_mode", json_create_flight_mode(&state->telemetry.flight_mode));
    cJSON_AddItemToObject(root, "telemetry_tx", json_create_telemetry_tx(&state->telemetry_tx));
    return root;
}

static cJSON *json_create_manual_telemetry(const app_state_t *state)
{
    cJSON *root = cJSON_CreateObject();
    cJSON *values = cJSON_CreateObject();
    cJSON *editable_fields = cJSON_CreateArray();
    cJSON *flight_mode_options = cJSON_CreateArray();

    cJSON_AddNumberToObject(root, "timestamp_ms", state->manual_telemetry.timestamp_ms);

    cJSON_AddItemToObject(values, "gps", json_create_gps_telemetry(&state->manual_telemetry.gps));
    cJSON_AddItemToObject(values, "attitude", json_create_attitude(&state->manual_telemetry.attitude));
    cJSON_AddItemToObject(values, "barometric_altitude", json_create_baro(&state->manual_telemetry.barometric_altitude));
    cJSON_AddItemToObject(values, "battery", json_create_battery(&state->manual_telemetry.battery));
    cJSON_AddItemToObject(values, "flight_mode", json_create_flight_mode(&state->manual_telemetry.flight_mode));
    cJSON_AddItemToObject(root, "values", values);

    cJSON_AddItemToArray(editable_fields, cJSON_CreateString("gps.latitude"));
    cJSON_AddItemToArray(editable_fields, cJSON_CreateString("gps.longitude"));
    cJSON_AddItemToArray(editable_fields, cJSON_CreateString("gps.gps_altitude_m"));
    cJSON_AddItemToArray(editable_fields, cJSON_CreateString("gps.ground_speed_kmh"));
    cJSON_AddItemToArray(editable_fields, cJSON_CreateString("gps.heading_deg"));
    cJSON_AddItemToArray(editable_fields, cJSON_CreateString("gps.satellites"));
    cJSON_AddItemToArray(editable_fields, cJSON_CreateString("gps.fix_valid"));
    cJSON_AddItemToArray(editable_fields, cJSON_CreateString("attitude.pitch_deg"));
    cJSON_AddItemToArray(editable_fields, cJSON_CreateString("attitude.roll_deg"));
    cJSON_AddItemToArray(editable_fields, cJSON_CreateString("attitude.yaw_deg"));
    cJSON_AddItemToArray(editable_fields, cJSON_CreateString("barometric_altitude.baro_altitude_m"));
    cJSON_AddItemToArray(editable_fields, cJSON_CreateString("battery.voltage_v"));
    cJSON_AddItemToArray(editable_fields, cJSON_CreateString("battery.current_a"));
    cJSON_AddItemToArray(editable_fields, cJSON_CreateString("flight_mode.mode_name"));
    cJSON_AddItemToObject(root, "editable_fields", editable_fields);

    cJSON_AddItemToArray(flight_mode_options, cJSON_CreateString("Angle Mode"));
    cJSON_AddItemToArray(flight_mode_options, cJSON_CreateString("Horizon Mode"));
    cJSON_AddItemToArray(flight_mode_options, cJSON_CreateString("Acro Mode"));
    cJSON_AddItemToObject(root, "flight_mode_options", flight_mode_options);
    return root;
}

static cJSON *json_create_ui_state(const ui_state_t *ui)
{
    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "screen_id", app_state_screen_id_to_string(ui->screen_id));
    cJSON_AddStringToObject(json, "selected_item", ui->selected_item);
    cJSON_AddBoolToObject(json, "edit_mode", ui->mode == UI_MODE_EDIT);
    cJSON_AddStringToObject(json, "gps_source", app_state_gps_source_to_string(ui->gps_source));
    cJSON_AddStringToObject(json, "dialog_state", ui->dialog_state);
    return json;
}

static cJSON *json_create_operation_accepted(const char *operation, const char *profile_path, const char *message)
{
    cJSON *json = cJSON_CreateObject();
    cJSON_AddBoolToObject(json, "accepted", true);
    cJSON_AddStringToObject(json, "operation", operation);
    cJSON_AddNumberToObject(json, "timestamp_ms", app_state_now_ms());
    cJSON_AddStringToObject(json, "profile_path", (profile_path != NULL) ? profile_path : "");
    cJSON_AddStringToObject(json, "message", (message != NULL) ? message : "");
    return json;
}

static cJSON *json_create_profiles_list_response(const sdcard_json_profile_list_t *profiles, const char *selected_profile_path)
{
    cJSON *root = cJSON_CreateObject();
    cJSON *items = cJSON_CreateArray();

    cJSON_AddStringToObject(root, "profiles_directory", APP_SIMULATOR_DEFAULT_PROFILE_DIR);
    cJSON_AddStringToObject(root, "selected_profile_path", (selected_profile_path != NULL) ? selected_profile_path : "");

    for (int i = 0; i < profiles->count; ++i) {
        cJSON *item = cJSON_CreateObject();
        cJSON_AddStringToObject(item, "name", profiles->items[i].name);
        cJSON_AddStringToObject(item, "path", profiles->items[i].path);
        cJSON_AddNumberToObject(item, "size_bytes", profiles->items[i].size_bytes);
        cJSON_AddNumberToObject(item, "modified_at_ms", profiles->items[i].modified_at_ms);
        cJSON_AddItemToArray(items, item);
    }

    cJSON_AddItemToObject(root, "profiles", items);
    return root;
}

static cJSON *json_create_sdcard_status(const sdcard_status_t *sdcard)
{
    cJSON *json = cJSON_CreateObject();
    cJSON_AddBoolToObject(json, "mounted", sdcard->mounted);
    cJSON_AddStringToObject(json, "profiles_directory", sdcard->profiles_directory);
    cJSON_AddNumberToObject(json, "free_bytes", sdcard->free_bytes);
    cJSON_AddStringToObject(json, "last_error", sdcard->last_error);
    return json;
}

static cJSON *json_create_wifi_status(const wifi_status_t *wifi)
{
    cJSON *json = cJSON_CreateObject();
    cJSON_AddBoolToObject(json, "ready", wifi->ready);
    cJSON_AddStringToObject(json, "mode", app_state_wifi_mode_to_string(wifi->mode));
    cJSON_AddStringToObject(json, "ssid", wifi->ssid);
    cJSON_AddStringToObject(json, "ip_address", wifi->ip_address);
    cJSON_AddStringToObject(json, "hostname", wifi->hostname);
    cJSON_AddNumberToObject(json, "connected_clients", wifi->connected_clients);
    cJSON_AddBoolToObject(json, "web_ui_ready", wifi->web_ui_ready);
    return json;
}

static cJSON *json_create_json_operation_status(const json_operation_status_t *operation)
{
    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "operation", app_state_json_operation_type_to_string(operation->operation));
    cJSON_AddStringToObject(json, "status", app_state_json_operation_status_to_string(operation->status));
    cJSON_AddNumberToObject(json, "timestamp_ms", operation->timestamp_ms);
    cJSON_AddStringToObject(json, "profile_path", operation->profile_path);
    cJSON_AddStringToObject(json, "message", operation->message);
    return json;
}

static cJSON *json_create_simulator(const app_state_t *state)
{
    cJSON *root = cJSON_CreateObject();
    cJSON *params = cJSON_CreateObject();

    cJSON_AddNumberToObject(root, "timestamp_ms", app_state_now_ms());
    cJSON_AddStringToObject(root, "state", app_state_simulator_state_to_string(state->simulator.state));
    cJSON_AddNumberToObject(params, "version", state->simulator.parameters.version);
    cJSON_AddNumberToObject(params, "start_latitude", state->simulator.parameters.start_latitude);
    cJSON_AddNumberToObject(params, "start_longitude", state->simulator.parameters.start_longitude);
    cJSON_AddNumberToObject(params, "altitude_m", state->simulator.parameters.altitude_m);
    cJSON_AddNumberToObject(params, "speed_kmh", state->simulator.parameters.speed_kmh);
    cJSON_AddNumberToObject(params, "heading_deg", state->simulator.parameters.heading_deg);
    cJSON_AddNumberToObject(params, "update_interval_ms", state->simulator.parameters.update_interval_ms);
    cJSON_AddNumberToObject(params, "satellites", state->simulator.parameters.satellites);
    cJSON_AddBoolToObject(params, "fix_valid", state->simulator.parameters.fix_valid);
    cJSON_AddItemToObject(root, "parameters", params);
    cJSON_AddItemToObject(root, "current_gps", json_create_gps_telemetry(&state->simulator.current_gps));
    cJSON_AddStringToObject(root, "active_profile_name", state->simulator.active_profile_name);
    cJSON_AddStringToObject(root, "active_profile_path", state->simulator.active_profile_path);
    cJSON_AddNumberToObject(root, "next_emit_in_ms", state->simulator.next_emit_in_ms);
    cJSON_AddItemToObject(root, "sdcard", json_create_sdcard_status(&state->sdcard));
    cJSON_AddItemToObject(root, "last_json_operation", json_create_json_operation_status(&state->simulator.last_json_operation));
    return root;
}

static cJSON *json_create_gps_status(const app_state_t *state)
{
    cJSON *root = cJSON_CreateObject();
    cJSON *serial = cJSON_CreateObject();
    cJSON *last_fix = cJSON_CreateObject();
    cJSON *messages = cJSON_CreateArray();

    cJSON_AddNumberToObject(root, "timestamp_ms", app_state_now_ms());
    cJSON_AddBoolToObject(root, "connected", state->gps_status.connected);
    cJSON_AddBoolToObject(root, "data_stream_present", state->gps_status.data_stream_present);
    cJSON_AddStringToObject(root, "protocol", "NMEA_0183");

    cJSON_AddStringToObject(serial, "type", state->gps_status.serial_type_name);
    cJSON_AddNumberToObject(serial, "rx_gpio", state->gps_status.rx_gpio);
    cJSON_AddNumberToObject(serial, "tx_gpio", state->gps_status.tx_gpio);
    cJSON_AddNumberToObject(serial, "baud_rate", state->gps_status.baud_rate);
    cJSON_AddItemToObject(root, "serial", serial);

    cJSON_AddNumberToObject(last_fix, "latitude", state->gps_status.last_fix.latitude);
    cJSON_AddNumberToObject(last_fix, "longitude", state->gps_status.last_fix.longitude);
    cJSON_AddNumberToObject(last_fix, "speed_kmh", state->gps_status.last_fix.ground_speed_kmh);
    cJSON_AddNumberToObject(last_fix, "course_deg", state->gps_status.last_fix.heading_deg);
    cJSON_AddNumberToObject(last_fix, "altitude_m", state->gps_status.last_fix.gps_altitude_m);
    cJSON_AddNumberToObject(last_fix, "satellites", state->gps_status.last_fix.satellites);
    cJSON_AddBoolToObject(last_fix, "fix_valid", state->gps_status.last_fix.fix_valid);
    cJSON_AddNumberToObject(last_fix, "last_update_age_ms", state->gps_status.last_update_age_ms);
    cJSON_AddItemToObject(root, "last_fix", last_fix);

    if (state->gps_status.parse_gga) {
        cJSON_AddItemToArray(messages, cJSON_CreateString("GGA"));
    }
    if (state->gps_status.parse_rmc) {
        cJSON_AddItemToArray(messages, cJSON_CreateString("RMC"));
    }
    if (state->gps_status.parse_gsa) {
        cJSON_AddItemToArray(messages, cJSON_CreateString("GSA"));
    }
    if (state->gps_status.parse_gsv) {
        cJSON_AddItemToArray(messages, cJSON_CreateString("GSV"));
    }

    cJSON_AddItemToObject(root, "parsed_messages", messages);
    cJSON_AddNumberToObject(root, "parser_errors", state->gps_status.parser_errors);
    return root;
}

static cJSON *json_create_system_state(const app_state_t *state)
{
    cJSON *root = cJSON_CreateObject();
    cJSON *diagnostics = cJSON_CreateArray();
    cJSON *diagnostic = cJSON_CreateObject();

    cJSON_AddNumberToObject(root, "device_time_ms", app_state_now_ms());
    cJSON_AddItemToObject(root, "crsf_link", json_create_crsf_link(&state->crsf_link));
    cJSON_AddItemToObject(root, "telemetry_tx", json_create_telemetry_tx(&state->telemetry_tx));
    cJSON_AddStringToObject(root, "gps_source", app_state_gps_source_to_string(state->gps_source));
    cJSON_AddItemToObject(root, "ui", json_create_ui_state(&state->ui));
    cJSON_AddItemToObject(root, "sdcard", json_create_sdcard_status(&state->sdcard));
    cJSON_AddItemToObject(root, "wifi", json_create_wifi_status(&state->wifi));
    cJSON_AddItemToObject(root, "channels", json_create_channels(state));
    cJSON_AddItemToObject(root, "link_statistics", json_create_link_statistics(state));
    cJSON_AddItemToObject(root, "telemetry", json_create_telemetry_overview(state));
    cJSON_AddItemToObject(root, "manual_telemetry", json_create_manual_telemetry(state));
    cJSON_AddItemToObject(root, "simulator", json_create_simulator(state));
    cJSON_AddItemToObject(root, "gps_status", json_create_gps_status(state));

    cJSON_AddStringToObject(diagnostic, "severity", "info");
    cJSON_AddStringToObject(diagnostic, "code", state->diagnostic.code);
    cJSON_AddStringToObject(diagnostic, "message", state->diagnostic.message);
    cJSON_AddNumberToObject(diagnostic, "timestamp_ms", app_state_now_ms());
    cJSON_AddItemToArray(diagnostics, diagnostic);
    cJSON_AddItemToObject(root, "diagnostics", diagnostics);
    return root;
}

static esp_err_t send_json(httpd_req_t *req, cJSON *json, int status_code)
{
    esp_err_t result = ESP_OK;
    const char *status_text = "200 OK";
    char *payload = NULL;

    if (status_code == 202) {
        status_text = "202 Accepted";
    } else if (status_code == 400) {
        status_text = "400 Bad Request";
    } else if (status_code == 404) {
        status_text = "404 Not Found";
    } else if (status_code == 409) {
        status_text = "409 Conflict";
    } else if (status_code == 503) {
        status_text = "503 Service Unavailable";
    }

    payload = cJSON_PrintUnformatted(json);
    if (payload == NULL) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "JSON serialization failed");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, PATCH, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
    httpd_resp_set_status(req, status_text);
    result = httpd_resp_sendstr(req, payload);

    free(payload);
    cJSON_Delete(json);
    return result;
}

static esp_err_t send_error(httpd_req_t *req, int status_code, const char *code, const char *message)
{
    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "code", code);
    cJSON_AddStringToObject(json, "message", message);
    return send_json(req, json, status_code);
}

static esp_err_t options_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, PATCH, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, NULL, 0);
}

static char *read_request_body(httpd_req_t *req)
{
    if (req->content_len <= 0) {
        return NULL;
    }

    char *buffer = calloc(1, (size_t) req->content_len + 1U);
    if (buffer == NULL) {
        return NULL;
    }

    int received = 0;
    while (received < req->content_len) {
        const int ret = httpd_req_recv(req, buffer + received, req->content_len - received);
        if (ret <= 0) {
            free(buffer);
            return NULL;
        }

        received += ret;
    }

    return buffer;
}

static esp_err_t get_health_handler(httpd_req_t *req)
{
    app_state_t state;
    cJSON *json = cJSON_CreateObject();

    app_state_get_snapshot(&state);
    cJSON_AddStringToObject(json, "status", "ok");
    cJSON_AddNumberToObject(json, "device_time_ms", app_state_now_ms());
    cJSON_AddBoolToObject(json, "web_ui_ready", state.wifi.web_ui_ready);
    cJSON_AddBoolToObject(json, "wifi_ready", state.wifi.ready);
    cJSON_AddStringToObject(json, "version", "0.1.0");
    return send_json(req, json, 200);
}

static esp_err_t get_system_state_handler(httpd_req_t *req)
{
    app_state_t state;
    app_state_get_snapshot(&state);
    return send_json(req, json_create_system_state(&state), 200);
}

static esp_err_t get_ui_state_handler(httpd_req_t *req)
{
    app_state_t state;
    app_state_get_snapshot(&state);
    return send_json(req, json_create_ui_state(&state.ui), 200);
}

static esp_err_t get_channels_handler(httpd_req_t *req)
{
    app_state_t state;
    app_state_get_snapshot(&state);
    return send_json(req, json_create_channels(&state), 200);
}

static esp_err_t get_link_statistics_handler(httpd_req_t *req)
{
    app_state_t state;
    app_state_get_snapshot(&state);
    return send_json(req, json_create_link_statistics(&state), 200);
}

static esp_err_t get_telemetry_overview_handler(httpd_req_t *req)
{
    app_state_t state;
    app_state_get_snapshot(&state);
    return send_json(req, json_create_telemetry_overview(&state), 200);
}

static esp_err_t get_manual_telemetry_handler(httpd_req_t *req)
{
    app_state_t state;
    app_state_get_snapshot(&state);
    return send_json(req, json_create_manual_telemetry(&state), 200);
}

static void apply_ui_state_patch(app_state_t *state, void *ctx)
{
    ui_state_patch_t *patch = (ui_state_patch_t *) ctx;

    if (patch->screen_id_present) {
        state->ui.screen_id = patch->screen_id;
    }
    if (patch->selected_item_present) {
        snprintf(state->ui.selected_item, sizeof(state->ui.selected_item), "%s", patch->selected_item);
    }
    if (patch->edit_mode_present) {
        state->ui.mode = patch->edit_mode ? UI_MODE_EDIT : UI_MODE_VIEW;
    }
    if (patch->dialog_state_present) {
        snprintf(state->ui.dialog_state, sizeof(state->ui.dialog_state), "%s", patch->dialog_state);
    }
    if (patch->gps_source_present) {
        telemetry_manager_set_gps_source_locked(state, patch->gps_source);
    } else {
        state->ui.gps_source = state->gps_source;
    }
}

static esp_err_t patch_ui_state_handler(httpd_req_t *req)
{
    char *body = read_request_body(req);
    if (body == NULL) {
        return send_error(req, 400, "bad_request", "Missing request body");
    }

    cJSON *json = cJSON_Parse(body);
    free(body);
    if (json == NULL) {
        return send_error(req, 400, "bad_request", "Invalid JSON body");
    }

    ui_state_patch_t patch = {0};
    cJSON *screen_id = cJSON_GetObjectItemCaseSensitive(json, "screen_id");
    cJSON *selected_item = cJSON_GetObjectItemCaseSensitive(json, "selected_item");
    cJSON *edit_mode = cJSON_GetObjectItemCaseSensitive(json, "edit_mode");
    cJSON *gps_source = cJSON_GetObjectItemCaseSensitive(json, "gps_source");
    cJSON *dialog_state = cJSON_GetObjectItemCaseSensitive(json, "dialog_state");

    if (screen_id != NULL) {
        if (!cJSON_IsString(screen_id) || !app_state_parse_screen_id(screen_id->valuestring, &patch.screen_id)) {
            cJSON_Delete(json);
            return send_error(req, 400, "bad_request", "Field screen_id is invalid");
        }
        patch.screen_id_present = true;
    }

    if (selected_item != NULL) {
        if (!cJSON_IsString(selected_item)) {
            cJSON_Delete(json);
            return send_error(req, 400, "bad_request", "Field selected_item must be a string");
        }
        patch.selected_item_present = true;
        snprintf(patch.selected_item, sizeof(patch.selected_item), "%s", selected_item->valuestring);
    }

    if (edit_mode != NULL) {
        if (!cJSON_IsBool(edit_mode)) {
            cJSON_Delete(json);
            return send_error(req, 400, "bad_request", "Field edit_mode must be boolean");
        }
        patch.edit_mode_present = true;
        patch.edit_mode = cJSON_IsTrue(edit_mode);
    }

    if (gps_source != NULL) {
        if (!cJSON_IsString(gps_source) || !app_state_parse_gps_source(gps_source->valuestring, &patch.gps_source)) {
            cJSON_Delete(json);
            return send_error(req, 400, "bad_request", "Field gps_source is invalid");
        }
        patch.gps_source_present = true;
    }

    if (dialog_state != NULL) {
        if (!cJSON_IsString(dialog_state)) {
            cJSON_Delete(json);
            return send_error(req, 400, "bad_request", "Field dialog_state must be a string");
        }
        patch.dialog_state_present = true;
        snprintf(patch.dialog_state, sizeof(patch.dialog_state), "%s", dialog_state->valuestring);
    }

    app_state_write(apply_ui_state_patch, &patch);
    if (patch.gps_source_present) {
        (void) storage_save_runtime_state();
    }
    cJSON_Delete(json);

    app_state_t state;
    app_state_get_snapshot(&state);
    return send_json(req, json_create_ui_state(&state.ui), 200);
}

static bool parse_float_field(cJSON *object, const char *name, bool *present, float *value)
{
    cJSON *item = cJSON_GetObjectItemCaseSensitive(object, name);
    if (item == NULL) {
        *present = false;
        return true;
    }

    if (!cJSON_IsNumber(item)) {
        return false;
    }

    *present = true;
    *value = (float) item->valuedouble;
    return true;
}

static bool parse_double_field(cJSON *object, const char *name, bool *present, double *value)
{
    cJSON *item = cJSON_GetObjectItemCaseSensitive(object, name);
    if (item == NULL) {
        *present = false;
        return true;
    }

    if (!cJSON_IsNumber(item)) {
        return false;
    }

    *present = true;
    *value = item->valuedouble;
    return true;
}

static bool parse_int_field(cJSON *object, const char *name, bool *present, int *value)
{
    cJSON *item = cJSON_GetObjectItemCaseSensitive(object, name);
    if (item == NULL) {
        *present = false;
        return true;
    }

    if (!cJSON_IsNumber(item)) {
        return false;
    }

    *present = true;
    *value = item->valueint;
    return true;
}

static bool parse_bool_field(cJSON *object, const char *name, bool *present, bool *value)
{
    cJSON *item = cJSON_GetObjectItemCaseSensitive(object, name);
    if (item == NULL) {
        *present = false;
        return true;
    }

    if (!cJSON_IsBool(item)) {
        return false;
    }

    *present = true;
    *value = cJSON_IsTrue(item);
    return true;
}

static void apply_manual_telemetry_patch(app_state_t *state, void *ctx)
{
    manual_telemetry_patch_t *patch = (manual_telemetry_patch_t *) ctx;

    if (patch->gps_latitude_present) {
        state->manual_telemetry.gps.latitude = patch->gps_latitude;
    }
    if (patch->gps_longitude_present) {
        state->manual_telemetry.gps.longitude = patch->gps_longitude;
    }
    if (patch->gps_altitude_present) {
        state->manual_telemetry.gps.gps_altitude_m = patch->gps_altitude_m;
    }
    if (patch->gps_speed_present) {
        state->manual_telemetry.gps.ground_speed_kmh = patch->gps_speed_kmh;
    }
    if (patch->gps_heading_present) {
        state->manual_telemetry.gps.heading_deg = patch->gps_heading_deg;
    }
    if (patch->gps_satellites_present) {
        state->manual_telemetry.gps.satellites = (uint8_t) patch->gps_satellites;
    }
    if (patch->gps_fix_valid_present) {
        state->manual_telemetry.gps.fix_valid = patch->gps_fix_valid;
    }

    if (patch->attitude_pitch_present) {
        state->manual_telemetry.attitude.pitch_deg = patch->attitude_pitch_deg;
    }
    if (patch->attitude_roll_present) {
        state->manual_telemetry.attitude.roll_deg = patch->attitude_roll_deg;
    }
    if (patch->attitude_yaw_present) {
        state->manual_telemetry.attitude.yaw_deg = patch->attitude_yaw_deg;
    }

    if (patch->baro_altitude_present) {
        state->manual_telemetry.barometric_altitude.baro_altitude_m = patch->baro_altitude_m;
    }

    if (patch->battery_voltage_present) {
        state->manual_telemetry.battery.voltage_v = patch->battery_voltage_v;
    }
    if (patch->battery_current_present) {
        state->manual_telemetry.battery.current_a = patch->battery_current_a;
    }

    if (patch->flight_mode_present) {
        snprintf(state->manual_telemetry.flight_mode.mode_name,
                 sizeof(state->manual_telemetry.flight_mode.mode_name),
                 "%s",
                 patch->flight_mode);
    }

    state->manual_telemetry.gps.source = GPS_SOURCE_MANUAL;
    state->manual_telemetry.timestamp_ms = app_state_now_ms();
    telemetry_manager_rebuild_locked(state);
}

static esp_err_t patch_manual_telemetry_handler(httpd_req_t *req)
{
    char *body = read_request_body(req);
    if (body == NULL) {
        return send_error(req, 400, "bad_request", "Missing request body");
    }

    cJSON *json = cJSON_Parse(body);
    free(body);
    if (json == NULL) {
        return send_error(req, 400, "bad_request", "Invalid JSON body");
    }

    manual_telemetry_patch_t patch = {0};
    cJSON *gps = cJSON_GetObjectItemCaseSensitive(json, "gps");
    cJSON *attitude = cJSON_GetObjectItemCaseSensitive(json, "attitude");
    cJSON *baro = cJSON_GetObjectItemCaseSensitive(json, "barometric_altitude");
    cJSON *battery = cJSON_GetObjectItemCaseSensitive(json, "battery");
    cJSON *flight_mode = cJSON_GetObjectItemCaseSensitive(json, "flight_mode");

    if ((gps != NULL) && !cJSON_IsObject(gps)) {
        cJSON_Delete(json);
        return send_error(req, 400, "bad_request", "Field gps must be an object");
    }
    if ((attitude != NULL) && !cJSON_IsObject(attitude)) {
        cJSON_Delete(json);
        return send_error(req, 400, "bad_request", "Field attitude must be an object");
    }
    if ((baro != NULL) && !cJSON_IsObject(baro)) {
        cJSON_Delete(json);
        return send_error(req, 400, "bad_request", "Field barometric_altitude must be an object");
    }
    if ((battery != NULL) && !cJSON_IsObject(battery)) {
        cJSON_Delete(json);
        return send_error(req, 400, "bad_request", "Field battery must be an object");
    }
    if ((flight_mode != NULL) && !cJSON_IsObject(flight_mode)) {
        cJSON_Delete(json);
        return send_error(req, 400, "bad_request", "Field flight_mode must be an object");
    }

    if ((gps != NULL) &&
        (!parse_double_field(gps, "latitude", &patch.gps_latitude_present, &patch.gps_latitude) ||
         !parse_double_field(gps, "longitude", &patch.gps_longitude_present, &patch.gps_longitude) ||
         !parse_float_field(gps, "gps_altitude_m", &patch.gps_altitude_present, &patch.gps_altitude_m) ||
         !parse_float_field(gps, "ground_speed_kmh", &patch.gps_speed_present, &patch.gps_speed_kmh) ||
         !parse_float_field(gps, "heading_deg", &patch.gps_heading_present, &patch.gps_heading_deg) ||
         !parse_int_field(gps, "satellites", &patch.gps_satellites_present, &patch.gps_satellites) ||
         !parse_bool_field(gps, "fix_valid", &patch.gps_fix_valid_present, &patch.gps_fix_valid))) {
        cJSON_Delete(json);
        return send_error(req, 400, "bad_request", "Invalid field inside gps");
    }

    if ((attitude != NULL) &&
        (!parse_float_field(attitude, "pitch_deg", &patch.attitude_pitch_present, &patch.attitude_pitch_deg) ||
         !parse_float_field(attitude, "roll_deg", &patch.attitude_roll_present, &patch.attitude_roll_deg) ||
         !parse_float_field(attitude, "yaw_deg", &patch.attitude_yaw_present, &patch.attitude_yaw_deg))) {
        cJSON_Delete(json);
        return send_error(req, 400, "bad_request", "Invalid field inside attitude");
    }

    if ((baro != NULL) &&
        !parse_float_field(baro, "baro_altitude_m", &patch.baro_altitude_present, &patch.baro_altitude_m)) {
        cJSON_Delete(json);
        return send_error(req, 400, "bad_request", "Invalid field inside barometric_altitude");
    }

    if ((battery != NULL) &&
        (!parse_float_field(battery, "voltage_v", &patch.battery_voltage_present, &patch.battery_voltage_v) ||
         !parse_float_field(battery, "current_a", &patch.battery_current_present, &patch.battery_current_a))) {
        cJSON_Delete(json);
        return send_error(req, 400, "bad_request", "Invalid field inside battery");
    }

    if (flight_mode != NULL) {
        cJSON *mode_name = cJSON_GetObjectItemCaseSensitive(flight_mode, "mode_name");
        if (!cJSON_IsString(mode_name) || !is_valid_flight_mode(mode_name->valuestring)) {
            cJSON_Delete(json);
            return send_error(req, 400, "bad_request", "Field flight_mode.mode_name is invalid");
        }

        patch.flight_mode_present = true;
        snprintf(patch.flight_mode, sizeof(patch.flight_mode), "%s", mode_name->valuestring);
    }

    app_state_write(apply_manual_telemetry_patch, &patch);
    (void) storage_save_runtime_state();
    cJSON_Delete(json);

    app_state_t state;
    app_state_get_snapshot(&state);
    return send_json(req, json_create_manual_telemetry(&state), 200);
}

static void set_gps_source_callback(app_state_t *state, void *ctx)
{
    gps_source_t *source = (gps_source_t *) ctx;
    telemetry_manager_set_gps_source_locked(state, *source);
}

static esp_err_t patch_telemetry_source_handler(httpd_req_t *req)
{
    char *body = read_request_body(req);
    if (body == NULL) {
        return send_error(req, 400, "bad_request", "Missing request body");
    }

    cJSON *json = cJSON_Parse(body);
    free(body);
    if (json == NULL) {
        return send_error(req, 400, "bad_request", "Invalid JSON body");
    }

    const cJSON *gps_source_item = cJSON_GetObjectItemCaseSensitive(json, "gps_source");
    if (!cJSON_IsString(gps_source_item)) {
        cJSON_Delete(json);
        return send_error(req, 400, "bad_request", "Field gps_source must be a string");
    }

    gps_source_t source;
    if (!app_state_parse_gps_source(gps_source_item->valuestring, &source)) {
        cJSON_Delete(json);
        return send_error(req, 400, "bad_request", "Unsupported gps_source value");
    }

    app_state_write(set_gps_source_callback, &source);
    (void) storage_save_runtime_state();
    cJSON_Delete(json);

    app_state_t state;
    app_state_get_snapshot(&state);
    return send_json(req, json_create_telemetry_overview(&state), 200);
}

static esp_err_t get_simulator_handler(httpd_req_t *req)
{
    app_state_t state;
    app_state_get_snapshot(&state);
    return send_json(req, json_create_simulator(&state), 200);
}

static esp_err_t get_profiles_handler(httpd_req_t *req)
{
    app_state_t state;
    sdcard_json_profile_list_t profiles;
    esp_err_t err = sdcard_json_list_profiles(&profiles);
    if (err != ESP_OK) {
        return send_error(req, 503, "subsystem_unavailable", "SDCard is not mounted");
    }

    app_state_get_snapshot(&state);
    return send_json(req, json_create_profiles_list_response(&profiles, state.simulator.active_profile_path), 200);
}

static void apply_simulator_parameters_patch(app_state_t *state, void *ctx)
{
    simulator_parameters_patch_t *patch = (simulator_parameters_patch_t *) ctx;

    if (patch->version_present) {
        state->simulator.parameters.version = patch->version;
    }
    if (patch->start_latitude_present) {
        state->simulator.parameters.start_latitude = patch->start_latitude;
    }
    if (patch->start_longitude_present) {
        state->simulator.parameters.start_longitude = patch->start_longitude;
    }
    if (patch->altitude_present) {
        state->simulator.parameters.altitude_m = patch->altitude_m;
    }
    if (patch->speed_present) {
        state->simulator.parameters.speed_kmh = patch->speed_kmh;
    }
    if (patch->heading_present) {
        state->simulator.parameters.heading_deg = patch->heading_deg;
    }
    if (patch->update_interval_present) {
        state->simulator.parameters.update_interval_ms = (uint32_t) patch->update_interval_ms;
    }
    if (patch->satellites_present) {
        state->simulator.parameters.satellites = (uint8_t) patch->satellites;
    }
    if (patch->fix_valid_present) {
        state->simulator.parameters.fix_valid = patch->fix_valid;
    }

    state->simulator.next_emit_in_ms = (int32_t) state->simulator.parameters.update_interval_ms;
    telemetry_manager_rebuild_locked(state);
}

static esp_err_t patch_simulator_parameters_handler(httpd_req_t *req)
{
    char *body = read_request_body(req);
    if (body == NULL) {
        return send_error(req, 400, "bad_request", "Missing request body");
    }

    cJSON *json = cJSON_Parse(body);
    free(body);
    if (json == NULL) {
        return send_error(req, 400, "bad_request", "Invalid JSON body");
    }

    simulator_parameters_patch_t patch = {0};
    if (!parse_int_field(json, "version", &patch.version_present, &patch.version) ||
        !parse_double_field(json, "start_latitude", &patch.start_latitude_present, &patch.start_latitude) ||
        !parse_double_field(json, "start_longitude", &patch.start_longitude_present, &patch.start_longitude) ||
        !parse_float_field(json, "altitude_m", &patch.altitude_present, &patch.altitude_m) ||
        !parse_float_field(json, "speed_kmh", &patch.speed_present, &patch.speed_kmh) ||
        !parse_float_field(json, "heading_deg", &patch.heading_present, &patch.heading_deg) ||
        !parse_int_field(json, "update_interval_ms", &patch.update_interval_present, &patch.update_interval_ms) ||
        !parse_int_field(json, "satellites", &patch.satellites_present, &patch.satellites) ||
        !parse_bool_field(json, "fix_valid", &patch.fix_valid_present, &patch.fix_valid)) {
        cJSON_Delete(json);
        return send_error(req, 400, "bad_request", "Invalid simulator parameter field");
    }

    app_state_write(apply_simulator_parameters_patch, &patch);
    cJSON_Delete(json);

    app_state_t state;
    app_state_get_snapshot(&state);
    return send_json(req, json_create_simulator(&state), 200);
}

static esp_err_t post_profile_load_handler(httpd_req_t *req)
{
    char *body = read_request_body(req);
    char requested_path[APP_MAX_PROFILE_PATH_LEN];
    if (body == NULL) {
        return send_error(req, 400, "bad_request", "Missing request body");
    }

    cJSON *json = cJSON_Parse(body);
    free(body);
    if (json == NULL) {
        return send_error(req, 400, "bad_request", "Invalid JSON body");
    }

    cJSON *profile_path = cJSON_GetObjectItemCaseSensitive(json, "profile_path");
    if (!cJSON_IsString(profile_path)) {
        cJSON_Delete(json);
        return send_error(req, 400, "bad_request", "Field profile_path must be a string");
    }

    snprintf(requested_path, sizeof(requested_path), "%s", profile_path->valuestring);
    esp_err_t err = sdcard_json_load_profile(requested_path);
    cJSON_Delete(json);
    if (err == ESP_ERR_NOT_FOUND) {
        return send_error(req, 404, "not_found", "Profile file does not exist or SDCard is unavailable");
    }
    if (err != ESP_OK) {
        return send_error(req, 400, "bad_request", "Failed to load profile");
    }

    (void) storage_save_runtime_state();
    return send_json(req, json_create_operation_accepted("load_profile", requested_path, "Profile loaded"), 202);
}

static esp_err_t post_profile_save_handler(httpd_req_t *req)
{
    char *body;
    bool overwrite = true;
    const char *path_value = NULL;
    char requested_path[APP_MAX_PROFILE_PATH_LEN] = {0};

    if (req->content_len > 0) {
        cJSON *json;
        cJSON *profile_path;
        cJSON *overwrite_item;

        body = read_request_body(req);
        if (body == NULL) {
            return send_error(req, 400, "bad_request", "Invalid request body");
        }

        json = cJSON_Parse(body);
        free(body);
        if (json == NULL) {
            return send_error(req, 400, "bad_request", "Invalid JSON body");
        }

        profile_path = cJSON_GetObjectItemCaseSensitive(json, "profile_path");
        overwrite_item = cJSON_GetObjectItemCaseSensitive(json, "overwrite");
        if ((profile_path != NULL) && !cJSON_IsString(profile_path)) {
            cJSON_Delete(json);
            return send_error(req, 400, "bad_request", "Field profile_path must be a string");
        }
        if ((overwrite_item != NULL) && !cJSON_IsBool(overwrite_item)) {
            cJSON_Delete(json);
            return send_error(req, 400, "bad_request", "Field overwrite must be boolean");
        }

        path_value = (profile_path != NULL) ? profile_path->valuestring : NULL;
        if (path_value != NULL) {
            snprintf(requested_path, sizeof(requested_path), "%s", path_value);
            path_value = requested_path;
        }
        overwrite = (overwrite_item != NULL) ? cJSON_IsTrue(overwrite_item) : true;

        esp_err_t err = sdcard_json_save_profile(path_value, overwrite);
        cJSON_Delete(json);
        if (err == ESP_ERR_NOT_FOUND) {
            return send_error(req, 503, "subsystem_unavailable", "SDCard is not mounted");
        }
        if (err == ESP_ERR_INVALID_STATE) {
            return send_error(req, 409, "state_conflict", "Profile already exists");
        }
        if (err != ESP_OK) {
            return send_error(req, 400, "bad_request", "Failed to save profile");
        }
    } else {
        esp_err_t err = sdcard_json_save_profile(NULL, true);
        if (err == ESP_ERR_NOT_FOUND) {
            return send_error(req, 503, "subsystem_unavailable", "SDCard is not mounted");
        }
        if (err != ESP_OK) {
            return send_error(req, 400, "bad_request", "Failed to save profile");
        }
    }

    (void) storage_save_runtime_state();
    app_state_t state;
    app_state_get_snapshot(&state);
    return send_json(req, json_create_operation_accepted("save_profile", state.simulator.active_profile_path, "Profile saved"), 202);
}

static void apply_simulator_command(app_state_t *state, void *ctx)
{
    simulator_command_t *command = (simulator_command_t *) ctx;
    telemetry_manager_handle_simulator_command_locked(state, *command);
}

static esp_err_t post_simulator_command_handler(httpd_req_t *req)
{
    char *body = read_request_body(req);
    if (body == NULL) {
        return send_error(req, 400, "bad_request", "Missing request body");
    }

    cJSON *json = cJSON_Parse(body);
    free(body);
    if (json == NULL) {
        return send_error(req, 400, "bad_request", "Invalid JSON body");
    }

    const cJSON *command_item = cJSON_GetObjectItemCaseSensitive(json, "command");
    if (!cJSON_IsString(command_item)) {
        cJSON_Delete(json);
        return send_error(req, 400, "bad_request", "Field command must be a string");
    }

    simulator_command_t command;
    if (!app_state_parse_simulator_command(command_item->valuestring, &command)) {
        cJSON_Delete(json);
        return send_error(req, 400, "bad_request", "Unsupported simulator command");
    }

    app_state_write(apply_simulator_command, &command);
    cJSON_Delete(json);

    app_state_t state;
    app_state_get_snapshot(&state);
    return send_json(req, json_create_simulator(&state), 200);
}

static esp_err_t get_gps_status_handler(httpd_req_t *req)
{
    app_state_t state;
    app_state_get_snapshot(&state);
    return send_json(req, json_create_gps_status(&state), 200);
}

static esp_err_t send_sse_json_event(httpd_req_t *req, const char *event_name, cJSON *json)
{
    char *payload = cJSON_PrintUnformatted(json);
    char header[64];
    esp_err_t err;

    cJSON_Delete(json);
    if (payload == NULL) {
        return ESP_ERR_NO_MEM;
    }

    snprintf(header, sizeof(header), "event: %s\n", event_name);
    err = httpd_resp_send_chunk(req, header, HTTPD_RESP_USE_STRLEN);
    if (err == ESP_OK) {
        err = httpd_resp_send_chunk(req, "data: ", 6);
    }
    if (err == ESP_OK) {
        err = httpd_resp_send_chunk(req, payload, HTTPD_RESP_USE_STRLEN);
    }
    if (err == ESP_OK) {
        err = httpd_resp_send_chunk(req, "\n\n", 2);
    }

    free(payload);
    return err;
}

static esp_err_t get_events_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/event-stream");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Connection", "keep-alive");

    for (int i = 0; i < APP_WEB_SSE_MAX_BURST_COUNT; ++i) {
        app_state_t state;
        esp_err_t err;

        app_state_get_snapshot(&state);
        err = send_sse_json_event(req, "system_state", json_create_system_state(&state));
        if (err != ESP_OK) {
            return err;
        }

        err = send_sse_json_event(req, "telemetry_overview", json_create_telemetry_overview(&state));
        if (err != ESP_OK) {
            return err;
        }

        err = send_sse_json_event(req, "link_statistics", json_create_link_statistics(&state));
        if (err != ESP_OK) {
            return err;
        }

        vTaskDelay(pdMS_TO_TICKS(APP_WEB_SSE_INTERVAL_MS));
    }

    return httpd_resp_send_chunk(req, NULL, 0);
}

static const char INDEX_HTML[] =
"<!doctype html><html><head><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
"<title>CRSF Display</title><style>"
":root{color-scheme:dark;--bg:#120f12;--screen:#08110d;--panel:#132d24;--line:#245246;--text:#f9f1db;--muted:#7ca599;--acc:#ffd36e;--cyan:#82e6d2;--bad:#ff737c;--ok:#90f0a5}"
"*{box-sizing:border-box}body{margin:0;background:radial-gradient(circle at top left,#3a211c 0,transparent 32%),linear-gradient(180deg,#100d10,#0d0c0f);color:var(--text);font-family:system-ui,-apple-system,Segoe UI,sans-serif}"
".wrap{max-width:1180px;margin:0 auto;padding:18px}.top{display:flex;gap:12px;align-items:center;justify-content:space-between;margin-bottom:14px}.brand{display:flex;gap:10px;align-items:center}.dot{width:12px;height:12px;border-radius:50%;background:var(--bad);box-shadow:0 0 18px var(--bad)}.dot.ok{background:var(--ok);box-shadow:0 0 18px var(--ok)}"
"h1{font-size:24px;margin:0;color:var(--acc)}.sub{color:var(--muted);font-size:13px}.grid{display:grid;grid-template-columns:320px 1fr;gap:14px}.lcd,.panel{border:1px solid var(--line);border-radius:18px;background:rgba(19,45,36,.28);box-shadow:inset 0 0 24px rgba(130,230,210,.07)}"
".lcd{height:240px;padding:12px;background:var(--screen);font-family:ui-monospace,SFMono-Regular,Consolas,monospace}.lcdHead,.lcdFoot{display:flex;justify-content:space-between;color:var(--cyan);font-size:11px;text-transform:uppercase;letter-spacing:.08em}.lcdFoot{border-top:1px solid rgba(130,230,210,.18);padding-top:7px;color:var(--muted)}"
".channels{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;margin:10px 0}.ch{border:1px solid rgba(130,230,210,.16);border-radius:10px;padding:5px;background:rgba(19,45,36,.45)}.ch b{display:block;color:var(--cyan);font-size:10px}.ch span{font-size:14px}.bar{height:4px;background:#24342d;border-radius:99px;overflow:hidden;margin-top:4px}.bar i{display:block;height:100%;background:linear-gradient(90deg,var(--cyan),var(--acc));width:50%}"
".panel{padding:14px}.tabs{display:flex;flex-wrap:wrap;gap:8px;margin-bottom:12px}.tabs button,.cmd button{border:1px solid var(--line);background:rgba(255,255,255,.03);color:var(--text);border-radius:999px;padding:7px 10px}.tabs button.active{border-color:var(--acc);color:var(--acc);background:rgba(255,211,110,.12)}"
".cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:10px}.card{border:1px solid rgba(130,230,210,.14);border-radius:14px;background:rgba(8,17,13,.55);padding:10px}.card small{display:block;color:var(--cyan);text-transform:uppercase;letter-spacing:.08em}.card strong{display:block;margin-top:4px;font-size:18px}.muted{color:var(--muted)}"
".cmd{display:flex;gap:8px;flex-wrap:wrap;margin-top:12px}pre{white-space:pre-wrap;margin:0;font-size:12px;color:var(--muted)}@media(max-width:760px){.grid{grid-template-columns:1fr}.lcd{max-width:320px}}"
"</style></head><body><div class=wrap><div class=top><div class=brand><div id=linkDot class=dot></div><div><h1>CRSF Display</h1><div class=sub id=status>loading...</div></div></div><div class=sub id=time></div></div>"
"<div class=grid><div class=lcd><div class=lcdHead><span id=lcdTitle>CRSF Channels</span><span id=lcdStatus>LINK</span></div><div id=lcdBody class=channels></div><div class=lcdFoot><span>A Back</span><span>B Select</span><span>C Next</span></div></div>"
"<main class=panel><div class=tabs id=tabs></div><div class=cards id=cards></div><div class=cmd id=cmd></div></main></div></div>"
"<script>"
"const screens=['SCREEN_CRSF_CHANNELS','SCREEN_LINK_STATISTICS','SCREEN_TELEMETRY_OVERVIEW','SCREEN_MANUAL_TELEMETRY','SCREEN_SIMULATION_PARAMETERS','SCREEN_GPS_STATUS'];"
"const names={SCREEN_CRSF_CHANNELS:'Channels',SCREEN_LINK_STATISTICS:'Link',SCREEN_TELEMETRY_OVERVIEW:'Telemetry',SCREEN_MANUAL_TELEMETRY:'Manual',SCREEN_SIMULATION_PARAMETERS:'Simulator',SCREEN_GPS_STATUS:'GPS'};"
"let state=null;async function api(p,o){let r=await fetch(p,o);if(!r.ok)throw new Error(await r.text());return r.json()}"
"async function setScreen(s){await api('/api/v1/ui/state',{method:'PATCH',headers:{'Content-Type':'application/json'},body:JSON.stringify({screen_id:s})});refresh()}"
"async function setGps(g){await api('/api/v1/telemetry/source',{method:'PATCH',headers:{'Content-Type':'application/json'},body:JSON.stringify({gps_source:g})});refresh()}"
"async function sim(c){await api('/api/v1/simulator/commands',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({command:c})});refresh()}"
"function card(k,v){return `<div class=card><small>${k}</small><strong>${v==null?'':v}</strong></div>`}"
"function renderTabs(){tabs.innerHTML=screens.map(s=>`<button class='${state.ui.screen_id==s?'active':''}' onclick=\"setScreen('${s}')\">${names[s]}</button>`).join('')}"
"function renderLcd(){lcdTitle.textContent=names[state.ui.screen_id];lcdStatus.textContent=state.crsf_link.link_up?'LINK UP':'LINK DOWN';let ch=state.channels.channels||[];lcdBody.innerHTML=ch.map(c=>{let p=Math.round(c.normalized_value*100),w=Math.max(0,Math.min(100,(p+100)/2));return `<div class=ch><b>${c.name}</b><span>${p>0?'+':''}${p}</span><div class=bar><i style='width:${w}%'></i></div></div>`}).join('')}"
"function renderCards(){let s=state.ui.screen_id,h='';cmd.innerHTML='';if(s==='SCREEN_CRSF_CHANNELS'){h=state.channels.channels.map(c=>card(c.name,Math.round(c.normalized_value*100)+'%')).join('')}"
"else if(s==='SCREEN_LINK_STATISTICS'){let x=state.link_statistics.statistics;h=card('Uplink LQ',x.uplink_lq_percent+'%')+card('RSSI 1','-'+x.uplink_rssi_ant1_dbm_neg+' dBm')+card('RSSI 2','-'+x.uplink_rssi_ant2_dbm_neg+' dBm')+card('SNR',x.uplink_snr_db+' dB')+card('Down LQ',x.downlink_lq_percent+'%')+card('TX Power',x.uplink_tx_power)}"
"else if(s==='SCREEN_TELEMETRY_OVERVIEW'){let t=state.telemetry,g=t.gps;h=card('GPS Source',state.gps_source)+card('Fix',g.fix_valid?'YES':'NO')+card('Satellites',g.satellites)+card('Latitude',g.latitude.toFixed(6))+card('Longitude',g.longitude.toFixed(6))+card('Speed',g.ground_speed_kmh+' km/h')+card('Battery',t.battery.voltage_v+' V');cmd.innerHTML=['GPS_SOURCE_MANUAL','GPS_SOURCE_EXTERNAL_GPS','GPS_SOURCE_SIMULATOR'].map(g=>`<button onclick=\"setGps('${g}')\">${g.replace('GPS_SOURCE_','')}</button>`).join('')}"
"else if(s==='SCREEN_MANUAL_TELEMETRY'){let g=state.manual_telemetry.values.gps;h=card('Manual view','Use API PATCH /api/v1/telemetry/manual')+card('Lat',g.latitude.toFixed(6))+card('Lon',g.longitude.toFixed(6))+card('Alt',g.gps_altitude_m+' m')}"
"else if(s==='SCREEN_SIMULATION_PARAMETERS'){let x=state.simulator,p=x.parameters;h=card('State',x.state)+card('Profile',x.active_profile_name)+card('Lat',p.start_latitude.toFixed(6))+card('Lon',p.start_longitude.toFixed(6))+card('Speed',p.speed_kmh+' km/h')+card('SD',x.sdcard.mounted?'mounted':'not mounted');cmd.innerHTML=['start','pause','resume','stop','reset'].map(c=>`<button onclick=\"sim('${c}')\">${c}</button>`).join('')}"
"else{let g=state.gps_status;h=card('Connected',g.connected?'YES':'NO')+card('Stream',g.data_stream_present?'YES':'NO')+card('Protocol',g.protocol)+card('Errors',g.parser_errors)+card('Serial',g.serial.rx_gpio+'/'+g.serial.tx_gpio)}cards.innerHTML=h}"
"async function refresh(){try{state=await api('/api/v1/system/state');linkDot.className='dot '+(state.crsf_link.link_up?'ok':'');status.textContent=`GPS ${state.gps_source} | WiFi ${state.wifi.ready?'ready':'booting'} | Web ${state.wifi.web_ui_ready?'ready':'booting'}`;time.textContent=state.device_time_ms+' ms';renderTabs();renderLcd();renderCards()}catch(e){status.textContent=e.message}}"
"refresh();setInterval(refresh,1000);"
"</script></body></html>";

static esp_err_t get_root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

esp_err_t web_ui_start(void)
{
    if (s_http_server != NULL) {
        return ESP_OK;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = APP_HTTP_SERVER_PORT;
    config.max_uri_handlers = APP_HTTP_MAX_URI_HANDLERS;
    config.stack_size = APP_HTTP_SERVER_STACK_SIZE;
    config.uri_match_fn = httpd_uri_match_wildcard;

    ESP_LOGI(TAG, "Starting HTTP server on port %u", config.server_port);
    esp_err_t err = httpd_start(&s_http_server, &config);
    if (err != ESP_OK) {
        return err;
    }

    const httpd_uri_t handlers[] = {
        {.uri = "/", .method = HTTP_GET, .handler = get_root_handler, .user_ctx = NULL},
        {.uri = "/api/v1/health", .method = HTTP_GET, .handler = get_health_handler, .user_ctx = NULL},
        {.uri = "/api/v1/system/state", .method = HTTP_GET, .handler = get_system_state_handler, .user_ctx = NULL},
        {.uri = "/api/v1/events", .method = HTTP_GET, .handler = get_events_handler, .user_ctx = NULL},
        {.uri = "/api/v1/ui/state", .method = HTTP_GET, .handler = get_ui_state_handler, .user_ctx = NULL},
        {.uri = "/api/v1/ui/state", .method = HTTP_PATCH, .handler = patch_ui_state_handler, .user_ctx = NULL},
        {.uri = "/api/v1/channels", .method = HTTP_GET, .handler = get_channels_handler, .user_ctx = NULL},
        {.uri = "/api/v1/link-statistics", .method = HTTP_GET, .handler = get_link_statistics_handler, .user_ctx = NULL},
        {.uri = "/api/v1/telemetry/overview", .method = HTTP_GET, .handler = get_telemetry_overview_handler, .user_ctx = NULL},
        {.uri = "/api/v1/telemetry/manual", .method = HTTP_GET, .handler = get_manual_telemetry_handler, .user_ctx = NULL},
        {.uri = "/api/v1/telemetry/manual", .method = HTTP_PATCH, .handler = patch_manual_telemetry_handler, .user_ctx = NULL},
        {.uri = "/api/v1/telemetry/source", .method = HTTP_PATCH, .handler = patch_telemetry_source_handler, .user_ctx = NULL},
        {.uri = "/api/v1/simulator", .method = HTTP_GET, .handler = get_simulator_handler, .user_ctx = NULL},
        {.uri = "/api/v1/simulator/parameters", .method = HTTP_PATCH, .handler = patch_simulator_parameters_handler, .user_ctx = NULL},
        {.uri = "/api/v1/simulator/commands", .method = HTTP_POST, .handler = post_simulator_command_handler, .user_ctx = NULL},
        {.uri = "/api/v1/simulator/profiles", .method = HTTP_GET, .handler = get_profiles_handler, .user_ctx = NULL},
        {.uri = "/api/v1/simulator/profiles/load", .method = HTTP_POST, .handler = post_profile_load_handler, .user_ctx = NULL},
        {.uri = "/api/v1/simulator/profiles/save", .method = HTTP_POST, .handler = post_profile_save_handler, .user_ctx = NULL},
        {.uri = "/api/v1/gps/status", .method = HTTP_GET, .handler = get_gps_status_handler, .user_ctx = NULL},
        {.uri = "/api/v1/*", .method = HTTP_OPTIONS, .handler = options_handler, .user_ctx = NULL},
    };

    for (size_t i = 0; i < (sizeof(handlers) / sizeof(handlers[0])); ++i) {
        err = httpd_register_uri_handler(s_http_server, &handlers[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to register handler for %s", handlers[i].uri);
            return err;
        }
    }

    wifi_manager_set_web_ui_ready(true);
    return ESP_OK;
}
