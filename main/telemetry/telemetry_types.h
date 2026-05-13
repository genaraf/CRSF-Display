#ifndef TELEMETRY_TYPES_H
#define TELEMETRY_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#include "app_config.h"

typedef enum {
    GPS_SOURCE_MANUAL = 0,
    GPS_SOURCE_EXTERNAL_GPS,
    GPS_SOURCE_SIMULATOR,
} gps_source_t;

typedef enum {
    SCREEN_CRSF_CHANNELS = 0,
    SCREEN_LINK_STATISTICS,
    SCREEN_TELEMETRY_OVERVIEW,
    SCREEN_MANUAL_TELEMETRY,
    SCREEN_SIMULATION_PARAMETERS,
    SCREEN_GPS_STATUS,
} ui_screen_id_t;

typedef enum {
    UI_MODE_VIEW = 0,
    UI_MODE_EDIT,
} ui_mode_t;

typedef enum {
    TELEMETRY_TX_IDLE = 0,
    TELEMETRY_TX_OK,
    TELEMETRY_TX_ERROR,
} telemetry_tx_status_t;

typedef enum {
    SIM_STOPPED = 0,
    SIM_RUNNING,
    SIM_PAUSED,
} simulator_run_state_t;

typedef enum {
    JSON_OP_NONE = 0,
    JSON_OP_LOAD_PROFILE,
    JSON_OP_SAVE_PROFILE,
} json_operation_type_t;

typedef enum {
    JSON_OP_STATUS_IDLE = 0,
    JSON_OP_STATUS_QUEUED,
    JSON_OP_STATUS_IN_PROGRESS,
    JSON_OP_STATUS_SUCCESS,
    JSON_OP_STATUS_ERROR,
} json_operation_status_code_t;

typedef enum {
    APP_WIFI_MODE_SOFTAP = 0,
    APP_WIFI_MODE_STA,
    APP_WIFI_MODE_AP_STA,
} app_wifi_mode_t;

typedef struct {
    double latitude;
    double longitude;
    float gps_altitude_m;
    float ground_speed_kmh;
    float heading_deg;
    uint8_t satellites;
    bool fix_valid;
    gps_source_t source;
} gps_telemetry_t;

typedef struct {
    float pitch_deg;
    float roll_deg;
    float yaw_deg;
} attitude_telemetry_t;

typedef struct {
    float baro_altitude_m;
} barometric_altitude_telemetry_t;

typedef struct {
    float voltage_v;
    float current_a;
} battery_telemetry_t;

typedef struct {
    char mode_name[APP_MAX_FLIGHT_MODE_LEN];
} flight_mode_telemetry_t;

typedef struct {
    gps_telemetry_t gps;
    attitude_telemetry_t attitude;
    barometric_altitude_telemetry_t barometric_altitude;
    battery_telemetry_t battery;
    flight_mode_telemetry_t flight_mode;
    int64_t timestamp_ms;
} telemetry_data_t;

typedef struct {
    char name[8];
    float normalized_value;
} rx_channel_t;

typedef struct {
    bool link_up;
    bool rx_active;
    bool frame_error;
    int64_t last_frame_age_ms;
    char uart_port_name[8];
} crsf_link_state_t;

typedef struct {
    telemetry_tx_status_t status;
    int64_t last_tx_age_ms;
    char last_error[APP_MAX_ERROR_TEXT_LEN];
} telemetry_tx_state_t;

typedef struct {
    int uplink_rssi_ant1_dbm_neg;
    int uplink_rssi_ant2_dbm_neg;
    int uplink_lq_percent;
    int uplink_snr_db;
    uint8_t active_antenna;
    uint8_t rf_mode;
    uint8_t uplink_tx_power;
    int downlink_rssi_dbm_neg;
    int downlink_lq_percent;
    int downlink_snr_db;
    int64_t timestamp_ms;
} link_statistics_t;

typedef struct {
    ui_screen_id_t screen_id;
    char selected_item[APP_MAX_SELECTED_ITEM_LEN];
    ui_mode_t mode;
    gps_source_t gps_source;
    char dialog_state[APP_MAX_DIALOG_STATE_LEN];
} ui_state_t;

typedef struct {
    bool mounted;
    char profiles_directory[APP_MAX_PROFILE_PATH_LEN];
    int64_t free_bytes;
    char last_error[APP_MAX_ERROR_TEXT_LEN];
} sdcard_status_t;

typedef struct {
    json_operation_type_t operation;
    json_operation_status_code_t status;
    int64_t timestamp_ms;
    char profile_path[APP_MAX_PROFILE_PATH_LEN];
    char message[APP_MAX_ERROR_TEXT_LEN];
} json_operation_status_t;

typedef struct {
    bool ready;
    app_wifi_mode_t mode;
    char ssid[APP_MAX_SSID_LEN];
    char ip_address[APP_MAX_IP_LEN];
    char hostname[APP_MAX_HOSTNAME_LEN];
    int connected_clients;
    bool web_ui_ready;
} wifi_status_t;

typedef struct {
    int version;
    double start_latitude;
    double start_longitude;
    float altitude_m;
    float speed_kmh;
    float heading_deg;
    uint32_t update_interval_ms;
    uint8_t satellites;
    bool fix_valid;
} simulator_parameters_t;

typedef struct {
    simulator_run_state_t state;
    simulator_parameters_t parameters;
    gps_telemetry_t current_gps;
    char active_profile_name[APP_MAX_PROFILE_NAME_LEN];
    char active_profile_path[APP_MAX_PROFILE_PATH_LEN];
    int32_t next_emit_in_ms;
    json_operation_status_t last_json_operation;
} simulator_state_t;

typedef struct {
    bool connected;
    bool data_stream_present;
    char protocol_name[16];
    char serial_type_name[16];
    int rx_gpio;
    int tx_gpio;
    int baud_rate;
    gps_telemetry_t last_fix;
    int64_t last_update_age_ms;
    bool parse_gga;
    bool parse_rmc;
    bool parse_gsa;
    bool parse_gsv;
    uint32_t parser_errors;
} gps_status_t;

typedef struct {
    char code[24];
    char message[APP_MAX_DIAGNOSTIC_MESSAGE_LEN];
} diagnostic_state_t;

#endif
