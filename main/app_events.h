#ifndef APP_EVENTS_H
#define APP_EVENTS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "telemetry/telemetry_types.h"

typedef enum {
    BTN_A_SHORT = 0,
    BTN_B_SHORT,
    BTN_C_SHORT,
    BTN_A_LONG,
    BTN_B_LONG,
    BTN_C_LONG,
} button_event_id_t;

typedef enum {
    APP_EVENT_BUTTON = 0,
    APP_EVENT_CRSF_LINK_UP,
    APP_EVENT_CRSF_LINK_DOWN,
    APP_EVENT_CRSF_CHANNELS_UPDATED,
    APP_EVENT_CRSF_LINK_STATS_UPDATED,
    APP_EVENT_TELEMETRY_TX_OK,
    APP_EVENT_TELEMETRY_TX_ERROR,
    APP_EVENT_GPS_DATA_UPDATED,
    APP_EVENT_GPS_FIX_LOST,
    APP_EVENT_SIMULATOR_STARTED,
    APP_EVENT_SIMULATOR_PAUSED,
    APP_EVENT_SIMULATOR_STOPPED,
    APP_EVENT_SIMULATOR_DATA_UPDATED,
    APP_EVENT_SIMULATOR_JSON_SAVED,
    APP_EVENT_SIMULATOR_JSON_LOADED,
    APP_EVENT_SDCARD_MOUNT_ERROR,
    APP_EVENT_WIFI_READY,
    APP_EVENT_WEB_CLIENT_CONNECTED,
    APP_EVENT_WEB_CLIENT_DISCONNECTED,
    APP_EVENT_WEB_CONFIG_UPDATED,
    APP_EVENT_TICK_UI,
    APP_EVENT_SET_GPS_SOURCE,
    APP_EVENT_SIMULATOR_COMMAND,
} app_event_type_t;

typedef enum {
    SIMULATOR_COMMAND_START = 0,
    SIMULATOR_COMMAND_PAUSE,
    SIMULATOR_COMMAND_RESUME,
    SIMULATOR_COMMAND_STOP,
    SIMULATOR_COMMAND_RESET,
} simulator_command_t;

typedef struct {
    app_event_type_t type;
    int64_t timestamp_ms;
    union {
        button_event_id_t button_id;
        gps_source_t gps_source;
        simulator_command_t simulator_command;
    } data;
} app_event_t;

esp_err_t app_events_init(size_t queue_length);
esp_err_t app_events_post(const app_event_t *event, TickType_t timeout_ticks);
bool app_events_receive(app_event_t *event, TickType_t timeout_ticks);

#endif
