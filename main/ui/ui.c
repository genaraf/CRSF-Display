#include "ui/ui.h"

#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "app_events.h"
#include "app_state.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "telemetry/telemetry_manager.h"

static const char *TAG = "ui";

static void handle_button_locked(app_state_t *state, button_event_id_t button_id)
{
    switch (state->ui.screen_id) {
        case SCREEN_CRSF_CHANNELS:
            if (button_id == BTN_B_SHORT) {
                state->ui.screen_id = SCREEN_TELEMETRY_OVERVIEW;
            } else if (button_id == BTN_C_SHORT) {
                state->ui.screen_id = SCREEN_LINK_STATISTICS;
            }
            break;
        case SCREEN_LINK_STATISTICS:
            if (button_id == BTN_A_SHORT) {
                state->ui.screen_id = SCREEN_CRSF_CHANNELS;
            } else if (button_id == BTN_B_SHORT) {
                state->ui.screen_id = SCREEN_TELEMETRY_OVERVIEW;
            }
            break;
        case SCREEN_TELEMETRY_OVERVIEW:
            if (button_id == BTN_A_SHORT) {
                state->ui.screen_id = SCREEN_CRSF_CHANNELS;
            } else if (button_id == BTN_B_SHORT) {
                if (state->gps_source == GPS_SOURCE_MANUAL) {
                    state->ui.screen_id = SCREEN_MANUAL_TELEMETRY;
                } else if (state->gps_source == GPS_SOURCE_EXTERNAL_GPS) {
                    state->ui.screen_id = SCREEN_GPS_STATUS;
                } else {
                    state->ui.screen_id = SCREEN_SIMULATION_PARAMETERS;
                }
            } else if (button_id == BTN_C_SHORT) {
                gps_source_t next_source = GPS_SOURCE_MANUAL;
                if (state->gps_source == GPS_SOURCE_MANUAL) {
                    next_source = GPS_SOURCE_EXTERNAL_GPS;
                } else if (state->gps_source == GPS_SOURCE_EXTERNAL_GPS) {
                    next_source = GPS_SOURCE_SIMULATOR;
                }

                telemetry_manager_set_gps_source_locked(state, next_source);
            }
            break;
        case SCREEN_MANUAL_TELEMETRY:
        case SCREEN_SIMULATION_PARAMETERS:
        case SCREEN_GPS_STATUS:
            if (button_id == BTN_A_SHORT) {
                state->ui.screen_id = SCREEN_TELEMETRY_OVERVIEW;
            }
            break;
        default:
            break;
    }
}

static void handle_ui_event(app_state_t *state, void *ctx)
{
    app_event_t *event = (app_event_t *) ctx;

    switch (event->type) {
        case APP_EVENT_BUTTON:
            handle_button_locked(state, event->data.button_id);
            break;
        case APP_EVENT_SET_GPS_SOURCE:
            telemetry_manager_set_gps_source_locked(state, event->data.gps_source);
            break;
        case APP_EVENT_SIMULATOR_COMMAND:
            telemetry_manager_handle_simulator_command_locked(state, event->data.simulator_command);
            break;
        default:
            break;
    }

    state->ui.gps_source = state->gps_source;
}

static void ui_task(void *arg)
{
    (void) arg;

    ESP_LOGI(TAG, "Starting UI task");

    while (true) {
        app_event_t event;
        if (app_events_receive(&event, pdMS_TO_TICKS(APP_UI_REFRESH_MS))) {
            app_state_write(handle_ui_event, &event);
        }
    }
}

esp_err_t ui_start(void)
{
    BaseType_t result = xTaskCreate(ui_task, "task_ui", APP_TASK_STACK_MEDIUM, NULL, APP_TASK_PRIORITY_NORMAL, NULL);
    return (result == pdPASS) ? ESP_OK : ESP_FAIL;
}
