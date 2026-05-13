#include "wifi/wifi_manager.h"

#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "app_state.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "lwip/ip4_addr.h"

static const char *TAG = "wifi_manager";

static esp_netif_t *s_ap_netif;

typedef struct {
    int32_t event_id;
} wifi_event_update_t;

static void fill_ip_address(wifi_status_t *wifi_state)
{
    if ((s_ap_netif == NULL) || (wifi_state == NULL)) {
        return;
    }

    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(s_ap_netif, &ip_info) != ESP_OK) {
        return;
    }

    snprintf(wifi_state->ip_address, sizeof(wifi_state->ip_address), IPSTR, IP2STR(&ip_info.ip));
}

static void update_wifi_ip_address(app_state_t *state, void *ctx)
{
    (void) ctx;
    fill_ip_address(&state->wifi);
}

static void update_wifi_state_for_event(app_state_t *state, void *ctx)
{
    wifi_event_update_t *update = (wifi_event_update_t *) ctx;

    if (update == NULL) {
        return;
    }

    if (update->event_id == WIFI_EVENT_AP_START) {
        state->wifi.ready = true;
        fill_ip_address(&state->wifi);
    } else if (update->event_id == WIFI_EVENT_AP_STOP) {
        state->wifi.ready = false;
    } else if (update->event_id == WIFI_EVENT_AP_STACONNECTED) {
        state->wifi.connected_clients += 1;
    } else if (update->event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        if (state->wifi.connected_clients > 0) {
            state->wifi.connected_clients -= 1;
        }
    }
}

static void update_web_ui_ready_flag(app_state_t *state, void *ctx)
{
    bool *ready = (bool *) ctx;

    if (ready == NULL) {
        return;
    }

    state->wifi.web_ui_ready = *ready;
}

static void sync_ip_address(void)
{
    app_state_write(update_wifi_ip_address, NULL);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void) arg;
    (void) event_base;
    (void) event_data;

    wifi_event_update_t update = {
        .event_id = event_id,
    };

    app_state_write(update_wifi_state_for_event, &update);
}

esp_err_t wifi_manager_start(void)
{
    esp_err_t err = esp_netif_init();
    if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
        return err;
    }

    err = esp_event_loop_create_default();
    if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
        return err;
    }

    if (s_ap_netif == NULL) {
        s_ap_netif = esp_netif_create_default_wifi_ap();
        if (s_ap_netif == NULL) {
            return ESP_FAIL;
        }
    }

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&init_config);
    if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
        return err;
    }

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {0};
    snprintf((char *) wifi_config.ap.ssid, sizeof(wifi_config.ap.ssid), "%s", APP_WIFI_SOFTAP_SSID);
    wifi_config.ap.ssid_len = strlen(APP_WIFI_SOFTAP_SSID);
    wifi_config.ap.max_connection = APP_WIFI_SOFTAP_MAX_CONNECTIONS;
    wifi_config.ap.channel = 1;
    wifi_config.ap.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    sync_ip_address();
    ESP_LOGI(TAG, "SoftAP started with SSID %s", APP_WIFI_SOFTAP_SSID);
    return ESP_OK;
}

void wifi_manager_set_web_ui_ready(bool ready)
{
    app_state_write(update_web_ui_ready_flag, &ready);
}
