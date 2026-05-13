#include "storage/storage.h"

#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "app_state.h"
#include "esp_log.h"
#include "nvs.h"
#include "telemetry/telemetry_manager.h"

static const char *TAG = "storage";

typedef struct {
    bool has_gps_source;
    gps_source_t gps_source;
    bool has_manual_telemetry;
    telemetry_data_t manual_telemetry;
    bool has_profile_path;
    char profile_path[APP_MAX_PROFILE_PATH_LEN];
} storage_loaded_state_t;

typedef struct {
    gps_source_t gps_source;
    telemetry_data_t manual_telemetry;
    char profile_path[APP_MAX_PROFILE_PATH_LEN];
} storage_saved_state_t;

static void apply_loaded_state(app_state_t *state, void *ctx)
{
    storage_loaded_state_t *loaded = (storage_loaded_state_t *) ctx;

    if (loaded->has_gps_source) {
        state->gps_source = loaded->gps_source;
        state->ui.gps_source = loaded->gps_source;
    }

    if (loaded->has_manual_telemetry) {
        state->manual_telemetry = loaded->manual_telemetry;
    }

    if (loaded->has_profile_path) {
        const char *profile_name = NULL;
        size_t profile_name_len = 0;

        snprintf(state->simulator.active_profile_path,
                 sizeof(state->simulator.active_profile_path),
                 "%s",
                 loaded->profile_path);

        profile_name = strrchr(loaded->profile_path, '/');
        profile_name = (profile_name != NULL) ? (profile_name + 1) : loaded->profile_path;
        profile_name_len = strnlen(profile_name, sizeof(state->simulator.active_profile_name) - 1U);
        memcpy(state->simulator.active_profile_name, profile_name, profile_name_len);
        state->simulator.active_profile_name[profile_name_len] = '\0';
    }

    telemetry_manager_rebuild_locked(state);
}

static void capture_runtime_state(const app_state_t *state, void *ctx)
{
    storage_saved_state_t *saved = (storage_saved_state_t *) ctx;

    if (saved == NULL) {
        return;
    }

    saved->gps_source = state->gps_source;
    saved->manual_telemetry = state->manual_telemetry;
    snprintf(saved->profile_path, sizeof(saved->profile_path), "%s", state->simulator.active_profile_path);
}

esp_err_t storage_load_runtime_state(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(APP_NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "NVS namespace not initialized yet");
        return ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }

    storage_loaded_state_t loaded = {0};
    uint8_t gps_source_raw = 0;
    size_t blob_size = sizeof(telemetry_data_t);
    size_t str_size = sizeof(loaded.profile_path);

    if (nvs_get_u8(nvs_handle, APP_NVS_KEY_GPS_SOURCE, &gps_source_raw) == ESP_OK) {
        loaded.has_gps_source = true;
        loaded.gps_source = (gps_source_t) gps_source_raw;
    }

    if (nvs_get_blob(nvs_handle, APP_NVS_KEY_MANUAL_TLM, &loaded.manual_telemetry, &blob_size) == ESP_OK &&
        blob_size == sizeof(telemetry_data_t)) {
        loaded.has_manual_telemetry = true;
    }

    if (nvs_get_str(nvs_handle, APP_NVS_KEY_PROFILE_PATH, loaded.profile_path, &str_size) == ESP_OK) {
        loaded.has_profile_path = true;
    }

    nvs_close(nvs_handle);
    app_state_write(apply_loaded_state, &loaded);
    ESP_LOGI(TAG, "Runtime state loaded from NVS");
    return ESP_OK;
}

esp_err_t storage_save_runtime_state(void)
{
    storage_saved_state_t saved = {0};
    nvs_handle_t nvs_handle;

    app_state_read(capture_runtime_state, &saved);

    esp_err_t err = nvs_open(APP_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_u8(nvs_handle, APP_NVS_KEY_GPS_SOURCE, (uint8_t) saved.gps_source);
    if (err == ESP_OK) {
        err = nvs_set_blob(nvs_handle, APP_NVS_KEY_MANUAL_TLM, &saved.manual_telemetry, sizeof(saved.manual_telemetry));
    }
    if (err == ESP_OK) {
        err = nvs_set_str(nvs_handle, APP_NVS_KEY_PROFILE_PATH, saved.profile_path);
    }
    if (err == ESP_OK) {
        err = nvs_commit(nvs_handle);
    }

    nvs_close(nvs_handle);
    return err;
}
