#include "sdcard/sdcard_json.h"

#include <dirent.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>

#include "app_config.h"
#include "app_state.h"
#include "cJSON.h"
#include "display/display.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"
#include "telemetry/telemetry_manager.h"

static const char *TAG = "sdcard_json";
static sdmmc_card_t *s_sdcard_card;
static bool s_sdcard_mounted;
static bool s_spi_bus_initialized;
static const spi_host_device_t SDCARD_SPI_HOST = SPI3_HOST;

typedef struct {
    bool mounted;
    int64_t free_bytes;
    char last_error[APP_MAX_ERROR_TEXT_LEN];
} sdcard_probe_result_t;

typedef struct {
    simulator_parameters_t parameters;
    char profile_path[APP_MAX_PROFILE_PATH_LEN];
    char profile_name[APP_MAX_PROFILE_NAME_LEN];
} loaded_profile_result_t;

typedef struct {
    json_operation_type_t operation;
    json_operation_status_code_t status;
    char profile_path[APP_MAX_PROFILE_PATH_LEN];
    char message[APP_MAX_ERROR_TEXT_LEN];
} json_operation_update_t;

typedef struct {
    simulator_parameters_t parameters;
    char active_profile_path[APP_MAX_PROFILE_PATH_LEN];
} save_profile_state_t;

static void copy_string_bounded(char *dst, size_t dst_size, const char *src)
{
    size_t copy_len;

    if (dst == NULL || dst_size == 0) {
        return;
    }

    if (src == NULL) {
        dst[0] = '\0';
        return;
    }

    copy_len = strnlen(src, dst_size - 1U);
    memcpy(dst, src, copy_len);
    dst[copy_len] = '\0';
}

static void basename_from_path(const char *path, char *name, size_t name_size)
{
    const char *last_slash = strrchr(path, '/');
    copy_string_bounded(name, name_size, (last_slash != NULL) ? (last_slash + 1) : path);
}

static bool path_exists(const char *path, bool *is_dir)
{
    struct stat st;
    if (stat(path, &st) != 0) {
        return false;
    }

    if (is_dir != NULL) {
        *is_dir = S_ISDIR(st.st_mode);
    }

    return true;
}

static esp_err_t mkdir_if_possible(const char *path)
{
    if (mkdir(path, 0755) == 0 || errno == EEXIST) {
        return ESP_OK;
    }

    ESP_LOGE(TAG, "mkdir('%s') failed: errno=%d", path, errno);
    return ESP_FAIL;
}

static esp_err_t ensure_profiles_directory(void)
{
    char directory[APP_MAX_PROFILE_PATH_LEN];
    size_t len = strlen(APP_SIMULATOR_DEFAULT_PROFILE_DIR);
    size_t pos = 0;

    if (len >= sizeof(directory)) {
        return ESP_ERR_INVALID_SIZE;
    }

    memset(directory, 0, sizeof(directory));
    for (size_t i = 0; i < len; ++i) {
        directory[pos++] = APP_SIMULATOR_DEFAULT_PROFILE_DIR[i];
        directory[pos] = '\0';
        if (directory[pos - 1] == '/' && pos > 1) {
            directory[pos - 1] = '\0';
            if (mkdir_if_possible(directory) != ESP_OK) {
                return ESP_FAIL;
            }
            directory[pos - 1] = '/';
        }
    }

    if (pos > 0 && directory[pos - 1] == '/') {
        directory[pos - 1] = '\0';
    }
    return mkdir_if_possible(directory);
}

static esp_err_t mount_sdcard_if_needed(sdcard_probe_result_t *probe)
{
    if (s_sdcard_mounted) {
        probe->mounted = true;
        probe->free_bytes = -1;
        return ESP_OK;
    }

    s_spi_bus_initialized = true;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SDCARD_SPI_HOST;

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.host_id = SDCARD_SPI_HOST;
    slot_config.gpio_cs = APP_SDCARD_CS_GPIO;

    esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 8,
        .allocation_unit_size = 16 * 1024,
    };

    bool display_locked = display_lock(1000);
    if (display_locked) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    esp_err_t err = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &s_sdcard_card);

    if (display_locked) {
        display_unlock();
    }

    if (err != ESP_OK) {
        snprintf(probe->last_error, sizeof(probe->last_error), "Failed to mount SDCard: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "Failed to mount SDCard: %s", esp_err_to_name(err));
        return err;
    }

    s_sdcard_mounted = true;
    probe->mounted = true;
    probe->free_bytes = -1;
    ESP_LOGI(TAG, "SDCard mounted on /sdcard");
    return ESP_OK;
}

static void apply_sdcard_probe(app_state_t *state, void *ctx)
{
    sdcard_probe_result_t *probe = (sdcard_probe_result_t *) ctx;

    state->sdcard.mounted = probe->mounted;
    state->sdcard.free_bytes = probe->free_bytes;
    if (probe->last_error[0] != '\0') {
        snprintf(state->sdcard.last_error, sizeof(state->sdcard.last_error), "%s", probe->last_error);
        snprintf(state->diagnostic.code, sizeof(state->diagnostic.code), "%s", "sdcard");
        snprintf(state->diagnostic.message, sizeof(state->diagnostic.message), "%s", probe->last_error);
    } else {
        state->sdcard.last_error[0] = '\0';
    }
}

static void update_json_operation(app_state_t *state, void *ctx)
{
    json_operation_update_t *update = (json_operation_update_t *) ctx;

    state->simulator.last_json_operation.operation = update->operation;
    state->simulator.last_json_operation.status = update->status;
    state->simulator.last_json_operation.timestamp_ms = app_state_now_ms();
    snprintf(state->simulator.last_json_operation.profile_path,
             sizeof(state->simulator.last_json_operation.profile_path),
             "%s",
             update->profile_path);
    snprintf(state->simulator.last_json_operation.message,
             sizeof(state->simulator.last_json_operation.message),
             "%s",
             update->message);
}

static void apply_loaded_profile(app_state_t *state, void *ctx)
{
    loaded_profile_result_t *loaded = (loaded_profile_result_t *) ctx;

    state->simulator.parameters = loaded->parameters;
    snprintf(state->simulator.active_profile_path,
             sizeof(state->simulator.active_profile_path),
             "%s",
             loaded->profile_path);
    snprintf(state->simulator.active_profile_name,
             sizeof(state->simulator.active_profile_name),
             "%s",
             loaded->profile_name);

    state->simulator.current_gps.latitude = loaded->parameters.start_latitude;
    state->simulator.current_gps.longitude = loaded->parameters.start_longitude;
    state->simulator.current_gps.gps_altitude_m = loaded->parameters.altitude_m;
    state->simulator.current_gps.ground_speed_kmh = loaded->parameters.speed_kmh;
    state->simulator.current_gps.heading_deg = loaded->parameters.heading_deg;
    state->simulator.current_gps.satellites = loaded->parameters.satellites;
    state->simulator.current_gps.fix_valid = loaded->parameters.fix_valid;
    state->simulator.current_gps.source = GPS_SOURCE_SIMULATOR;
    state->simulator.next_emit_in_ms = (int32_t) loaded->parameters.update_interval_ms;

    telemetry_manager_rebuild_locked(state);
}

static void capture_profile_save_state(const app_state_t *state, void *ctx)
{
    save_profile_state_t *save_state = (save_profile_state_t *) ctx;

    if (save_state == NULL) {
        return;
    }

    save_state->parameters = state->simulator.parameters;
    copy_string_bounded(save_state->active_profile_path,
                        sizeof(save_state->active_profile_path),
                        state->simulator.active_profile_path);
}

static esp_err_t parse_profile_json(const char *json_text, loaded_profile_result_t *profile)
{
    cJSON *root = cJSON_Parse(json_text);
    if (root == NULL) {
        return ESP_FAIL;
    }

    cJSON *version = cJSON_GetObjectItemCaseSensitive(root, "version");
    cJSON *start_latitude = cJSON_GetObjectItemCaseSensitive(root, "start_latitude");
    cJSON *start_longitude = cJSON_GetObjectItemCaseSensitive(root, "start_longitude");
    cJSON *altitude_m = cJSON_GetObjectItemCaseSensitive(root, "altitude_m");
    cJSON *speed_kmh = cJSON_GetObjectItemCaseSensitive(root, "speed_kmh");
    cJSON *heading_deg = cJSON_GetObjectItemCaseSensitive(root, "heading_deg");
    cJSON *update_interval_ms = cJSON_GetObjectItemCaseSensitive(root, "update_interval_ms");
    cJSON *satellites = cJSON_GetObjectItemCaseSensitive(root, "satellites");
    cJSON *fix_valid = cJSON_GetObjectItemCaseSensitive(root, "fix_valid");

    if (!cJSON_IsNumber(version) ||
        !cJSON_IsNumber(start_latitude) ||
        !cJSON_IsNumber(start_longitude) ||
        !cJSON_IsNumber(altitude_m) ||
        !cJSON_IsNumber(speed_kmh) ||
        !cJSON_IsNumber(heading_deg) ||
        !cJSON_IsNumber(update_interval_ms) ||
        !cJSON_IsNumber(satellites) ||
        !cJSON_IsBool(fix_valid)) {
        cJSON_Delete(root);
        return ESP_FAIL;
    }

    profile->parameters.version = version->valueint;
    profile->parameters.start_latitude = start_latitude->valuedouble;
    profile->parameters.start_longitude = start_longitude->valuedouble;
    profile->parameters.altitude_m = (float) altitude_m->valuedouble;
    profile->parameters.speed_kmh = (float) speed_kmh->valuedouble;
    profile->parameters.heading_deg = (float) heading_deg->valuedouble;
    profile->parameters.update_interval_ms = (uint32_t) update_interval_ms->valueint;
    profile->parameters.satellites = (uint8_t) satellites->valueint;
    profile->parameters.fix_valid = cJSON_IsTrue(fix_valid);

    cJSON_Delete(root);
    return ESP_OK;
}

esp_err_t sdcard_json_refresh_status(void)
{
    sdcard_probe_result_t probe = {0};
    esp_err_t err = mount_sdcard_if_needed(&probe);

    if (err == ESP_OK) {
        if (ensure_profiles_directory() != ESP_OK) {
            probe.mounted = false;
            probe.free_bytes = -1;
            snprintf(probe.last_error, sizeof(probe.last_error), "%s", "Failed to ensure profiles directory");
            ESP_LOGE(TAG, "Failed to ensure profiles directory: %s", APP_SIMULATOR_DEFAULT_PROFILE_DIR);
            err = ESP_FAIL;
        }
    }

    app_state_write(apply_sdcard_probe, &probe);
    return probe.mounted ? ESP_OK : err;
}

esp_err_t sdcard_json_init(void)
{
    return sdcard_json_refresh_status();
}

esp_err_t sdcard_json_list_profiles(sdcard_json_profile_list_t *profiles)
{
    DIR *directory;
    struct dirent *entry;
    int count = 0;

    if (profiles == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(profiles, 0, sizeof(*profiles));
    if (sdcard_json_refresh_status() != ESP_OK) {
        return ESP_ERR_NOT_FOUND;
    }

    directory = opendir(APP_SIMULATOR_DEFAULT_PROFILE_DIR);
    if (directory == NULL) {
        return ESP_FAIL;
    }

    while (((entry = readdir(directory)) != NULL) && (count < APP_MAX_PROFILE_LIST_ITEMS)) {
        struct stat st;
        char path[APP_MAX_PROFILE_PATH_LEN];
        const size_t base_len = strlen(APP_SIMULATOR_DEFAULT_PROFILE_DIR);
        const size_t name_len = strnlen(entry->d_name, NAME_MAX);

        if (entry->d_name[0] == '.') {
            continue;
        }

        if (strstr(entry->d_name, ".json") == NULL) {
            continue;
        }

        if ((base_len + name_len + 1U) > sizeof(path)) {
            continue;
        }

        if ((name_len + 1U) > sizeof(profiles->items[count].name)) {
            continue;
        }

        memcpy(path, APP_SIMULATOR_DEFAULT_PROFILE_DIR, base_len);
        memcpy(path + base_len, entry->d_name, name_len);
        path[base_len + name_len] = '\0';
        if (stat(path, &st) != 0 || !S_ISREG(st.st_mode)) {
            continue;
        }

        memcpy(profiles->items[count].name, entry->d_name, name_len);
        profiles->items[count].name[name_len] = '\0';

        memcpy(profiles->items[count].path, path, base_len + name_len + 1U);
        profiles->items[count].size_bytes = (int64_t) st.st_size;
        profiles->items[count].modified_at_ms = (int64_t) st.st_mtime * 1000LL;
        count++;
    }

    closedir(directory);
    profiles->count = count;
    return ESP_OK;
}

esp_err_t sdcard_json_load_profile(const char *profile_path)
{
    FILE *file;
    long length;
    char *buffer = NULL;
    loaded_profile_result_t profile = {0};
    json_operation_update_t operation = {0};
    esp_err_t result = ESP_OK;

    if (profile_path == NULL || profile_path[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    operation.operation = JSON_OP_LOAD_PROFILE;
    operation.status = JSON_OP_STATUS_IN_PROGRESS;
    copy_string_bounded(operation.profile_path, sizeof(operation.profile_path), profile_path);
    snprintf(operation.message, sizeof(operation.message), "%s", "Loading profile");
    app_state_write(update_json_operation, &operation);

    if (sdcard_json_refresh_status() != ESP_OK) {
        operation.status = JSON_OP_STATUS_ERROR;
        snprintf(operation.message, sizeof(operation.message), "%s", "SDCard is unavailable");
        app_state_write(update_json_operation, &operation);
        return ESP_ERR_NOT_FOUND;
    }

    file = fopen(profile_path, "rb");
    if (file == NULL) {
        operation.status = JSON_OP_STATUS_ERROR;
        snprintf(operation.message, sizeof(operation.message), "%s", "Failed to open profile");
        app_state_write(update_json_operation, &operation);
        return ESP_ERR_NOT_FOUND;
    }

    fseek(file, 0, SEEK_END);
    length = ftell(file);
    fseek(file, 0, SEEK_SET);

    if (length <= 0) {
        fclose(file);
        operation.status = JSON_OP_STATUS_ERROR;
        snprintf(operation.message, sizeof(operation.message), "%s", "Profile is empty");
        app_state_write(update_json_operation, &operation);
        return ESP_ERR_INVALID_SIZE;
    }

    buffer = calloc(1, (size_t) length + 1U);
    if (buffer == NULL) {
        fclose(file);
        operation.status = JSON_OP_STATUS_ERROR;
        snprintf(operation.message, sizeof(operation.message), "%s", "Out of memory while reading profile");
        app_state_write(update_json_operation, &operation);
        return ESP_ERR_NO_MEM;
    }

    if (fread(buffer, 1, (size_t) length, file) != (size_t) length) {
        free(buffer);
        fclose(file);
        operation.status = JSON_OP_STATUS_ERROR;
        snprintf(operation.message, sizeof(operation.message), "%s", "Failed to read profile");
        app_state_write(update_json_operation, &operation);
        return ESP_FAIL;
    }
    fclose(file);

    result = parse_profile_json(buffer, &profile);
    free(buffer);
    if (result != ESP_OK) {
        operation.status = JSON_OP_STATUS_ERROR;
        snprintf(operation.message, sizeof(operation.message), "%s", "Invalid profile JSON");
        app_state_write(update_json_operation, &operation);
        return result;
    }

    copy_string_bounded(profile.profile_path, sizeof(profile.profile_path), profile_path);
    basename_from_path(profile_path, profile.profile_name, sizeof(profile.profile_name));
    app_state_write(apply_loaded_profile, &profile);

    operation.status = JSON_OP_STATUS_SUCCESS;
    snprintf(operation.message, sizeof(operation.message), "%s", "Profile loaded");
    app_state_write(update_json_operation, &operation);
    return ESP_OK;
}

esp_err_t sdcard_json_save_profile(const char *profile_path, bool overwrite)
{
    save_profile_state_t save_state = {0};
    cJSON *root = NULL;
    char *json_text = NULL;
    FILE *file;
    json_operation_update_t operation = {0};
    char resolved_path[APP_MAX_PROFILE_PATH_LEN];
    esp_err_t result = ESP_OK;

    operation.operation = JSON_OP_SAVE_PROFILE;
    operation.status = JSON_OP_STATUS_IN_PROGRESS;
    app_state_read(capture_profile_save_state, &save_state);

    if (profile_path == NULL || profile_path[0] == '\0') {
        copy_string_bounded(resolved_path, sizeof(resolved_path), save_state.active_profile_path);
    } else {
        copy_string_bounded(resolved_path, sizeof(resolved_path), profile_path);
    }

    copy_string_bounded(operation.profile_path, sizeof(operation.profile_path), resolved_path);
    snprintf(operation.message, sizeof(operation.message), "%s", "Saving profile");
    app_state_write(update_json_operation, &operation);

    if (sdcard_json_refresh_status() != ESP_OK) {
        operation.status = JSON_OP_STATUS_ERROR;
        snprintf(operation.message, sizeof(operation.message), "%s", "SDCard is unavailable");
        app_state_write(update_json_operation, &operation);
        return ESP_ERR_NOT_FOUND;
    }

    if (!overwrite && path_exists(resolved_path, NULL)) {
        operation.status = JSON_OP_STATUS_ERROR;
        snprintf(operation.message, sizeof(operation.message), "%s", "Profile already exists");
        app_state_write(update_json_operation, &operation);
        return ESP_ERR_INVALID_STATE;
    }

    root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "version", save_state.parameters.version);
    cJSON_AddNumberToObject(root, "start_latitude", save_state.parameters.start_latitude);
    cJSON_AddNumberToObject(root, "start_longitude", save_state.parameters.start_longitude);
    cJSON_AddNumberToObject(root, "altitude_m", save_state.parameters.altitude_m);
    cJSON_AddNumberToObject(root, "speed_kmh", save_state.parameters.speed_kmh);
    cJSON_AddNumberToObject(root, "heading_deg", save_state.parameters.heading_deg);
    cJSON_AddNumberToObject(root, "update_interval_ms", save_state.parameters.update_interval_ms);
    cJSON_AddNumberToObject(root, "satellites", save_state.parameters.satellites);
    cJSON_AddBoolToObject(root, "fix_valid", save_state.parameters.fix_valid);

    json_text = cJSON_Print(root);
    cJSON_Delete(root);
    if (json_text == NULL) {
        operation.status = JSON_OP_STATUS_ERROR;
        snprintf(operation.message, sizeof(operation.message), "%s", "Failed to serialize profile");
        app_state_write(update_json_operation, &operation);
        return ESP_ERR_NO_MEM;
    }

    file = fopen(resolved_path, "wb");
    if (file == NULL) {
        free(json_text);
        operation.status = JSON_OP_STATUS_ERROR;
        snprintf(operation.message, sizeof(operation.message), "%s", "Failed to open file for write");
        app_state_write(update_json_operation, &operation);
        return ESP_FAIL;
    }

    if (fwrite(json_text, 1, strlen(json_text), file) != strlen(json_text)) {
        result = ESP_FAIL;
    }
    fclose(file);
    free(json_text);

    if (result != ESP_OK) {
        operation.status = JSON_OP_STATUS_ERROR;
        snprintf(operation.message, sizeof(operation.message), "%s", "Failed to write profile");
        app_state_write(update_json_operation, &operation);
        return result;
    }

    loaded_profile_result_t loaded = {
        .parameters = save_state.parameters,
    };
    copy_string_bounded(loaded.profile_path, sizeof(loaded.profile_path), resolved_path);
    basename_from_path(resolved_path, loaded.profile_name, sizeof(loaded.profile_name));
    app_state_write(apply_loaded_profile, &loaded);

    operation.status = JSON_OP_STATUS_SUCCESS;
    snprintf(operation.message, sizeof(operation.message), "%s", "Profile saved");
    app_state_write(update_json_operation, &operation);
    return ESP_OK;
}
