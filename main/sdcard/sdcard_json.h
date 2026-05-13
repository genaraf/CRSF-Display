#ifndef SDCARD_JSON_H
#define SDCARD_JSON_H

#include <stdbool.h>
#include <stdint.h>

#include "app_config.h"
#include "esp_err.h"

typedef struct {
    char name[APP_MAX_PROFILE_NAME_LEN];
    char path[APP_MAX_PROFILE_PATH_LEN];
    int64_t size_bytes;
    int64_t modified_at_ms;
} sdcard_json_profile_item_t;

typedef struct {
    int count;
    sdcard_json_profile_item_t items[APP_MAX_PROFILE_LIST_ITEMS];
} sdcard_json_profile_list_t;

esp_err_t sdcard_json_init(void);
esp_err_t sdcard_json_refresh_status(void);
esp_err_t sdcard_json_list_profiles(sdcard_json_profile_list_t *profiles);
esp_err_t sdcard_json_load_profile(const char *profile_path);
esp_err_t sdcard_json_save_profile(const char *profile_path, bool overwrite);

#endif
